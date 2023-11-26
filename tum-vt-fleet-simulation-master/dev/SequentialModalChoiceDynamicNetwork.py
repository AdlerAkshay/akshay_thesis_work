# -------------------------------------------------------------------------------------------------------------------- #
# standard distribution imports
# -----------------------------
import logging
import importlib
import os
import json

# additional module imports (> requirements)
# ------------------------------------------

# src imports
# -----------
from src.FleetSimulationBase import FleetSimulationBase

# -------------------------------------------------------------------------------------------------------------------- #
# global variables
# ----------------
from src.misc.globals import *
LOG = logging.getLogger(__name__)

JUST_MIV_FOR_CALIBRATION = False
JUST_MIV_AND_PT_FOR_CALIBRATION = False

# -------------------------------------------------------------------------------------------------------------------- #
# functions
# ---------


# -------------------------------------------------------------------------------------------------------------------- #
# main
# ----
class SequentialModalChoiceDynamicNetworkSimulation(FleetSimulationBase):
    """
    Init main simulation module. Check the documentation for a flowchart of this particular simulation environment.
    Main attributes:
    - agent list per time step query public transport and fleet operator for offers and immediate decide
    - fleet operator offers ride pooling service
    - division of study area
        + first/last mile service in different parts of the study area
        + different parking costs/toll/subsidy in different parts of the study area
    """
    def add_init(self, scenario_parameters):
        """
        Simulation specific additional init.
        :param scenario_parameters: row of pandas data-frame; entries are saved as x["key"]
        """
        # units: park cost in cent, toll costs in cent/km -> derive cent/m value
        self.zones.set_current_park_costs(general_park_cost=scenario_parameters.get(G_PARK_COST_SCALE, 0))
        self.zones.set_current_toll_cost_scale_factor(scenario_parameters.get(G_TOLL_COST_SCALE, 0)/1000)

    def check_sim_env_spec_inputs(self, scenario_parameters):
        # TODO # throw errors if wrong demand is loaded
        if scenario_parameters.get(G_NETWORK_TYPE) != "NetworkDynamicNFDClusters":
            raise IOError("This simulation framework only works with 'NetworkDynamicNFDClusters' network/routing module!")
        if scenario_parameters.get(G_PT_TYPE) is None or scenario_parameters.get(G_GTFS_NAME) is None:
            raise IOError("This simulation framework requires a public transportation module!")

    def add_evaluate(self):
        # TODO # evaluate profit and social welfare at the end of the day
        # TODO # define costs, emissions of PV, PT, AMoD
        pass

    def run(self):
        """
        loop over time:
            # 1) update fleets, count moving fleet vehicles per cluster of last time steps
            # 2) current NFD factor, current PT crowding factor
            # get list of new travelers
            # loop over new travelers
                + check zones of origin and destination of traveler [first/last mile]
                    > private vehicle query
                    > public transport query
                    > query to list of fleets
                    > possibly intermodal query
                + mode choice: depending on decision, assignment changes
                    > route of chosen fleet vehicle
                    > public transport crowding dict
                    > street network density dict
            # periodic fleet optimization
                + pricing
        call self.evaluate()
        :return: (social welfare, list of [operator profits])
        """
        if not self._started:
            last_time = None
            self._started = True
            for sim_time in range(self.start_time, self.end_time+1, self.time_step):
                self.record_stats(force=True)
                if sim_time % 900 == 0:
                    hour = sim_time // 3600
                    minute = (sim_time%3600) // 60
                    time_str = f"{hour:0>2}:{minute:0>2}"
                    LOG.info(f"simulation time: {sim_time} ({time_str})")
                # 1)
                for op_obj in self.operators:
                    op_obj.time_trigger(sim_time)
                self.update_sim_state_fleets(sim_time-self.time_step, sim_time)
                # 2)
                self.routing_engine.update_network(sim_time)
                self.pt.update_pt_network(sim_time)
                # 3)
                list_new_traveler_rid_obj = []
                if last_time is not None:
                    # requests are saved in seconds internally
                    for t in range(last_time + 1, sim_time):
                        list_new_traveler_rid_obj.extend(self.demand.get_new_travelers(t))
                list_new_traveler_rid_obj.extend(self.demand.get_new_travelers(sim_time))
                last_time = sim_time
                # 4)
                for rid, rq_obj in list_new_traveler_rid_obj:
                    # 5)
                    o_node = rq_obj.get_origin_node()
                    o_pos = self.routing_engine.return_node_position(o_node)
                    d_node = rq_obj.get_destination_node()
                    d_pos = self.routing_engine.return_node_position(d_node)
                    # 5a) private vehicle
                    LOG.debug(f"Request {rid}: Checking PV option ...")
                    direct_route = self.routing_engine.return_best_route_1to1(o_pos, d_pos)
                    (arrival_time, distance) = self.routing_engine.return_route_infos(direct_route, 0.0, sim_time)
                    external_pv_costs, toll_costs, parking_costs = \
                        self.zones.get_external_route_costs(self.routing_engine, sim_time, direct_route,
                                                            park_origin=True, park_destination=True)
                    t_access, t_egress = self.zones.get_parking_average_access_egress_times(o_node, d_node)
                    pv_option = {G_OFFER_DRIVE:arrival_time-sim_time, G_OFFER_DIST: distance,
                                 G_OFFER_ACCESS_W:t_access, G_OFFER_EGRESS_W:t_egress, G_OFFER_TOLL:toll_costs,
                                 G_OFFER_PARK:parking_costs}
                    if pv_option:
                        rq_obj.receive_offer(G_MC_DEC_PV, pv_option)
                    # 5b) public transport
                    if not JUST_MIV_FOR_CALIBRATION:
                        LOG.debug(f"Request {rid}: Checking PT option ...")
                        pt_offer = self.pt.query(o_node, d_node, sim_time)
                        if pt_offer:
                            rq_obj.receive_offer(G_MC_DEC_PT, pt_offer)
                        # 5c) MoD
                    if not JUST_MIV_FOR_CALIBRATION or not JUST_MIV_AND_PT_FOR_CALIBRATION:
                        for op_id in range(self.n_op):
                            LOG.debug(f"Request {rid}: Checking AMoD option of operator {op_id} ...")
                            amod_offer = self.operators[op_id].user_request(rq_obj, sim_time)
                            if amod_offer:
                                rq_obj.receive_offer(op_id, amod_offer)
                        # 5d) intermodal MoD/PT solution
                        # > either do not use it or use one operator for all possibilities
                        # IM-operator-ID = - operator_id + G_MC_DEC_IM
                        # i) check if zones qualify for first/last mile
                        (mod_access, mod_egress) = self.zones.check_first_last_mile_option(o_node, d_node)
                        LOG.debug(f"Request {rid}: Checking intermodal options"
                                f" with (mod_access, mod_egress)={(mod_access, mod_egress)} ...")
                        list_check_sub_rq_op_tuples = self.simple_intermodal_query(rq_obj, sim_time, mod_access, mod_egress)

                    # 5e) mode choice and consequences
                    LOG.debug(f"Request {rid}: calling mode choice ...")
                    chosen_operator = rq_obj.choose_offer(self.scenario_parameters)
                    LOG.debug(f"Request {rid}: choice: {chosen_operator} ...")
                    if chosen_operator > G_MC_DEC_IM:
                        # 6) info to operators > update fleets
                        for op_id in range(self.n_op):
                            for sub_rq_op_tuple in list_check_sub_rq_op_tuples:
                                sub_rq, flm_op_id = sub_rq_op_tuple
                                im_rid = sub_rq.get_rid_struct()
                                if flm_op_id == op_id:
                                    self.operators[op_id].user_cancels_request(im_rid, sim_time)
                            if chosen_operator == op_id:
                                self.operators[op_id].user_confirms_booking(rid, sim_time)
                            else:
                                self.operators[op_id].user_cancels_request(rid, sim_time)
                        # 7) PT/PV chosen: assign traveler to respective network; remove rq from demand db
                        if chosen_operator == G_MC_DEC_PV:
                            # TODO # self.routing_engine.assign_route_to_network(rq_obj, sim_time)
                            # TODO # computation of route only when necessary
                            self.routing_engine.assign_route_to_network(direct_route, sim_time)
                            self.demand.user_chooses_PV(rid, sim_time)
                        elif chosen_operator == G_MC_DEC_PT:
                            pt_offer = rq_obj.return_offer(G_MC_DEC_PT)
                            pt_start_time = sim_time + pt_offer.get(G_OFFER_ACCESS_W, 0) + pt_offer.get(G_OFFER_WAIT, 0)
                            pt_end_time = pt_start_time + pt_offer.get(G_OFFER_DRIVE, 0)
                            self.pt.assign_to_pt_network(pt_start_time, pt_end_time)
                            self.demand.user_chooses_PT(rid, sim_time)
                    else:
                        # 6)
                        # IM-operator-ID = - operator_id + G_MC_DEC_IM
                        chosen_im_amod_operator = -1*(chosen_operator - G_MC_DEC_IM)
                        im_offer = rq_obj.return_offer(chosen_operator)
                        pt_start_time = im_offer[G_IM_OFFER_PT_START]
                        pt_end_time = im_offer[G_IM_OFFER_PT_END]
                        self.pt.assign_to_pt_network(pt_start_time, pt_end_time)
                        for op_id in range(self.n_op):
                            if chosen_im_amod_operator == op_id:
                                for sub_rq_op_tuple in list_check_sub_rq_op_tuples:
                                    sub_rq, flm_op_id = sub_rq_op_tuple
                                    im_rid = sub_rq.get_rid_struct()
                                    if flm_op_id == op_id:
                                        self.demand.user_chooses_intermodal(rid, sub_rq, op_id, sim_time, im_offer)
                                        self.operators[op_id].user_confirms_booking(im_rid, sim_time)
                            else:
                                for sub_rq_op_tuple in list_check_sub_rq_op_tuples:
                                    sub_rq, flm_op_id = sub_rq_op_tuple
                                    im_rid = sub_rq.get_rid_struct()
                                    if flm_op_id == op_id:
                                        self.operators[op_id].user_cancels_request(im_rid, sim_time)
            # save final state, record remaining travelers and vehicle tasks
            self.save_final_state()
            self.record_remaining_assignments()
        # call evaluation
        if self.scenario_parameters.get("evaluate"):
            self.evaluate()

    def _count_moving_vehicles(self, last_time, next_time):
        """
        This method counts the number of moving vehicles between last_time and next_time. For simplicity, it only counts
        the number of moving vehicles at last_time
        :param last_time:
        :param next_time:
        :return: zone_move_veh_dict {}: zone > number of driving vehicles in zone
        """
        zone_move_veh_dict = {}
        for veh_obj in self.sim_vehicles.values():
            if veh_obj.status in G_DRIVING_STATUS:
                zone = self.zones.get_zone_from_node(veh_obj.pos)
                try:
                    zone_move_veh_dict[zone] += 1
                except KeyError:
                    zone_move_veh_dict[zone] = 1
        return zone_move_veh_dict

