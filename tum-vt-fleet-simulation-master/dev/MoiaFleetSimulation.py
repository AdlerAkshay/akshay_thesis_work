# -------------------------------------------------------------------------------------------------------------------- #
# standard distribution imports
# -----------------------------
import logging
import random

# additional module imports (> requirements)
# ------------------------------------------

# src imports
# -----------
from src.ImmediateDecisionsSimulation import ImmediateDecisionsSimulation

# -------------------------------------------------------------------------------------------------------------------- #
# global variables
# ----------------
from src.misc.globals import *
LOG = logging.getLogger(__name__)

# TODO # delete?

# -------------------------------------------------------------------------------------------------------------------- #
# functions
# ---------

def routing_min_distance_cost_function(travel_time, travel_distance, current_node_index):
    """computes the customized section cost for routing (input for routing functions)

    :param travel_time: travel_time of a section
    :type travel time: float
    :param travel_distance: travel_distance of a section
    :type travel_distance: float
    :param current_node_index: index of current_node_obj in dijkstra computation to be settled
    :type current_node_index: int
    :return: travel_cost_value of section
    :rtype: float
    """
    return travel_distance

# -------------------------------------------------------------------------------------------------------------------- #
# main
# ----
class MoiaFleetSimulation(ImmediateDecisionsSimulation):
    """
    This fleetsimulation will be used for the MOIA-project # TODO # work in progress
    """
    def add_init(self, scenario_parameters):
        """
        Simulation specific additional init.
        :param scenario_parameters: row of pandas data-frame; entries are saved as x["key"]
        """
        super().add_init(scenario_parameters)

    def add_evaluate(self):
        """Runs standard and simulation environment specific evaluations over simulation results."""
        output_dir = self.dir_names[G_DIR_OUTPUT]
        from src.evaluation.MOIA_temporal import temporal_evaluation
        temporal_evaluation(output_dir)

    def update_sim_state_fleets(self, last_time, next_time, force_update_plan=False):
        """
        This method updates the simulation vehicles, records, ends and starts tasks and returns some data that
        will be used for additional state updates (fleet control information, demand, network, ...)
        :param last_time: simulation time before the state update
        :param next_time: simulation time of the state update
        :param force_update_plan: flag that can force vehicle plan to be updated
        """
        LOG.debug(f"updating MoD state from {last_time} to {next_time}")
        for opid_vid_tuple, veh_obj in self.sim_vehicles.items():
            op_id, vid = opid_vid_tuple
            boarding_requests, alighting_requests, passed_VRL, dict_start_alighting =\
                veh_obj.update_veh_state(last_time, next_time)
            for rid, boarding_time_and_pos in boarding_requests.items():
                boarding_time, boarding_pos = boarding_time_and_pos
                LOG.debug(f"rid {rid} boarding at {boarding_time} at pos {boarding_pos}")
                rq_obj = self.demand.rq_db[rid]
                walking_distance = self._return_walking_distance(rq_obj.o_pos, boarding_pos)
                t_access = walking_distance * self.scenario_parameters[G_WALKING_SPEED]
                self.demand.record_boarding(rid, vid, op_id, boarding_time, pu_pos=boarding_pos, t_access=t_access)
                self.operators[op_id].acknowledge_boarding(rid, vid, boarding_time)
            for rid, alighting_start_time_and_pos in dict_start_alighting.items():
                # record user stats at beginning of alighting process
                alighting_start_time, alighting_pos = alighting_start_time_and_pos
                LOG.debug(f"rid {rid} deboarding at {alighting_start_time} at pos {alighting_pos}")
                rq_obj = self.demand.rq_db[rid]
                walking_distance = self._return_walking_distance(rq_obj.d_pos, alighting_pos)
                t_egress = walking_distance * self.scenario_parameters[G_WALKING_SPEED]
                self.demand.record_alighting_start(rid, vid, op_id, alighting_start_time, do_pos=alighting_pos, t_egress=t_egress)
            for rid, alighting_end_time in alighting_requests.items():
                # # record user stats at end of alighting process
                self.demand.user_ends_alighting(rid, vid, op_id, alighting_end_time)
                self.operators[op_id].acknowledge_alighting(rid, vid, alighting_end_time)
            # send update to operator
            self.operators[op_id].receive_status_update(vid, next_time, passed_VRL, force_update_plan)
        # TODO # after ISTTT: live visualization: send vehicle states (self.live_visualization_flag==True)

    def _return_walking_distance(self, origin_pos, target_pos):
        """ returns the walking distance from an origin network position to an target network position
        :param origin_pos: network position of origin
        :param target_pos: network position of target
        :return: walking distance in m
        """
        walking_distance = None
        _, _, dis1 = self.routing_engine.return_travel_costs_1to1(origin_pos, target_pos, customized_section_cost_function = routing_min_distance_cost_function)
        _, _, dis2 = self.routing_engine.return_travel_costs_1to1(target_pos, origin_pos, customized_section_cost_function = routing_min_distance_cost_function)
        if dis1 < dis2:
            walking_distance = dis1
        else:
            walking_distance = dis2
        return walking_distance


# for bugfix of multimodal

class MoiaTestMultiModalFleetsim(MoiaFleetSimulation):

    def run(self):
        """
        loop over time:
            # 1) update fleets, count moving fleet vehicles per cluster of last time steps
            # 2) get new travelers
            # 3) call ride pooling optimisation
        """
        if not self._started:
            last_time = None
            self._started = True
            unassigned_rqs = {}
            for sim_time in range(self.start_time, self.end_time, self.time_step):
                print(sim_time)
                # 1)
                self.update_sim_state_fleets(sim_time-self.time_step, sim_time, force_update_plan=True)
                # 2)
                new_travel_times = self.routing_engine.update_network(sim_time)
                if new_travel_times:
                    self.update_vehicle_routes(sim_time)
                    for op in self.operators:
                        op.inform_network_travel_time_update(sim_time)
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
                    # 5c) MoD
                    #offers = {}
                    other_rq_states = [G_RQ_STATE_FIRSTMILE, G_RQ_STATE_LASTMILE]
                    fm_rq_obj = rq_obj.create_SubTripRequest(1, mod_o_node=None, mod_d_node=None, mod_start_time=None, modal_state = G_RQ_STATE_FIRSTMILE)
                    self.demand.rq_db[fm_rq_obj.get_rid_struct()] = fm_rq_obj
                    lm_rq_obj = rq_obj.create_SubTripRequest(2, mod_o_node=None, mod_d_node=None, mod_start_time=sim_time + 1800, modal_state = G_RQ_STATE_LASTMILE)
                    self.demand.rq_db[lm_rq_obj.get_rid_struct()] = lm_rq_obj
                    for a_rq_obj in [rq_obj, fm_rq_obj, lm_rq_obj]:
                        for op_id in range(self.n_op):
                            LOG.debug(f"Request {a_rq_obj.get_rid_struct()}: Checking AMoD option of operator {op_id} ...")
                            amod_offer = self.operators[op_id].user_request(a_rq_obj, sim_time)
                            LOG.debug(f"amod offer {amod_offer}")
                            if amod_offer is not None:
                                #offers[op_id] = amod_offer
                                a_rq_obj.receive_offer(op_id, amod_offer, sim_time)
                    
                    rqs = [rq_obj, fm_rq_obj, lm_rq_obj]
                    accept_id = random.randint(0, 2)
                    # accept sub rid first
                    a_rq_obj = rqs[accept_id]
                    a_rid = a_rq_obj.get_rid_struct()
                    chosen_operator = a_rq_obj.choose_offer(self.scenario_parameters)
                    LOG.debug(f" -> chosen operator: {chosen_operator} ({a_rid})")
                    for i, operator in enumerate(self.operators):
                        if i != chosen_operator:
                            operator.user_cancels_request(a_rid, sim_time)
                        else:
                            operator.user_confirms_booking(a_rid, sim_time)
                    if chosen_operator is None or chosen_operator < 0:
                        self.demand.record_user(a_rid)
                    # decline others
                    for y, a_rq_obj in enumerate(rqs):
                        if y == accept_id:
                            continue
                        a_rid = a_rq_obj.get_rid_struct()
                        chosen_operator = a_rq_obj.choose_offer(self.scenario_parameters)
                        chosen_operator = None
                        LOG.debug(f" -> chosen operator: {chosen_operator} ({a_rid})")
                        for i, operator in enumerate(self.operators):
                            if i != chosen_operator:
                                operator.user_cancels_request(a_rid, sim_time)
                            else:
                                operator.user_confirms_booking(a_rid, sim_time)
                        # TODO # GLOBAL FRAMEWORK: make higher level functionality?
                        if chosen_operator is None or chosen_operator < 0:
                            self.demand.record_user(a_rid)

                #exit()
                for op in self.operators:
                    op.time_trigger(sim_time)

                self.record_stats()

            # save final state, record remaining travelers and vehicle tasks
            self.save_final_state()
            self.record_remaining_assignments()
        # call evaluation
        self.evaluate()