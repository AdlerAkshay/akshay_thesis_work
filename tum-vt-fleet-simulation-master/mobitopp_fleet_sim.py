# standard imports
import os
import sys
import logging
import traceback
import time

# further imports
import pandas as pd

# fleet-sim package imports
from src.com.mobitopp import MobiToppSocket
from src.FleetSimulationBase import FleetSimulationBase, TravellerOffer, Rejection
# -------------------------------------------------------------------------------------------------------------------- #
# global variables
from src.misc.globals import *
import src.misc.config as config
LOG = logging.getLogger(__name__)


# -------------------------------------------------------------------------------------------------------------------- #
# help functions
def read_yaml_inputs(yaml_file):
    """This function transforms the yaml_file input into a dictionary and changes keys if necessary"""
    return_dict = config.ConstantConfig(yaml_file)
    return return_dict

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
# main class
class MobiToppFleetSimulation(FleetSimulationBase):
    def __init__(self, scenario_parameters):
        super().__init__(scenario_parameters)
        init_status = 0
        # initialization of communication socket
        self.com = MobiToppSocket(self, init_status)
        # TODO # additional initializations necessary?
        # only one operator
        self.fs_time = self.scenario_parameters[G_SIM_START_TIME]
        self.moia = self.operators[0]

        self.max_im_offer_proc = self.scenario_parameters.get(G_MAX_IM_OF_PROC)

        self.offer_id_to_agent_id_and_subrid = {}   # offer_id -> (agent_id, subrid) to synchronize mono/first/lastmile requests (agent_id should be the same for all entries!)
        self.current_offer_id = 1
        self.fltctrl_registered_subrids = {}    # sub_rid -> 1 if request has been forwarded to fleetctrl

    @staticmethod
    def get_directory_dict(scenario_parameters):
        """
        This function provides the correct paths to certain data according to the specified data directory structure.
        :param scenario_parameters: simulation input (pandas series)
        :return: dictionary with paths to the respective data directories
        """
        # TODO # include zones and forecasts later on
        # study_name = scenario_parameters[G_STUDY_NAME]
        scenario_name = scenario_parameters[G_SCENARIO_NAME]
        network_name = scenario_parameters[G_NETWORK_NAME]
        demand_name = scenario_parameters.get(G_DEMAND_NAME, None)
        zone_name = scenario_parameters.get(G_ZONE_SYSTEM_NAME, None)
        infra_name = scenario_parameters.get(G_INFRA_NAME, None)
        fc_type = scenario_parameters.get(G_FC_TYPE, None)
        fc_t_res = scenario_parameters.get(G_FC_TR, None)
        # gtfs_name = scenario_parameters.get(G_GTFS_NAME, None)
        #
        dirs = {}
        dirs[G_DIR_MAIN] = os.path.abspath(os.path.join(__file__, os.path.pardir, os.path.pardir, os.path.pardir,
                                                        os.path.pardir))
        dirs[G_DIR_DATA] = os.path.join(dirs[G_DIR_MAIN], "data", "fleetsim")
        #dirs[G_DIR_OUTPUT] = os.path.join(dirs[G_DIR_MAIN], "output", "results", "simulation", scenario_name)
        dirs[G_DIR_OUTPUT] = os.path.join(dirs[G_DIR_MAIN], "output", scenario_name, "simulation-fleetsim")
        dirs[G_DIR_NETWORK] = os.path.join(dirs[G_DIR_DATA], "networks", network_name)
        dirs[G_DIR_VEH] = os.path.join(dirs[G_DIR_DATA], "vehicles")
        dirs[G_DIR_FCTRL] = os.path.join(dirs[G_DIR_DATA], "fleetctrl")
        if infra_name is not None:
            dirs[G_DIR_INFRA] = os.path.join(dirs[G_DIR_DATA], "infra", infra_name, network_name)
        # dirs[G_DIR_DEMAND] = os.path.join(dirs[G_DIR_DATA], "demand", demand_name, "matched", network_name)
        if zone_name is not None:
            dirs[G_DIR_ZONES] = os.path.join(dirs[G_DIR_DATA], "zones", zone_name, network_name)
            if fc_type is not None and fc_t_res is not None:
                dirs[G_DIR_FC] = os.path.join(dirs[G_DIR_DATA], "demand", demand_name, "aggregated", zone_name, str(fc_t_res))
        # if gtfs_name is not None:
        #     dirs[G_DIR_PT] = os.path.join(dirs[G_DIR_DATA], "pubtrans", gtfs_name)
        return dirs

    def run(self):
        # simulation is controlled by mobitopp triggers
        self.com.keep_socket_alive()
        # save final state, record remaining travelers and vehicle tasks
        self.save_final_state()
        self.record_remaining_assignments()
        # call fleet evaluation
        self.evaluate()

    def check_sim_env_spec_inputs(self, scenario_parameters):
        return scenario_parameters

    def add_init(self, scenario_parameters):
        super().add_init(scenario_parameters)

    def add_evaluate(self):
        output_dir = self.dir_names[G_DIR_OUTPUT]
        from src.evaluation.MOIA_temporal import temporal_evaluation
        temporal_evaluation(output_dir)

    def create_RP_offer(self, agent_id, o_node, d_node, simulation_time, earliest_pickup_time, number_passengers):
        """This method calls the fleet control to create an offer for a new request.

        :param agent_id:
        :param o_node:
        :param d_node:
        :param simulation_time:
        :param earliest_pickup_time:
        :param number_passengers:
        :return: offer dictionary
        """
        t = time.time()
        rq_info_dict = {G_RQ_ID: agent_id, G_RQ_ORIGIN: o_node, G_RQ_DESTINATION: d_node, G_RQ_TIME: simulation_time,
                        G_RQ_EPT: earliest_pickup_time, G_RQ_PAX: number_passengers}
        if o_node != d_node and o_node >= 0 and d_node >= 0:
            rq_series = pd.Series(rq_info_dict)
            rq_series.name = agent_id
            rq_obj = self.demand.add_request(rq_series, self.current_offer_id, self.routing_engine, self.fs_time)
            self.moia.user_request(rq_obj, self.fs_time)
            self.fltctrl_registered_subrids[rq_obj.get_rid_struct()] = 1
            offer = self.moia.get_current_offer(rq_obj.get_rid_struct())
            if not offer.service_declined():
                offer.extend_offer({G_OFFER_ID : self.current_offer_id})
            rq_obj.receive_offer(0, offer, simulation_time)
            self.offer_id_to_agent_id_and_subrid[self.current_offer_id] = (self.current_offer_id, rq_obj.get_rid_struct())
            self.current_offer_id += 1
        else:
            LOG.info("automatic decline: no offer for {}".format(rq_info_dict))
            offer = Rejection(agent_id, 0)
        LOG.info("normal offer rid {} took {}".format(agent_id, time.time() - t))
        return offer

    def create_first_mile_RP_offer(self, agent_id, o_node, d_nodes_with_pt_tt, simulation_time, earliest_pickup_time, number_passengers):
        """This method calls the fleet control to create an offer for a new request.

        :param agent_id:
        :param o_node:
        :param d_nodes_with_pt_tt: list of tuple of (d_node, tt with public transport)
        :param simulation_time:
        :param earliest_pickup_time:
        :param number_passengers:
        :return: list of single offer dictionary (only with fastest overall travel time (rp + pt))
        """
        t = time.time()

        best_offer_value = float("inf")
        rq_obj_with_best_offer = None
        best_offer = Rejection(agent_id, 0)
        rqs_to_query = []

        if self.max_im_offer_proc is not None and len(d_nodes_with_pt_tt) > self.max_im_offer_proc:
            rq_guessed_tt = []
            for d_node, pt_tt in d_nodes_with_pt_tt:
                if o_node != d_node and o_node >= 0 and d_node >= 0:
                    if pt_tt > 700000:  # mehr als 8 tage (large int falls keine öv verbindung vorhanden)
                        continue
                    rq_info_dict = {G_RQ_ID: agent_id, G_RQ_ORIGIN: o_node, G_RQ_DESTINATION: d_node, G_RQ_TIME: simulation_time,
                                    G_RQ_EPT: earliest_pickup_time, G_RQ_PAX: number_passengers}
                    rq_series = pd.Series(rq_info_dict)
                    rq_series.name = agent_id
                    rq_obj = self.demand.add_request(rq_series, self.current_offer_id, self.routing_engine, self.fs_time, modal_state=G_RQ_STATE_FIRSTMILE)
                    ept, edt = self.moia.fast_intermediate_user_offer(rq_obj, self.fs_time)
                    if ept is not None:
                        rq_guessed_tt.append( (rq_obj, pt_tt + edt - ept, d_node, pt_tt, self.current_offer_id) )
                    self.current_offer_id += 1
            for i, entry in enumerate(sorted(rq_guessed_tt, key = lambda x:x[1])):
                if i >= self.max_im_offer_proc:
                    break
                rq_obj, _, d_node, pt_tt, offer_id = entry
                self.moia.user_request(rq_obj, self.fs_time, calculate_parallel=True)
                self.fltctrl_registered_subrids[rq_obj.get_rid_struct()] = 1
                rqs_to_query.append( (rq_obj, d_node, pt_tt, offer_id) )
        else:
            for d_node, pt_tt in d_nodes_with_pt_tt:
                if o_node != d_node and o_node >= 0 and d_node >= 0:
                    if pt_tt > 700000:  # mehr als 8 tage (large int falls keine öv verbindung vorhanden)
                        continue
                    rq_info_dict = {G_RQ_ID: agent_id, G_RQ_ORIGIN: o_node, G_RQ_DESTINATION: d_node, G_RQ_TIME: simulation_time,
                                    G_RQ_EPT: earliest_pickup_time, G_RQ_PAX: number_passengers}
                    rq_series = pd.Series(rq_info_dict)
                    rq_series.name = agent_id
                    rq_obj = self.demand.add_request(rq_series, self.current_offer_id, self.routing_engine, self.fs_time, modal_state=G_RQ_STATE_FIRSTMILE)
                    self.moia.user_request(rq_obj, self.fs_time, calculate_parallel=True)
                    self.fltctrl_registered_subrids[rq_obj.get_rid_struct()] = 1
                    rqs_to_query.append( (rq_obj, d_node, pt_tt, self.current_offer_id) )
                    self.current_offer_id += 1
        for rq_obj, d_node, pt_tt, offer_id in rqs_to_query:
            offer = self.moia.get_current_offer(rq_obj.get_rid_struct())
            if not offer.service_declined():
                offer_value = pt_tt + offer.get(G_OFFER_ACCESS_W, 0) + offer.get(G_OFFER_EGRESS_W, 0) + offer[G_OFFER_DRIVE]
                if offer_value < best_offer_value:
                    best_offer_value = offer_value
                    rq_obj_with_best_offer = rq_obj
                    best_offer = offer
                    best_offer.extend_offer({G_OFFER_ID : offer_id, G_RQ_DESTINATION: d_node})
                    self.offer_id_to_agent_id_and_subrid[offer_id] = (agent_id, rq_obj.get_rid_struct())
        LOG.info("first mile offer rid {} took {}".format(agent_id, time.time() - t))
        if rq_obj_with_best_offer is not None:
            rq_obj_with_best_offer.receive_offer(0, best_offer, simulation_time)
            return [best_offer]
        else:
            return []

    def create_last_mile_RP_offer(self, agent_id, o_nodes_with_pt_tt, d_node, simulation_time, number_passengers):
        """This method calls the fleet control to create an offer for a new request.

        :param agent_id:
        :param o_nodes_with_pt_tt: list of tuple of (o_node, tt with public transport)
        :param d_node:
        :param simulation_time:
        :param number_passengers:
        :return: list of single offer dictionary (only with fastest overall travel time (rp + pt))
        """
        t = time.time()
        best_offer_value = float("inf")
        rq_obj_with_best_offer = None
        best_offer = Rejection(agent_id, 0)
        rqs_to_query = []

        if self.max_im_offer_proc is not None and len(o_nodes_with_pt_tt) > self.max_im_offer_proc:
            rq_guessed_tt = []
            for o_node, pt_tt in o_nodes_with_pt_tt:
                if o_node != d_node and o_node >= 0 and d_node >= 0:
                    if pt_tt > 700000:  # mehr als 8 tage (large int falls keine öv verbindung vorhanden)
                        continue
                    rq_info_dict = {G_RQ_ID: agent_id, G_RQ_ORIGIN: o_node, G_RQ_DESTINATION: d_node, G_RQ_TIME: simulation_time,
                                    G_RQ_EPT: simulation_time + pt_tt, G_RQ_PAX: number_passengers}
                    rq_series = pd.Series(rq_info_dict)
                    rq_series.name = agent_id
                    rq_obj = self.demand.add_request(rq_series, self.current_offer_id, self.routing_engine, self.fs_time, modal_state=G_RQ_STATE_LASTMILE)
                    ept, edt = self.moia.fast_intermediate_user_offer(rq_obj, self.fs_time)
                    if ept is not None:
                        rq_guessed_tt.append( (rq_obj, pt_tt + edt - ept, o_node, pt_tt, self.current_offer_id) )
                    self.current_offer_id += 1
            for i, entry in enumerate(sorted(rq_guessed_tt, key = lambda x:x[1])):
                if i >= self.max_im_offer_proc:
                    break
                rq_obj, _, o_node, pt_tt, offer_id = entry
                self.moia.user_request(rq_obj, self.fs_time, calculate_parallel=True)
                self.fltctrl_registered_subrids[rq_obj.get_rid_struct()] = 1
                rqs_to_query.append( (rq_obj, o_node, pt_tt, offer_id) )
        else:
            for o_node, pt_tt in o_nodes_with_pt_tt:
                if o_node != d_node and o_node >= 0 and d_node >= 0:
                    if pt_tt > 700000:  # mehr als 8 tage (large int falls keine öv verbindung vorhanden)
                        continue
                    rq_info_dict = {G_RQ_ID: agent_id, G_RQ_ORIGIN: o_node, G_RQ_DESTINATION: d_node, G_RQ_TIME: simulation_time,
                                    G_RQ_EPT: simulation_time + pt_tt, G_RQ_PAX: number_passengers}
                    rq_series = pd.Series(rq_info_dict)
                    rq_series.name = agent_id
                    rq_obj = self.demand.add_request(rq_series, self.current_offer_id, self.routing_engine, self.fs_time, modal_state=G_RQ_STATE_LASTMILE)
                    self.moia.user_request(rq_obj, self.fs_time, calculate_parallel=True)
                    self.fltctrl_registered_subrids[rq_obj.get_rid_struct()] = 1
                    rqs_to_query.append( (rq_obj, o_node, pt_tt, self.current_offer_id) )
                    self.current_offer_id += 1
        for rq_obj, o_node, pt_tt, offer_id in rqs_to_query:
            offer = self.moia.get_current_offer(rq_obj.get_rid_struct())
            if not offer.service_declined():
                offer_value = pt_tt + offer.get(G_OFFER_ACCESS_W, 0) + offer.get(G_OFFER_EGRESS_W, 0) + offer[G_OFFER_DRIVE]
                if offer_value < best_offer_value:
                    best_offer_value = offer_value
                    rq_obj_with_best_offer = rq_obj
                    best_offer = offer
                    best_offer.extend_offer({G_OFFER_ID : offer_id, G_RQ_ORIGIN : o_node})
                    self.offer_id_to_agent_id_and_subrid[offer_id] = (agent_id, rq_obj.get_rid_struct())
        LOG.info("last mile offer rid {} took {}".format(agent_id, time.time() - t))
        if rq_obj_with_best_offer is not None:
            rq_obj_with_best_offer.receive_offer(0, best_offer, simulation_time)
            return [best_offer]
        else:
            return []

    def user_response(self, agent_id, chosen_offer_id):
        """
        This method either updates the current solution for a new customer or deletes the entries of an unsuccessful
        request. As this process should not block the listening device, these process should run in a parallel
        or asynchronous process.
        :param agent_id:
        :param chosen_offer_id: use id of offer; 0 means cancel of request
        """
        # TODO # discuss with KIT: who records MoD offers?
        if chosen_offer_id == 0:
            for sub_rid_id in self.fltctrl_registered_subrids.keys():
                self.moia.user_cancels_request(sub_rid_id, self.fs_time)
                self.demand.user_cancels_request(sub_rid_id, self.fs_time)
        else:
            _, chosen_sub_rid = self.offer_id_to_agent_id_and_subrid[chosen_offer_id]
            self.moia.user_confirms_booking(chosen_sub_rid, self.fs_time)
            for sub_rid_id in self.fltctrl_registered_subrids.keys():
                if sub_rid_id != chosen_sub_rid:
                    self.moia.user_cancels_request(sub_rid_id, self.fs_time)
                    self.demand.user_cancels_request(sub_rid_id, self.fs_time)
        self.offer_id_to_agent_id_and_subrid = {}
        self.fltctrl_registered_subrids = {}
        self.current_offer_id = 1

    def update_network(self, simulation_time):
        """
        This method updates all edge travel times in the network. Furthermore, all current V2RBs are updated, where
        all previously accepted solutions remain feasible.
        :param simulation_time:
        """
        new_travel_times = self.routing_engine.update_network(simulation_time) 
        if new_travel_times:
            self.update_vehicle_routes(simulation_time)
            self.moia.inform_network_travel_time_update(simulation_time)

    def optimize_fleet(self):
        """
        This method calls all operator functions to control the vehicle fleet. This includes an improved customer-
        assignment, moving illegally parked vehicles, charging vehicles and a vehicle repositioning strategy.
        This process is blocking.
        """
        # customer-vehicle assignment, move illegally parking vehicles, charging, vehicle repositioning
        self.moia.time_trigger(self.fs_time)

    def update_state(self, simulation_time):
        """
        This method increases the time of the fleet simulation (it receives the time of the next time step as
        argument), performs all vehicle processes, including boarding processes (and the resulting RV and RR deletions),
        and returns a list of customers that arrive at their destination in this time step.
        :param simulation_time:
        :return: list of agent-ids of arriving customers
        """
        # from FleetSimulation.update_sim_state_fleets()
        list_arrivals = []
        last_time = self.fs_time
        self.fs_time = simulation_time
        LOG.debug(f"updating MoD state from {last_time} to {self.fs_time}")
        for opid_vid_tuple, veh_obj in self.sim_vehicles.items():
            op_id, vid = opid_vid_tuple
            boarding_requests, alighting_requests, passed_VRL, dict_start_alighting =\
                veh_obj.update_veh_state(last_time, self.fs_time)
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
            # collect information for mobitopp
            list_arrivals.extend([self.demand.rq_db[rid] for rid in alighting_requests.keys()])
            for rid, alighting_end_time in alighting_requests.items():
                # record user stats at end of alighting process
                self.demand.user_ends_alighting(rid, vid, op_id, alighting_end_time)
                self.operators[op_id].acknowledge_alighting(rid, vid, alighting_end_time)
            # send update to operator
            self.operators[op_id].receive_status_update(vid, self.fs_time, passed_VRL, force_update=True)
        # return information for mobitopp
        self.record_stats()
        self.end_time = simulation_time
        #
        return list_arrivals

    def evaluate(self):
        """Runs standard and simulation environment specific evaluations over simulation results."""
        output_dir = self.dir_names[G_DIR_OUTPUT]
        # standard evaluation
        from src.evaluation.standard import standard_evaluation
        standard_evaluation(output_dir)
        from src.evaluation.MOIA_temporal import temporal_evaluation
        temporal_evaluation(output_dir)       

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


# -------------------------------------------------------------------------------------------------------------------- #
# script call
if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise IOError("Fleet simulation requires exactly one input parameter (*yaml configuration file)!")
    yaml_file = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir, os.path.pardir, os.path.pardir, sys.argv[1]))
    print(yaml_file)
    if not os.path.isfile(yaml_file) or not yaml_file.endswith("yaml"):
        prt_str = f"Configuration file {yaml_file} not found or with invalid file ending!"
        raise IOError(prt_str)
    # read configuration and bringt to standard input format
    scenario_parameter_dict = read_yaml_inputs(yaml_file)
    # initialize fleet simulation
    fs = MobiToppFleetSimulation(scenario_parameter_dict)
    # run simulation
    fs.run()
