# standard imports
import os
import sys
import logging
import traceback

# further imports
import pandas as pd
import numpy as np

# fleet-sim package imports
from dev.com.aimsun import AimsunSocket
from src.FleetSimulationBase import FleetSimulationBase, create_or_empty_dir
from src.simulation.Vehicles import SimulationVehicle
from src.simulation.Legs import VehicleRouteLeg
# -------------------------------------------------------------------------------------------------------------------- #
# global variables
from src.misc.globals import *
LOG = logging.getLogger(__name__)


class AimsunControlledSimulationVehicle(SimulationVehicle):
    def __init__(self, operator_id, vehicle_id, vehicle_data_dir, vehicle_type, routing_engine, rq_db, op_output,
                 record_route_flag, replay_flag):
        """ this class is used for a simulation vehicle which is moved within Aimsun. Therefore only boarding and waiting updates are performed within this class. 
        new routing tasks are sent to aimsun an position updates are only made when communicated by aimsun."""
        super().__init__(operator_id, vehicle_id, vehicle_data_dir, vehicle_type, routing_engine, rq_db, op_output,
                 record_route_flag, replay_flag)
        self.start_new_route_flag = False

    def start_next_leg(self, simulation_time):
        """
        This function resets the current task attributes of a vehicle. Furthermore, it returns a list of currently
        boarding requests.
        adoption for aimsun: if a driving vrl is started, the flag self.start_new_route_flag is set to send the corresponding route to aimsun
        :param simulation_time
        :return: list of currently boarding requests, list of starting to alight requests
        """
        if self.assigned_route and simulation_time >= self.assigned_route[0].earliest_start_time:
            #LOG.debug(f"Vehicle {self.vid} starts new VRL {self.assigned_route[0].__dict__} at time {simulation_time}")
            self.cl_start_time = simulation_time
            self.cl_start_pos = self.pos
            self.cl_start_soc = self.soc
            self.cl_driven_distance = 0.0
            self.cl_driven_route = []
            ca = self.assigned_route[0]
            self.status = ca.status
            if self.pos != ca.destination_pos or self.status in G_DRIVING_STATUS:
                if ca.route and self.pos[0] == ca.route[0]:
                    self.cl_remaining_route = ca.route
                else:
                    self.cl_remaining_route = self.routing_engine.return_best_route_1to1(self.pos, ca.destination_pos)
                try:
                    self.cl_remaining_route.remove(self.pos[0])
                except ValueError:
                    # TODO # check out after ISTTT
                    LOG.warning(f"First node in position {self.pos} not found in currently assigned route {ca.route}!")
                LOG.debug(f"start_next_leg: new_route vid {self.vid} pos {self.pos} route {self.cl_remaining_route}")
                self.cl_driven_route.append(self.pos[0])
                self.cl_driven_route_times.append(simulation_time)
                self.cl_remaining_time = None
                list_boarding_pax = []
                list_start_alighting_pax = []
                self.start_new_route_flag = True
            else:
                self.cl_remaining_route = []
                if ca.duration is not None:
                    self.cl_remaining_time = ca.duration
                    if ca.status in G_LOCK_DURATION_STATUS:
                        ca.locked = True
                else:
                    if not ca.locked:
                        # this will usually only happen for waiting tasks, which can be stopped at any time
                        self.cl_remaining_time = None
                    else:
                        raise AssertionError(f"Current locked task of vehicle {self.vid}"
                                             f"(operator {self.op_id}) has no stop criterion!")
                list_boarding_pax = [rq.get_rid_struct() for rq in ca.rq_dict.get(1, [])]
                list_start_alighting_pax = [rq.get_rid_struct() for rq in ca.rq_dict.get(-1, [])]
                for rq_obj in ca.rq_dict.get(1, []):
                    self.pax.append(rq_obj)
            LOG.debug("veh start next leg at time {}: {}".format(simulation_time, self))
            return list_boarding_pax, list_start_alighting_pax
        elif self.assigned_route and simulation_time < self.assigned_route[0].earliest_start_time:
            # insert waiting leg
            waiting_vrl = VehicleRouteLeg(4, self.pos, {}, duration=self.assigned_route[0].earliest_start_time - simulation_time)
            self.assigned_route = [waiting_vrl] + self.assigned_route
            LOG.debug("veh start next leg at time {}: {}".format(simulation_time, self))
            return self.start_next_leg(simulation_time)
        else:
            LOG.debug("veh start next leg at time {}: {}".format(simulation_time, self))
            return [], []

    def update_veh_state(self, current_time, next_time):
        """This method updates the current state of a simulation vehicle. This includes moving, boarding etc.
        The method updates the vehicle position, soc. Additionally, it triggers the end and start of VehicleRouteLegs.
        It returns a list of boarding request, alighting requests.
        adoption for aimsun: if the vehicle is currently driving, this function doesnt do anything since it is controlled in aimsun

        :param current_time: this time corresponds to the current state of the vehicle
        :type current_time: float
        :param next_time: the time until which the state should be updated
        :type next_time: float
        :return:(dict of boarding requests, dict of alighting request objects, list of passed VRL, dict_start_alighting)
        :rtype: list
        """
        #LOG.debug(f"update veh state {current_time} -> {next_time} : {self}")
        dict_boarding_requests = {}
        dict_start_alighting = {}
        dict_alighting_requests = {}
        list_passed_VRL = []
        c_time = current_time
        remaining_step_time = next_time - current_time
        if self.start_next_leg_first:
            add_boarding_rids, start_alighting_rids = self.start_next_leg(c_time)
            for rid in add_boarding_rids:
                dict_boarding_requests[rid] = c_time
            for rid in start_alighting_rids:
                dict_start_alighting[rid] = c_time
            self.start_next_leg_first = False
        #LOG.debug(f"veh update state from {current_time} to {next_time}")
        while remaining_step_time > 0:
            #LOG.debug(f" veh moving c_time {c_time} remaining time step {remaining_step_time}")
            if self.status in G_DRIVING_STATUS:
                # 1) moving is controlled externally
                remaining_step_time = 0

            elif self.status != 0:
                # 2) non-moving:
                #LOG.debug(f"Vehicle {self.vid} performs non-moving task between {c_time} and {next_time}")
                if remaining_step_time < self.cl_remaining_time:
                    #   a) duration is ongoing: do nothing
                    self.cl_remaining_time -= remaining_step_time
                    if self.assigned_route[0].power > 0:
                        self.soc += self.compute_soc_charging(self.assigned_route[0].power, remaining_step_time)
                        self.soc = max(self.soc, 1.0)
                    remaining_step_time = 0
                else:
                    #   b) duration is passed; end task, start next task; continue with remaining time
                    c_time += self.cl_remaining_time
                    #LOG.debug(f"cl remainig time {self.cl_remaining_time}")
                    remaining_step_time -= self.cl_remaining_time
                    if self.assigned_route[0].power > 0:
                        self.soc += self.compute_soc_charging(self.assigned_route[0].power, self.cl_remaining_time)
                        self.soc = max(self.soc, 1.0)
                    add_alighting_rq, passed_VRL = self.end_current_leg(c_time)
                    for rid in add_alighting_rq:
                        dict_alighting_requests[rid] = c_time
                    list_passed_VRL.append(passed_VRL)
                    if self.assigned_route:
                        add_boarding_rids, start_alighting_rids = self.start_next_leg(c_time)
                        for rid in add_boarding_rids:
                            dict_boarding_requests[rid] = c_time
                        for rid in start_alighting_rids:
                            dict_start_alighting[rid] = c_time
            else:
                # 3) idle without VRL
                remaining_step_time = 0
        return dict_boarding_requests, dict_alighting_requests, list_passed_VRL, dict_start_alighting

    def get_pos_and_route_to_start(self):
        """ this function is used to return the current pos of the vehicle and the route that is needed to be driven in the aimsun simulation
        if something is return (i.e. a new driven vrl needs to be started) is indicated by the flag self.start_new_route_flag
        :return: None, if no route has to be started; (network_position_tuple, node_index_list) otherwise
        """
        #LOG.debug(f"get_pos_and_route: {self.vid} | {self.start_new_route_flag} | {self.assigned_route} | {self.pos} | {self.cl_remaining_route}")
        if self.start_new_route_flag:
            self.start_new_route_flag = False
            if not self.assigned_route:
                if self.pos[1] is not None:
                    return self.pos, [self.pos[1]]
                else:
                    LOG.warning("new route to start after unassignment but on edge!")
                    return self.pos, [self.pos[0]]
            else:
                route = self.cl_remaining_route
                return self.pos, route
        else:
            return None

    def assign_vehicle_plan(self, list_route_legs, sim_time):
        """This method enables the fleet control modules to assign a plan to the simulation vehicles. It ends the
        previously assigned leg and starts the new one if necessary.
        adoption for aimsun: if a vehicle is currently driving, the leg is not ended, but the route to be driven is changed. the leg ends only, if the aimsun
        vehicle reaches its destination.
        :param list_route_legs: list of legs to assign to vehicle
        :type list_route_legs: list of VehicleRouteLeg
        """
        # transform rq from PlanRequest to SimulationRequest (based on RequestDesign)
        self.start_new_route_flag = False
        for vrl in list_route_legs:
            boarding_list = [self.rq_db[prq.get_rid_struct()] for prq in vrl.rq_dict.get(1,[])]
            alighting_list = [self.rq_db[prq.get_rid_struct()] for prq in vrl.rq_dict.get(-1,[])]
            vrl.rq_dict = {1:boarding_list, -1:alighting_list}
        #LOG.debug(f"Vehicle {self.vid} received new VRLs {[str(x) for x in list_route_legs]} at time {sim_time}")
        #LOG.debug(f"  -> current assignment: {self.assigned_route}")
        start_flag = True
        if self.assigned_route:
            if not list_route_legs or list_route_legs[0] != self.assigned_route[0]:
                if not self.assigned_route[0].locked:
                    if not self.status in G_DRIVING_STATUS:
                        self.end_current_leg(sim_time)
                    LOG.debug("unassigning vid!")
                else:
                    LOG.error("vid : {}".format(self.vid))
                    LOG.error("currently assigned: {}".format([str(x) for x in self.assigned_route]))
                    LOG.error("new: {}".format([str(x) for x in list_route_legs]))
                    LOG.error("current additional infos: {}".format(self.assigned_route[0].additional_str_infos()))
                    try:
                        LOG.error("new additional infos: {}".format(list_route_legs[0].additional_str_infos()))
                    except:
                        pass
                    raise AssertionError("assign_vehicle_plan(): Trying to assign new VRLs instead of a locked VRL.")
            else:
                start_flag = False

        self.assigned_route = list_route_legs
        if list_route_legs:
            if start_flag:
                self.start_next_leg_first = True
                if self.status in G_DRIVING_STATUS:
                    if not self.assigned_route[0].status in G_DRIVING_STATUS:
                        LOG.warning("while currently driving a new route without start driving vrl assigned {} {}".format(self, list_route_legs))
                        driving_vrl = VehicleRouteLeg(self.status, self.assigned_route[0].destination_pos, {})
                        self.assigned_route = [driving_vrl] + self.assigned_route[:]
        else: # check if vehicle needs to be stopped in aimsun control
            if self.status in G_DRIVING_STATUS:
                self.start_new_route_flag = True


    def update_current_position(self, new_pos):
        """ this function updates the current vehicle position when new information arrived from aimsun """
        self.pos = new_pos

    def recalculate_route(self):
        if self.status in G_DRIVING_STATUS:
            if not self.assigned_route:
                self.cl_remaining_route = []
            else:
                ca = self.assigned_route[0]
                self.cl_remaining_route = self.routing_engine.return_best_route_1to1(self.pos, ca.destination_pos)
            self.start_new_route_flag = True
        else:
            LOG.error("restarting should not happen for me! {}".format(self.vid))

    def reached_destination(self, simulation_time, route_with_times):
        """ this function is called, when the corresponding aimsun vehicle reaches its destination in aimsun
        :param simulation_time: time the vehicle reached destination
        :param route_with_times: list of tuple (node_index, time_of_passing) of driven route
        """
        driven_route = [r[0] for r in route_with_times]
        driven_route_times = [r[1] for r in route_with_times]
        if self.status in G_DRIVING_STATUS:

            if len(driven_route) > 1:
                self.pos = self.routing_engine.return_node_position(driven_route[-1])
                _, driven_distance = self.routing_engine.return_route_infos(driven_route, 0.0, 0.0)
            elif len(driven_route) == 1:
                try:
                    rel_pos = 0.0
                    if self.pos[2] != None:
                        rel_pos = self.pos[2]
                    _, driven_distance = self.routing_engine.return_route_infos([self.pos[0], driven_route[-1]], rel_pos, 0.0)
                except:
                    driven_distance = 0.0
                self.pos = self.routing_engine.return_node_position(driven_route[-1])
            else:
                LOG.debug(f"driven but not driven! {self} | {driven_route_times}")
                driven_distance = 0.0
                if self.pos[1] != None:
                    self.pos = self.routing_engine.return_node_position(self.pos[1])

            if len(self.assigned_route) == 0:
                LOG.debug("no assigned route but moved -> unassignment?")
            else:
                target_pos = self.assigned_route[0].destination_pos
                if self.pos != target_pos:
                    LOG.debug("reached destination but not at target {}: pos {} target pos {}".format(self, self.pos, target_pos))
                    r = self.routing_engine.return_best_route_1to1(self.pos, target_pos)
                    if len(r) <= 2:
                        LOG.debug("only one edge missed: if this is a turn, everything is fine! route: {}".format(r))
                        self.pos = target_pos
                        driven_route.append(target_pos[0])
                        if len(driven_route_times) > 0:
                            driven_route_times.append( driven_route_times[-1] )
                        else:
                            driven_route_times.append( (target_pos[0], simulation_time) )
                    else:
                        self.cl_driven_route.extend(driven_route)
                        self.cl_driven_distance += driven_distance
                        self.cl_driven_route_times.extend(driven_route_times)
                        self.cl_remaining_route = r
                        self.start_new_route_flag = True
                        return

            self.cl_driven_route.extend(driven_route)
            self.cl_driven_distance += driven_distance
            self.cl_driven_route_times.extend(driven_route_times)
            self.end_current_leg(simulation_time)
            self.start_next_leg_first = True
        else:
            if not len(driven_route) > 1:
                pass
            else:
                LOG.error("vehicle reached destination without performing routing VRL!")
                LOG.error("unassignment might be the reason?")
                LOG.error(f"sim time {simulation_time} route with time {route_with_times} | veh {self}")
                raise NotImplementedError

# -------------------------------------------------------------------------------------------------------------------- #
# main class
class AimsunFleetSimulation(FleetSimulationBase):
    def __init__(self, scenario_parameters):
        """ this class is used for a simulation coupled with the aimsun module
        the aimsun socked is used to communicate with an aimsun api script (folder APIs) to communicate with aimsun
        simulation triggers are coming from the aimsun socket

        different VRLs are controlled in different simulation environments:
            - routing legs are controlled in aimsun
            - waiting/boarding legs are controlled in this environment
        """
        super().__init__(scenario_parameters)
        init_status = 0
        # initialization of communication socket
        self.com = AimsunSocket(self, init_status)
        # TODO # additional initializations necessary?

        # in the following path dynamic network stats from aimsun are stored; also routing engines update their travel times from there
        # TODO # thats not that nice (if you change the path parallelization will not work anymore in alonso mora)
        self.dynamic_travel_time_data_path = os.path.join(self.dir_names[G_DIR_OUTPUT], "network_data", scenario_parameters[G_NETWORK_NAME])
        create_or_empty_dir(self.dynamic_travel_time_data_path)
        self.routing_engine.add_init_data(self.dynamic_travel_time_data_path)
        # only one operator
        self.fs_time = -1 #self.scenario_parameters[G_SIM_START_TIME]
        self.operator = self.operators[0]

        self.unassigned_rqs = {}

        # simulation vehicles and fleet control modules
        LOG.info("Initialization of MoD fleets... Exchange Vehicle Objects ... ")
        route_output_flag = self.scenario_parameters.get(G_SIM_ROUTE_OUT_FLAG, True)
        replay_flag = self.scenario_parameters.get(G_SIM_REPLAY_FLAG, False)
        op_to_vid_list = {}
        for op_vid, sim_vehicle in self.sim_vehicles.items():
            op_id, vid = op_vid
            new_aimsun_vehicle = AimsunControlledSimulationVehicle(op_id, vid, self.dir_names[G_DIR_VEH], sim_vehicle.veh_type, self.routing_engine, self.demand.rq_db, self.op_output[op_id], route_output_flag, replay_flag)
            try:
                op_to_vid_list[op_id].append(new_aimsun_vehicle)
            except:
                op_to_vid_list[op_id] = [new_aimsun_vehicle]
            self.sim_vehicles[op_vid] = new_aimsun_vehicle
        for op_id, list_sim_vehicles in op_to_vid_list.items():
            self.operators[op_id].sim_vehicles = list_sim_vehicles[:]

        # load initial state depending on init_blocking attribute
        LOG.info("Creating or loading initial vehicle states...")
        np.random.seed(self.scenario_parameters[G_RANDOM_SEED])
        self.load_initial_state()
        LOG.info(f"Initialization of scenario {self.scenario_name} successful.")

    def run(self):
        # simulation is controlled by aimsun triggers
        self.com.keep_socket_alive()
        # save final state, record remaining travelers and vehicle tasks
        self.save_final_state()
        # call fleet evaluation
        self.evaluate()

    def check_sim_env_spec_inputs(self, scenario_parameters):
        return scenario_parameters

    def add_init(self, scenario_parameters):
        super().add_init(scenario_parameters)

    def add_evaluate(self):
        pass

    def vehicles_reached_destination(self, simulation_time, vids_reached_destination):
        """ this function is triggered if fleet vehicles in aimsun reached its destination;
        updates vehicle states, triggers start of boarding processes, adds information to stats
        :param simulation_time: int time of simulation from aimsun
        :param vids_reached_destination: dict (opid, vid) -> list of (passed_node, time_of_passing)"""
        LOG.debug("vehicles reached destination")
        for opid_vid, route_with_times in vids_reached_destination.items():
            veh_obj = self.sim_vehicles[opid_vid]
            LOG.debug(f"veh {veh_obj} reached destination : {route_with_times}")
            veh_obj.reached_destination(simulation_time, route_with_times)

    def step(self, sim_time, vids_reached_destination = {}):
        """
        this function is triggered in a new simulation time step in aimsun 
        - request handling
        - optimisation trigger (vehicle locations needed, additional argument or new function?)
        - boarding/deboarding
        - starting new routes for aimsun vehicles
        - starting vehicles in aimsun for requests that have been declined
        :param vids_reached_destination: dict (opid, vid) -> list of (passed_node, time_of_passing)
        :return: vehicle routes to start: opid_vid_tuple -> (current_pos, node_list)
        """
        if self.fs_time < 0: # indicates that simulation started
            self.fs_time = sim_time
        # 0)
        self.vehicles_reached_destination(sim_time, vids_reached_destination)
        # 1)
        force_update = False
        if self.request_vehicle_position_update_from_aimsun(sim_time): # if new opt has been triggered in op (dont like it this way)
            force_update = True
        new_routes_to_start = self.update_sim_state_fleets(self.fs_time, sim_time, force_update_plan=force_update)
        # 2)
        self.routing_engine.update_network(self.fs_time) # TODO #

        last_time = self.fs_time
        if last_time < self.start_time:
            last_time = None
        list_new_traveler_rid_obj = self.demand.get_new_travelers(sim_time, since=last_time)

        # 3)
        for rid, rq_obj in list_new_traveler_rid_obj:
            for op_id in range(self.n_op):
                LOG.debug(f"Request {rid}: To operator {op_id} ...")
                self.operators[op_id].user_request(rq_obj, sim_time)

        # 4)
        self._check_waiting_request_cancellations(sim_time)

        # 5)
        for op_id, op_obj in enumerate(self.operators):
            # here offers are created in batch assignment
            op_obj.time_trigger(sim_time)

        # 6)
        list_requests_using_pv = [] # list (origin_node_id, destination_node_id) that will be started in aimsun
        for rid, rq_obj in self.demand.get_undecided_travelers(sim_time):
            for op_id in range(self.n_op):
                amod_offer = self.operators[op_id].get_current_offer(rid)
                LOG.debug(f"amod offer {amod_offer}")
                if amod_offer is not None:
                    rq_obj.receive_offer(op_id, amod_offer, sim_time)
            chosen_operator = self._rid_chooses_offer(self, rid, rq_obj, sim_time)

            if chosen_operator is None or chosen_operator < 0:  # TODO # operator id, if no operator is chosen?
                route = self.routing_engine.return_best_route_1to1(rq_obj.o_pos, rq_obj.d_pos)
                list_requests_using_pv.append( route )

        # retrieve new routes to start
        if self.request_vehicle_position_update_from_aimsun(sim_time): # if new opt has been triggered in op (dont like it this way)
            new_routes_to_start.update(self.update_sim_state_fleets(sim_time, sim_time, force_update_plan=True))

        self.record_stats()

        self.fs_time = sim_time
        return new_routes_to_start, list_requests_using_pv

    def update_network_travel_times(self, new_edge_travel_times):
        """ this function is triggered if new travel time statistics are available from aimsun
        the new edge travel times are stored in a travel time file in the results folder
        this file is then read by the routing engine
        :param new_edge_travel_times: dict edge_id (start_node_index, end_node_index) -> travel time
        """
        output_folder = os.path.join(self.dynamic_travel_time_data_path, str(self.fs_time))
        create_or_empty_dir(output_folder)
        tt_list = []
        for edge_id, edge_tt in new_edge_travel_times.items():
            from_node, to_node = edge_id
            tt_list.append({"from_node" : from_node, "to_node" : to_node, "edge_tt" : edge_tt})
        tt_df = pd.DataFrame(tt_list)
        tt_df.to_csv(os.path.join(output_folder, "edges_td_att.csv"), index=False)
        # 2)
        self.routing_engine.update_network(self.fs_time, update_state = True)
        for operator in self.operators:
            operator.inform_network_travel_time_update(self.fs_time)

    def update_sim_state_fleets(self, last_time, next_time, force_update_plan=False):
        """
        This method updates the simulation vehicles, records, ends and starts tasks and returns some data that
        will be used for additional state updates (fleet control information, demand, network, ...)
        :param last_time: simulation time before the state update
        :param next_time: simulation time of the state update
        :param force_update_plan: flag that can force vehicle plan to be updated
        :return: vehicle routes to start: opid_vid_tuple -> (current_pos, node_list)
        """
        LOG.debug(f"updating MoD state from {last_time} to {next_time}")
        new_routes_to_start = {}        # opid_vid_tuple -> node_list
        for opid_vid_tuple, veh_obj in self.sim_vehicles.items():
            op_id, vid = opid_vid_tuple
            boarding_requests, alighting_requests, passed_VRL, dict_start_alighting =\
                veh_obj.update_veh_state(last_time, next_time)
            for rid, boarding_time in boarding_requests.items():
                LOG.debug(f"rid {rid} boarding at {boarding_time}")
                self.demand.record_boarding(rid, vid, op_id, boarding_time)
                self.operators[op_id].acknowledge_boarding(rid, vid, boarding_time)
            for rid, alighting_start_time in dict_start_alighting.items():
                # record user stats at beginning of alighting process
                LOG.debug(f"rid {rid} deboarding at {alighting_start_time}")
                self.demand.record_alighting_start(rid, vid, op_id, alighting_start_time)
            for rid, alighting_end_time in alighting_requests.items():
                # # record user stats at end of alighting process
                self.demand.user_ends_alighting(rid, vid, op_id, alighting_end_time)
                self.operators[op_id].acknowledge_alighting(rid, vid, alighting_end_time)
            # send update to operator
            self.operators[op_id].receive_status_update(vid, next_time, passed_VRL, force_update_plan)

            pos_route_to_start = veh_obj.get_pos_and_route_to_start()
            if pos_route_to_start is not None:
                pos, route_to_start = pos_route_to_start
                new_routes_to_start[opid_vid_tuple] = (pos, route_to_start)
        return new_routes_to_start


    def request_vehicle_position_update_from_aimsun(self, simulation_time):
        """ this function tells if aimsun should send new vehicle locations for optimisation """
        return self.operator.vehicle_information_needed_for_optimisation(simulation_time)

    def update_vehicle_information(self, opid_vid_to_pos_dict, veh_to_reroute):
        """ this function translates the vehicle updates sent from aimsun und set the veh_objs
        :param opid_vid_to_pos_dict: dictionary (op_id, vid) -> network position
        :param veh_to_reroute: list veh ids that need a new routing order
        """
        for opid_vid, veh_pos in opid_vid_to_pos_dict.items():
            op_id, vid = opid_vid   # only one operator accepted currently (TODO)
            veh_obj = self.sim_vehicles[(op_id, vid)]
            veh_obj.update_current_position(veh_pos)
        for vid in veh_to_reroute:
            self.sim_vehicles[vid].recalculate_route()



