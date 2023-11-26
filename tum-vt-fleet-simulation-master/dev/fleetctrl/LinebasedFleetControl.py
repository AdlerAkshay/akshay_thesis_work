# -------------------------------------------------------------------------------------------------------------------- #
# standard distribution imports
# -----------------------------
from __future__ import annotations
import logging
import os
from abc import abstractmethod
from typing import Dict, List, TYPE_CHECKING

# additional module imports (> requirements)
# ------------------------------------------
import numpy as np
import pandas as pd

# src imports
# -----------
from src.simulation.Offers import Rejection, TravellerOffer
from src.fleetctrl.FleetControlBase import FleetControlBase, get_assigned_rids_from_vehplan
from src.fleetctrl.planning.VehiclePlan import VehiclePlan, PlanStop
from src.fleetctrl.planning.PlanRequest import PlanRequest

# -------------------------------------------------------------------------------------------------------------------- #
# global variables
# ----------------
from src.misc.globals import *
LOG = logging.getLogger(__name__)
LARGE_INT = 100000

if TYPE_CHECKING:
    from src.routing.NetworkBase import NetworkBase
    from src.simulation.Vehicles import SimulationVehicle

# -------------------------------------------------------------------------------------------------------------------- #
# functions
# ---------
def create_stations(columns):
    return PTStation(columns["station_id"], columns["network_node_index"])


# -------------------------------------------------------------------------------------------------------------------- #
# class definitions
# -----------------

INPUT_PARAMETERS_LinebaseFleetControl = {
    "doc" : "this fleetcontrol class is used to model a fleet of vehicles serving customers on fixed lines",
    "inherit" : "FleetControlBase",
    "input_parameters_mandatory": [G_GTFS_NAME, G_PT_SCHEDULE_F],
    "input_parameters_optional": [G_PT_FARE_B, G_WALKING_SPEED],
    "mandatory_modules": [],
    "optional_modules": []
}

class PTStation:
    def __init__(self, station_id, street_network_node_id):
        self.station_id = station_id
        self.street_network_node_id = street_network_node_id


class PtLine():
    def __init__(self, line_id, pt_fleetcontrol_module, routing_engine : NetworkBase, sim_vehicle_dict, vid_to_schedule, sim_start_time):
        """
        :param pt_fleetcontrol_module:
        :param routing_engine: routing engine
        :param sim_vehicle_dict: dict veh_id -> simulaiton vehicle obj (for this line)
        :param vid_to_schedule: dict veh_id -> schedule_df
        """
        self.line_id = line_id
        self.pt_fleetcontrol_module : LinebasedFleetControl = pt_fleetcontrol_module
        self.routing_engine = routing_engine

        self.sim_vehicles : Dict[int, SimulationVehicle] = sim_vehicle_dict  # line_vehicle_id -> SimulationVehicle
        self.veh_plans : Dict[int, VehiclePlan] = {}     # line_vehicle_id -> vehicle plan

        self.node_index_to_station_id = {}  # node_index -> station id
        for vid, schedule_df in vid_to_schedule.items():
            list_plan_stops = []
            first_stop_in_stim_time_found = False
            for _, scheduled_stop in schedule_df.iterrows():
                station_id = scheduled_stop["station_id"]
                node_index = self.pt_fleetcontrol_module.station_dict[station_id].street_network_node_id
                earliest_departure_dict = {}
                if not np.isnan(scheduled_stop["departure"]):
                    earliest_departure_dict[-1] = scheduled_stop["departure"]
                ps = PlanStop(self.routing_engine.return_node_position(node_index), earliest_end_time=scheduled_stop["departure"])
                list_plan_stops.append(ps)
                self.node_index_to_station_id[node_index] = station_id
                LOG.debug(f"sim start time {sim_start_time} | earliest departure {earliest_departure_dict} | {scheduled_stop['departure']}")
                if earliest_departure_dict.get(-1, -1) < sim_start_time and not first_stop_in_stim_time_found:    # remove schedules before start time
                    list_plan_stops = []
                else:
                    first_stop_in_stim_time_found = True
            # init veh pos at first stop
            init_state = {
                G_V_INIT_NODE : list_plan_stops[0].get_pos()[0],
                G_V_INIT_SOC : 1,
                G_V_INIT_TIME : sim_start_time
            }
            LOG.debug(f"line vid {vid} with list plan stops: {[str(x) for x in list_plan_stops[:10]]}")
            self.sim_vehicles[vid].set_initial_state(self.pt_fleetcontrol_module, routing_engine, init_state, sim_start_time, veh_init_blocking=False)
            self.veh_plans[vid] = VehiclePlan(self.sim_vehicles[vid], sim_start_time, routing_engine, list_plan_stops)
            self.veh_plans[vid].update_plan(self.sim_vehicles[vid], sim_start_time, routing_engine, keep_time_infeasible=True)
            self.pt_fleetcontrol_module.assign_vehicle_plan(self.sim_vehicles[vid], self.veh_plans[vid], sim_start_time)

    def receive_status_update(self, vid, simulation_time, list_finished_VRL, force_update=True):
        """This method can be used to update plans and trigger processes whenever a simulation vehicle finished some
         VehicleRouteLegs.

        :param vid: vehicle id
        :type vid: int
        :param simulation_time: current simulation time
        :type simulation_time: float
        :param list_finished_VRL: list of VehicleRouteLeg objects
        :type list_finished_VRL: list
        :param force_update: indicates if also current vehicle plan feasibilities have to be checked
        :type force_update: bool
        """
        veh_obj = self.sim_vehicles[vid]
        self.veh_plans[vid].update_plan(veh_obj, simulation_time, self.routing_engine, list_finished_VRL)

    def query_travel_time_infos(self, o_pos, d_pos, earliest_start_time, nr_pax):
        """ this method will return waiting time and travel time between o_pos and d_pos starting from earliest start time
        if both positions are in the line with earliest arrival time
        :param o_pos: origin position
        :param d_pos: destination postion
        :param earliest_start_time: earliest starting time
        :param nr_pax: number of passengers
        :return: tuple of (waiting time, travel time, arrival time) if both nodes in line, None else
        """
        if self.node_index_to_station_id.get(o_pos[0]) is None or self.node_index_to_station_id.get(d_pos[0]) is None:
            return None
        best_arrival_time = float("inf")
        best_waiting = float("inf")
        best_travel_time = float("inf")
        for vid, veh_plan in self.veh_plans.items():
            first_stop_in_time_found = False
            pu_time = None
            do_time = None
            cur_pax = self.sim_vehicles[vid].get_nr_pax_without_currently_boarding()
            for i, ps in enumerate(veh_plan.list_plan_stops):
                cur_pax += ps.get_change_nr_pax()
                if ps.is_locked():
                    continue
                if not first_stop_in_time_found and ps.get_duration_and_earliest_departure()[1] >= earliest_start_time:
                    first_stop_in_time_found = True
                if first_stop_in_time_found:
                    if ps.get_pos() == o_pos:
                        if i < len(veh_plan.list_plan_stops) - 1:
                            if veh_plan.list_plan_stops[i+1].get_pos() == ps.get_pos():
                                continue
                        if cur_pax + nr_pax > self.sim_vehicles[vid].max_pax:
                            continue
                        pu_time = ps.get_duration_and_earliest_departure()[1]
                        p_cur_pax = cur_pax + nr_pax
                        for j in range(i+1, len(veh_plan.list_plan_stops)):
                            ps = veh_plan.list_plan_stops[j]
                            if ps.get_pos() == d_pos and pu_time is not None:
                                do_time = ps.get_duration_and_earliest_departure()[1]
                                break
                            p_cur_pax += ps.get_change_nr_pax()
                            if p_cur_pax > self.sim_vehicles[vid].max_pax:
                                pu_time = None
                                break
                        if pu_time is not None and do_time is not None:
                            break
            if do_time is not None:
                if do_time < best_arrival_time:
                    best_arrival_time = do_time
                    best_waiting = pu_time - earliest_start_time
                    best_travel_time = do_time - pu_time
                elif do_time == best_arrival_time and do_time - pu_time < best_travel_time:
                    best_arrival_time = do_time
                    best_waiting = pu_time - earliest_start_time
                    best_travel_time = do_time - pu_time
        LOG.info("query travel time {} : at : {} wt: {} tt: {}".format(self.line_id, best_arrival_time, best_waiting, best_travel_time))
        return best_waiting, best_travel_time, best_arrival_time

    def assign_user(self, rid, simulation_time):
        rq = self.pt_fleetcontrol_module.rq_dict[rid]
        nr_pax = rq.nr_pax
        o_pos, earliest_start_time, _ = rq.get_o_stop_info()
        d_pos, _, _ = rq.get_d_stop_info()
        if self.node_index_to_station_id.get(o_pos[0]) is None or self.node_index_to_station_id.get(d_pos[0]) is None:
            raise NotImplementedError("line {} cant served rid {} | {}".format(self.line_id, rid, rq))
        best_arrival_time = float("inf")
        best_travel_time = float("inf")
        best_waiting = float("inf")
        best_vid = None
        best_o_ps_index = None
        best_d_ps_index = None
        for vid, veh_plan in self.veh_plans.items():
            first_stop_in_time_found = False
            pu_time = None
            do_time = None
            o_ps_index = None
            d_ps_index = None
            cur_pax = self.sim_vehicles[vid].get_nr_pax_without_currently_boarding()
            for i, ps in enumerate(veh_plan.list_plan_stops):
                cur_pax += ps.get_change_nr_pax()
                if ps.is_locked():
                    continue
                if not first_stop_in_time_found and ps.get_duration_and_earliest_departure()[1] >= earliest_start_time:
                    first_stop_in_time_found = True
                if first_stop_in_time_found:
                    if ps.get_pos() == o_pos:
                        if i < len(veh_plan.list_plan_stops) - 1:
                            if veh_plan.list_plan_stops[i+1].get_pos() == ps.get_pos():
                                continue
                        if cur_pax + nr_pax > self.sim_vehicles[vid].max_pax:
                            continue
                        pu_time = ps.get_duration_and_earliest_departure()[1]
                        o_ps_index = i
                        p_cur_pax = cur_pax + nr_pax
                        for j in range(i+1, len(veh_plan.list_plan_stops)):
                            ps = veh_plan.list_plan_stops[j]
                            if ps.get_pos() == d_pos and pu_time is not None:
                                do_time = ps.get_duration_and_earliest_departure()[1]
                                d_ps_index = j
                                break
                            p_cur_pax += ps.get_change_nr_pax()
                            if p_cur_pax > self.sim_vehicles[vid].max_pax:
                                pu_time = None
                                o_ps_index = None
                                break
                        if pu_time is not None and do_time is not None:
                            break
            if do_time is not None:
                if do_time < best_arrival_time:
                    best_arrival_time = do_time
                    best_waiting = pu_time - earliest_start_time
                    best_travel_time = do_time - pu_time
                    best_o_ps_index = o_ps_index
                    best_d_ps_index = d_ps_index
                    best_vid = vid
                elif do_time == best_arrival_time and do_time - pu_time < best_travel_time:
                    best_arrival_time = do_time
                    best_waiting = pu_time - earliest_start_time
                    best_travel_time = do_time - pu_time
                    best_o_ps_index = o_ps_index
                    best_d_ps_index = d_ps_index
                    best_vid = vid

        list_plan_stops = self.veh_plans[best_vid].list_plan_stops
        
        o_ps : PlanStop = list_plan_stops[best_o_ps_index]
        boarding_list = o_ps.get_list_boarding_rids() + [rid]
        new_boarding_dict = {1:boarding_list, -1:o_ps.get_list_alighting_rids()}
        new_o_ps = PlanStop(o_ps.get_pos(), boarding_dict=new_boarding_dict, earliest_end_time=o_ps.get_duration_and_earliest_departure()[1], change_nr_pax=o_ps.get_change_nr_pax() + rq.nr_pax)
        list_plan_stops[best_o_ps_index] = new_o_ps
        
        d_ps : PlanStop = list_plan_stops[best_d_ps_index]
        deboarding_list = d_ps.get_list_alighting_rids() + [rid]
        new_boarding_dict = {1:d_ps.get_list_boarding_rids(), -1:deboarding_list}
        new_d_ps = PlanStop(d_ps.get_pos(), boarding_dict=new_boarding_dict, earliest_end_time=d_ps.get_duration_and_earliest_departure()[1], change_nr_pax=d_ps.get_change_nr_pax() - rq.nr_pax)
        list_plan_stops[best_d_ps_index] = new_d_ps

        new_veh_plan = VehiclePlan(self.sim_vehicles[best_vid], simulation_time, self.routing_engine, list_plan_stops)
        self.pt_fleetcontrol_module.assign_vehicle_plan(self.sim_vehicles[best_vid], new_veh_plan, simulation_time)
        self.pt_fleetcontrol_module.rid_to_assigned_vid[rid] = best_vid

    def remove_rid_from_line(self, rid, assigned_vid, simulation_time):
        """ this function is called when a request is canceled and allready assigned to pt line -> remove rid from vehplans
        """
        raise NotImplementedError
            
class LinebasedFleetControl(FleetControlBase):
    """ this fleetcontrol class is used to model a fleet of vehicles serving customers on fixed lines
    stations are specified by the stations-file at pt_data_dir/{nw_name}/station.csv
    schedules are specified by a schedule-file at pt_data_dir/schedules.csv
    for each line in the schedules file a PTLine object is created aggregating vehicles for this line
    vehicles follow the specified lines and wait at stations for boarding-processes until the departure time specified in the schedule file
    to be able to travel with a line, origin and destination have to be covered by the line.
    requests will return the option with the earliest arrival time at the destination
    """
    def __init__(self, op_id, pt_data_dir, routing_engine, zone_system, scenario_parameters,
                 dir_names, op_charge_depot_infra=None,
                 list_pub_charging_infra = []):
        
        self.op_id = op_id
        self.base_fare = scenario_parameters.get(G_PT_FARE_B,0)
        self.walking_speed = scenario_parameters.get(G_WALKING_SPEED,0)
        self.routing_engine = routing_engine
        self.zones = zone_system
        station_node_f = os.path.join(pt_data_dir, scenario_parameters[G_NETWORK_NAME], "stations.csv")
        station_node_df = pd.read_csv(station_node_f)
        
        self.begin_approach_buffer_time = 0
        # self.station_dict = {}  # station_id -> PTStation
        tmp_station_dict = station_node_df.apply(create_stations, axis=1).to_dict()
        self.station_dict : Dict[int, PTStation] = {}
        for _, pt_station in tmp_station_dict.items():
            self.station_dict[pt_station.station_id] = pt_station
        # creation of additional access options to pt-station objects
        self.st_nw_stations : Dict[int, List[int]] = {}   # street nw node id -> list station ids
        tmp_st_nw_stations = {}
        for station_obj in self.station_dict.values():
            try:
                tmp_st_nw_stations[station_obj.street_network_node_id].append(station_obj)
            except KeyError:
                tmp_st_nw_stations[station_obj.street_network_node_id] = [station_obj]
        # sort such that rail stations are first
        for k,v in tmp_st_nw_stations.items():
            self.st_nw_stations[k] = v

        # init line informations
        # schedules are read and the vehicles that have to be created in the fleetsimulation class are collected
        schedules = pd.read_csv(os.path.join(pt_data_dir, scenario_parameters[G_PT_SCHEDULE_F]))
        pt_vehicle_id = 0
        pt_line_specifications_list = []
        self.vehicles_to_initialize = {}    # pt_vehicle_id -> veh_type
        self.schedule_to_initialize = {}    # line -> pt_vehicle_id -> schedule_df
        for key, vehicle_line_schedule in schedules.groupby(["LINE", "line_vehicle_id", "vehicle_type"]):
            line, line_vehicle_id, vehicle_type = key
            pt_line_specifications_list.append({"line" : line, "line_vehicle_id" : line_vehicle_id, "vehicle_type" : vehicle_type, "sim_vehicle_id" : pt_vehicle_id})
            self.vehicles_to_initialize[pt_vehicle_id] = vehicle_type
            if self.schedule_to_initialize.get(line) is None:
                self.schedule_to_initialize[line] = {}
            self.schedule_to_initialize[line][pt_vehicle_id] = vehicle_line_schedule
            pt_vehicle_id += 1

        # line specification output
        pd.DataFrame(pt_line_specifications_list).to_csv(os.path.join(dir_names[G_DIR_OUTPUT], f"3-{self.op_id}_pt_vehicles.csv"), index=False)

        # PT lines
        self.PT_lines : Dict[int, PtLine] = {}  # line -> PtLine obj
        self.pt_vehicle_to_line = {}    # pt_veh_id -> line
        self.sim_time = -1

        #
        self.rq_dict = {}
        self.routing_engine = routing_engine
        self.zones = zone_system
        self.dyn_output_dict = {}
        self.rid_to_assigned_vid = {}
        #
        self._vid_to_assigned_charging_process = {}
        self.veh_plans = {}
        self.dyn_fleet_sizing = None
        self.repo = None

    def return_vehicles_to_initialize(self) -> Dict[int, str]:
        """
        return vehicles that have to be initialized int the fleetsimulation class
        :return dict pt_vehicle_id -> veh_type
        """
        return self.vehicles_to_initialize

    def continue_init(self, sim_vehicle_objs, sim_start_time):
        """
        this method continuoes initialization after simulation vehicles have been created in the fleetsimulation class
        :param sim_vehicle_objs: ordered list of sim_vehicle_objs
        :param sim_start_time: simulation start time
        """
        self.sim_time = sim_start_time
        veh_obj_dict = {veh.vid : veh for veh in sim_vehicle_objs}
        for line, vid_to_schedule_dict in self.schedule_to_initialize.items():
            for vid in vid_to_schedule_dict.keys():
                self.pt_vehicle_to_line[vid] = line
            schedule_vehicles = {vid : veh_obj_dict[vid] for vid in vid_to_schedule_dict.keys()}
            self.PT_lines[line] = PtLine(line, self, self.routing_engine, schedule_vehicles, vid_to_schedule_dict, sim_start_time)

    def assign_vehicle_plan(self, veh_obj, vehicle_plan, sim_time, force_assign=False, assigned_charging_task=None, add_arg=None):
        """ this method should be used to assign a new vehicle plan to a vehicle

        WHEN OVERWRITING THIS FUNCTION MAKE SURE TO CALL AT LEAST THE LINES BELOW (i.e. super())

        :param veh_obj: vehicle obj to assign vehicle plan to
        :type veh_obj: SimulationVehicle
        :param vehicle_plan: vehicle plan that should be assigned
        :type vehicle_plan: VehiclePlan
        :param sim_time: current simulation time in seconds
        :type sim_time: int
        :param force_assign: this parameter can be used to enforce the assignment, when a plan is (partially) locked
        :type force_assign: bool
        :param add_arg: possible additional argument if needed
        :type add_arg: not defined here
        """
        super().assign_vehicle_plan(veh_obj, vehicle_plan, sim_time, force_assign=force_assign, assigned_charging_task=assigned_charging_task, add_arg=add_arg)
        # new_vrl = vehicle_plan.build_VRL(veh_obj, self.rq_dict, charging_management=self.charging_management)
        # LOG.debug("init plan")
        # for ps in vehicle_plan.list_plan_stops:
        #     LOG.info(str(ps))
        # LOG.debug("init vrl")
        # for x in new_vrl:
        #     LOG.info(str(x))
        # veh_obj.assign_vehicle_plan(new_vrl, sim_time, force_ignore_lock=force_assign)
        if self.PT_lines.get(self.pt_vehicle_to_line[veh_obj.vid]) is not None:
            self.PT_lines[self.pt_vehicle_to_line[veh_obj.vid]].veh_plans[veh_obj.vid] = vehicle_plan
        else:
            LOG.warning("couldnt find {} or {} | only feasible in init".format(veh_obj.vid, self.pt_vehicle_to_line.get(veh_obj.vid)))
        #self.veh_plans[veh_obj.vid] = vehicle_plan
        for rid in get_assigned_rids_from_vehplan(vehicle_plan):
            pax_info = vehicle_plan.get_pax_info(rid)
            self.rq_dict[rid].set_assigned(pax_info[0], pax_info[1])
            self.rid_to_assigned_vid[rid] = veh_obj.vid

    def receive_status_update(self, vid, simulation_time, list_finished_VRL, force_update=True):
        """This method can be used to update plans and trigger processes whenever a simulation vehicle finished some
         VehicleRouteLegs.

        :param vid: vehicle id
        :type vid: int
        :param simulation_time: current simulation time
        :type simulation_time: float
        :param list_finished_VRL: list of VehicleRouteLeg objects
        :type list_finished_VRL: list
        :param force_update: indicates if also current vehicle plan feasibilities have to be checked
        :type force_update: bool
        """
        self.sim_time = simulation_time
        self.PT_lines[self.pt_vehicle_to_line[vid]].receive_status_update(vid, simulation_time, list_finished_VRL, force_update=force_update)

    def user_request(self, rq, simulation_time):
        """This method is triggered for a new incoming request. It generally generates a PlanRequest from the rq and
        adds it to the database. UserOffers can be created here but are not returned.
        These offers will be accessed by the simulation environment via the method self.get_current_offer(rid)
        Use the method "_create_user_offer" to create this dictionary!

        :param rq: request object containing all request information
        :type rq: RequestDesign
        :param simulation_time: current simulation time
        :type simulation_time: int
        """
        LOG.debug(f"Incoming request {rq.__dict__} at time {simulation_time}")
        # class PlanRequest:
        #     def __init__(self, rq, routing_engine, min_wait_time=0, max_wait_time=LARGE_INT, max_detour_time_factor=None,
        #                  max_constant_detour_time=None, add_constant_detour_time=None, min_detour_time_window=None,
        #                  boarding_time=0):
        prq = PlanRequest(rq, self.routing_engine)

        rid_struct = rq.get_rid_struct()
        self.rq_dict[rid_struct] = prq

        o_pos, epa, _ = prq.get_o_stop_info()
        d_pos, _, _ = prq.get_d_stop_info()

        best_line = None
        best_waiting_time = None
        best_travel_time = None
        earliest_arrival_time = float("inf")
        for line_id, line in self.PT_lines.items():
            stop_infos = line.query_travel_time_infos(o_pos, d_pos, epa, rq.nr_pax)
            if stop_infos is not None:
                waiting_time, travel_time, arrival_time = stop_infos
                if arrival_time < earliest_arrival_time:
                    best_line = line_id
                    earliest_arrival_time = arrival_time
                    best_waiting_time = waiting_time
                    best_travel_time = travel_time

        self._create_user_offer(prq, simulation_time, offer_dict_without_plan={G_OFFER_DRIVE : best_travel_time, G_OFFER_WAIT : best_waiting_time, G_OFFER_FARE : self.base_fare, "line" : best_line})   # TODO # FARE!

    def user_confirms_booking(self, rid, simulation_time):
        """This method is used to confirm a customer booking. This can trigger some database processes.

        WHEN OVERWRITING THIS METHOD MAKE SURE TO CALL AT LEAST THE FOLLOWING LINE!

        :param rid: request id
        :type rid: int
        :param simulation_time: current simulation time
        :type simulation_time: float
        """
        self.rq_dict[rid].set_service_accepted()
        current_offer = self.get_current_offer(rid)
        offered_line = current_offer["line"]
        self.PT_lines[offered_line].assign_user(rid, simulation_time)

    def user_cancels_request(self, rid, simulation_time):
        """This method is used to confirm a customer cancellation. This can trigger some database processes.

        :param rid: request id
        :type rid: int
        :param simulation_time: current simulation time
        :type simulation_time: float
        """
        if self.rid_to_assigned_vid.get(rid) is not None:
            assigned_vid = self.rid_to_assigned_vid[rid]
            line = self.pt_vehicle_to_line[rid]
            self.PT_lines[line].remove_rid_from_line(rid, assigned_vid, simulation_time)
            del self.rid_to_assigned_vid[rid]
        self.rq_dict[rid]

    def _create_user_offer(self, prq, simulation_time, assigned_vehicle_plan=None, offer_dict_without_plan={}):
        """ this method should be overwritten to create an offer for the request rid
        when overwriting this function, make sure to call the lines below depending on the offer returned

        THIS FUNCTION HAS TO BE OVERWRITTEN!

        :param prq: plan request
        :type prq: PlanRequest obj
        :param simulation_time: current simulation time
        :type simulation_time: int
        :param assigned_vehicle_plan: vehicle plan of initial solution to serve this request
        :type assigned_vehicle_plan: VehiclePlan or None
        :param offer_dict_without_plan: can be used to create an offer that is not derived from a vehicle plan
                    entries will be used to create/extend offer
        :type offer_dict_without_plan: dict
        :return: operator offer for the user
        :rtype: TravellerOffer
        """
        if offer_dict_without_plan["line"] is not None:
            offer = TravellerOffer(prq.get_rid_struct(), self.op_id, offer_dict_without_plan[G_OFFER_WAIT], offer_dict_without_plan[G_OFFER_DRIVE], offer_dict_without_plan[G_OFFER_FARE], additional_parameters={"line" : offer_dict_without_plan["line"]})
        else:
            offer = Rejection(prq.get_rid_struct(), self.op_id)
        prq.set_service_offered(offer)  # has to be called
        return offer

    def time_trigger(self, simulation_time):
        pass

    def _call_time_trigger_request_batch(self, simulation_time):
        pass

    def lock_current_vehicle_plan(self, vid):
        LOG.warning("lock vehicle plan for PT not feasible -> ignored!")
        pass

    def _lock_vid_rid_pickup(self, sim_time, vid, rid):
        LOG.warning("_lock_vid_rid_pickup for PT not feasible -> ignored!")
        pass

    def _prq_from_reservation_to_immediate(self, rid, sim_time):
        LOG.warning("_prq_from_reservation_to_immediate for PT not feasible -> ignored!")
        pass

    def change_prq_time_constraints(self, sim_time, rid, new_lpt, new_ept=None):
        LOG.warning("change_prq_time_constraints for PT not feasible -> ignored!")
        pass

    def acknowledge_boarding(self, rid, vid, simulation_time):
        """This method can trigger some database processes whenever a passenger is starting to board a vehicle.

        MAKE SURE TO CALL AT LEAST super().acknowledge_boarding(rid, vid, simulation_time) or add the following line to the
        overwritten method

        :param rid: request id
        :type rid: int
        :param vid: vehicle id
        :type vid: int
        :param simulation_time: current simulation time
        :type simulation_time: float
        """
        super().acknowledge_boarding(rid, vid, simulation_time)

    def acknowledge_alighting(self, rid, vid, simulation_time):
        """This method can trigger some database processes whenever a passenger is finishing to alight a vehicle.

        :param rid: request id
        :type rid: int
        :param vid: vehicle id
        :type vid: int
        :param simulation_time: current simulation time
        :type simulation_time: float
        """
        super().acknowledge_alighting(rid, vid, simulation_time)
