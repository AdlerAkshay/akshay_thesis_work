from __future__ import annotations
from abc import abstractmethod
from typing import Dict, List, TYPE_CHECKING, Tuple, Any
import numpy as np

from src.fleetctrl.planning.VehiclePlan import VehiclePlan, RoutingTargetPlanStop

from src.misc.globals import *
from src.fleetctrl.reservation.ReservationBase import ReservationBase
from src.fleetctrl.pooling.immediate.insertion import reservation_insertion_with_heuristics, simple_remove
from dev.fleetctrl.reservation.misc.RequestGroup import QuasiVehicle, QuasiVehiclePlan
from dev.fleetctrl.reservation.misc.objective import return_reservation_driving_leg_objective_function

import logging

from src.simulation.Offers import TravellerOffer
LOG = logging.getLogger(__name__)

if TYPE_CHECKING:
    from src.fleetctrl.planning.PlanRequest import PlanRequest
    from src.fleetctrl.planning.VehiclePlan import PlanStopBase
    
LARGE_INT = 1000000   # penalty for introducing a new vehicle in case a match between batches is not possible
RES_SOL_CFV_DYN_KEY = "reservation_sol_cfv"

class RevelationHorizonBase(ReservationBase):
    """ this reservation class can be used for reservation strategies with two rolling horizons:
        - the standard rolling horizon from which requests are revealed to the online global optimization
        - a revelation horizon that reveals parts of the offline vehicle plans to the online global optimization
    reservations are computed offline and not directly assigned to vehicles; only the first current plan stop is assigned with a time
        constraint specifying spatio temporal constraints for vehicles.
    these plan stops are referred to as "supporting points" which are stops in a route where the vehicle occupancy is zero. therefore vehicles
        can be reassigned to complete the rest of the route at these supporting points.
    therefore plans are not assigned to vehicles directly but are created independently of them and assigned to them later on
    this assignment is kept until the simulation time approches the earliest pickup time within the rolling horizon;
    then requests are revealed to the global optimisation and removed from the reservation class
    """
    def __init__(self, fleetctrl, operator_attributes, dir_names, solver="Gurobi"):
        super().__init__(fleetctrl, operator_attributes, dir_names, solver=solver)
        self.rolling_horizon = operator_attributes[G_RA_OPT_HOR]
        self.assignment_horizon = operator_attributes[G_RA_ASS_HOR]
        
        self.driving_leg_obj = return_reservation_driving_leg_objective_function(operator_attributes[G_OP_VR_CTRL_F])   # fast function to compute objective beween driving legs without ob-customers
        
        self._plan_id_to_off_plan: Dict[int, QuasiVehiclePlan] = {} # offline plan id -> offline plan (not vehicle specific)
        self._vid_to_plan_id: Dict[int, int] = {}   # current assignment of vehicle id to offline plan id
        self._plan_id_to_vid: Dict[int, int] = {}
        self._sorted_rids_with_epa = []  # list of (rid, earliest pick up time rid) sorted by epa
        self._reavealed_rids = {}
        self._rid_to_assigned_plan = {} # rid -> plan_id
        self._unprocessed_rids = {} # rid -> 1 for requests that havent been treated in the global optimisation yet
        
        self.vehicle_capacity = self.fleetctrl.sim_vehicles[0].max_pax  # TODO not possible for mixed fleets!
        self.fleet_size = len(self.fleetctrl.sim_vehicles)
        
        self._last_request_time = -1
        self._current_global_veh_plans : Dict[int, VehiclePlan] = {}
        self._current_vids_with_supporting_points = {}  # vid -> 1 if a supporting point is currently assigned to this vid
        
        self.re_assign_sps = operator_attributes.get(G_RA_RES_REASSIGN_SP, True)
        self.weak_reopt = operator_attributes[G_RA_REOPT_TS]    # after new assignment optimisation supporting points are also reassigned
        self.reopt_int = operator_attributes[G_RA_RES_BOPT_TS]   # without immediate offer queries, offers and new solutions are only created within this frequency
        self.fleetctrl._init_dynamic_fleetcontrol_output_key(RES_SOL_CFV_DYN_KEY)
        
    def add_reservation_request(self, plan_request: PlanRequest, sim_time: int):
        """ this function adds a new request which is treated as reservation 
        :param plan_request: PlanRequest obj
        :param sim_time: current simulation time"""
        super().add_reservation_request(plan_request, sim_time)
        if plan_request.get_o_stop_info()[1] <= sim_time + self.assignment_horizon + (-sim_time%self.reopt_int):
            LOG.debug(f" -> return immediate offer for rid {plan_request.get_rid_struct()}!")
            self.return_immediate_reservation_offer(plan_request.get_rid_struct(), sim_time)
            self._reavealed_rids[plan_request.get_rid_struct()] = 1
        else:
            self._unprocessed_rids[plan_request.get_rid_struct()] = 1
        
    def reveal_requests_for_online_optimization(self, sim_time):
        """ this function is triggered during the simulation and returns a list of request ids that should be treated as online requests in the global optimisation and
        the assignment process
        :param sim_time: current simulation time
        :return: list of request ids"""
        LOG.debug("sorted rids with epa {}".format(self._sorted_rids_with_epa))
        # check for rids which schedules are allready assigned to vehicles
        reveal_index = 0
        while reveal_index < len(self._sorted_rids_with_epa) and self._sorted_rids_with_epa[reveal_index][1] <= sim_time + self.rolling_horizon:
            reveal_index += 1
        to_return = [self._sorted_rids_with_epa[x][0] for x in range(reveal_index)]
        self._sorted_rids_with_epa = self._sorted_rids_with_epa[reveal_index:]
        for rid in to_return:
            try:
                del self.active_reservation_requests[rid]
            except KeyError:
                LOG.debug("rid {rid} allready revealed for global opt?")
                continue
            del self._reavealed_rids[rid]
            try:
                del self._rid_to_assigned_plan[rid]
            except KeyError:
                pass
            
        # check for currently assigned supporting points
        LOG.debug(f"reveal following reservation requests at time {sim_time} : {to_return}")
        for plan_id, res_veh_plan in list(self._plan_id_to_off_plan.items()):
            if res_veh_plan is None:
                continue
            vid = self._plan_id_to_vid[plan_id]
            veh_plan = self.fleetctrl.veh_plans[vid]
            LOG.debug(f"add to vid {vid} -> {plan_id}")
            LOG.debug(f"online plan: {veh_plan}")
            LOG.debug(f"reservation plan: {res_veh_plan}")
            last_ps = veh_plan.list_plan_stops[-1]
            arr, dep = last_ps.get_planned_arrival_and_departure_time()
            if arr is None:
                veh_plan.update_tt_and_check_plan(self.fleetctrl.sim_vehicles[vid], sim_time, self.routing_engine, keep_feasible=True)
                last_ps = veh_plan.list_plan_stops[-1]
                arr, dep = last_ps.get_planned_arrival_and_departure_time()
            est = last_ps.get_earliest_start_time()
            LOG.debug(f" -> est {est} arr {arr} thresh {sim_time + self.assignment_horizon}")
            if est > arr:
                arr = est
            # test for new schedules to reveal
            if arr < sim_time + self.assignment_horizon:
                veh_plan.list_plan_stops.pop()
                first_list_ps = self._split_plan_until_revelation(vid, sim_time)
                #append at current schedule
                veh_plan.list_plan_stops += first_list_ps
                veh_plan.update_tt_and_check_plan(self.fleetctrl.sim_vehicles[vid], sim_time, self.routing_engine, keep_feasible=True)
                self.fleetctrl.assign_vehicle_plan(self.fleetctrl.sim_vehicles[vid], veh_plan, sim_time)     
        return to_return
    
    def return_availability_constraints(self, sim_time):
        """ this function returns a list of network positions with times where a vehicle has to be available to fullfill future reservations
        this information can be included in the assignment process 
        (can be used as constraints for global optimisation)
        :param sim_time: current simulation time
        :return: list of (position, latest arrival time)"""
        raise NotImplementedError
        return []
    
    def user_confirms_booking(self, rid, sim_time):
        """ in this implementation nothing has to be done since the assignment is made in the "return_reservation_offer" methode
        :param rid: request id
        :param sim_time: current simulation time        
        """
        pass

    def user_cancels_request(self, rid, simulation_time):
        """ in case a reservation request which could be assigned earlier cancels the request
        this function removes the request from the assigned vehicle plan and deletes all entries in the database
        :param rid: request id
        :param simulation_time: current simulation time
        """
        LOG.debug("reservation request cancels request: {} {}".format(rid, simulation_time))
        if self._rid_to_assigned_plan.get(rid) is not None:
            self._last_request_time = -1    # to rebuild full vehicleplans in case a new request is coming in the same timestep
            LOG.debug(f" -> remove from offline plan {self._rid_to_assigned_plan[rid]}")
            plan_id = self._rid_to_assigned_plan[rid]
            plan = self._plan_id_to_off_plan[plan_id]
            new_plan = simple_remove(QuasiVehicle(None, self.vehicle_capacity), plan, rid, simulation_time, self.routing_engine, self.fleetctrl.vr_ctrl_f, self.active_reservation_requests, self.fleetctrl.const_bt, self.fleetctrl.add_bt)
            if len(new_plan.list_plan_stops) > 0:   # still reservation trip assigned, but supporting point might need to be updated
                self._plan_id_to_off_plan[plan_id] = new_plan
                sup_point_pos = new_plan.list_plan_stops[0].get_pos()
                sup_earliest_start_time = new_plan.list_plan_stops[0].get_earliest_start_time()
                assigned_vid = self._plan_id_to_vid[plan_id]
                current_veh_plan = self.fleetctrl.veh_plans[assigned_vid]
                new_ps = RoutingTargetPlanStop(sup_point_pos, earliest_start_time=sup_earliest_start_time, latest_start_time=sup_earliest_start_time, planstop_state=G_PLANSTOP_STATES.RESERVATION, duration=LARGE_INT, locked_end=True)
                current_veh_plan.add_plan_stop(new_ps, self.fleetctrl.sim_vehicles[assigned_vid], simulation_time, self.routing_engine)
                self.fleetctrl.assign_vehicle_plan(self.fleetctrl.sim_vehicles[assigned_vid], current_veh_plan, simulation_time)
            else:   # supporting point can be removed
                LOG.debug(f" -> no more reservation plan left")
                assigned_vid = self._plan_id_to_vid[plan_id]
                current_veh_plan = self.fleetctrl.veh_plans[assigned_vid]
                current_veh_plan.list_plan_stops.pop()  # remove supporting point
                self.fleetctrl.assign_vehicle_plan(self.fleetctrl.sim_vehicles[assigned_vid], current_veh_plan, simulation_time)
                del self._plan_id_to_vid[plan_id]
                del self._vid_to_plan_id[assigned_vid]
                del self._current_vids_with_supporting_points[assigned_vid]
                
        if self._reavealed_rids.get(rid) is not None:
            assigned_vid = self.fleetctrl.rid_to_assigned_vid.get(rid)
            LOG.debug(f" -> remove already revealed rid from vid {assigned_vid}")
            if assigned_vid is not None:
                current_veh_plan = self.fleetctrl.veh_plans[assigned_vid]
                new_veh_plan = simple_remove(self.fleetctrl.sim_vehicles[assigned_vid], current_veh_plan, rid, simulation_time, self.routing_engine, self.fleetctrl.vr_ctrl_f, self.fleetctrl.rq_dict, self.fleetctrl.const_bt, self.fleetctrl.add_bt)
                self.fleetctrl.assign_vehicle_plan(self.fleetctrl.sim_vehicles[assigned_vid], new_veh_plan, simulation_time)
        try:
            del self.active_reservation_requests[rid]
        except KeyError:
            LOG.debug("rid {rid} allready revealed for global opt?")
        try:
            del self._unprocessed_rids[rid]
        except KeyError:
            pass
        try:
            del self._reavealed_rids[rid]
        except KeyError:
            pass
        try:
            del self._rid_to_assigned_plan[rid]
        except KeyError:
            pass
            
    def time_trigger(self, sim_time):
        """ this function is triggered during the simulation time and might trigger reoptimization processes for example 
        :param sim_time: simulation time """
        if sim_time % self.reopt_int == 0:
            self._batch_optimisation(sim_time)
            self._supporting_point_reassignment(sim_time, preceeding_reopt=True)
            self._create_batch_offers(sim_time)
        elif self.re_assign_sps and sim_time % self.weak_reopt == 0:
            self._supporting_point_reassignment(sim_time, preceeding_reopt=False)
        current_off_sol_cfv = 0
        for off_plan in self._plan_id_to_off_plan.values():
            current_off_sol_cfv += off_plan.compute_obj_function(self.fleetctrl.vr_ctrl_f, self.routing_engine, self.active_reservation_requests)
        self.fleetctrl._add_to_dynamic_fleetcontrol_output(sim_time, {RES_SOL_CFV_DYN_KEY: current_off_sol_cfv})
    
    def return_immediate_reservation_offer(self, rid, sim_time):
        """ this function returns an offer if possible for an reservation request in case an immediate offer is needed for a reservation request
        the has been added to the reservation module before 
        in this implementation, an offer is created based on the implementation of an insertion heuristic with reservation heuristics
        :param rid: request id
        :param sim_time: current simulation time
        :return: offer for request """
        prq = self.active_reservation_requests[rid]
        # update the current complete vehicle plans (merge online and offline part of the plans!)
        if sim_time != self._last_request_time:
            self._current_global_veh_plans = {}
            for vid, veh_plan in self.fleetctrl.veh_plans.items():
                if self._current_vids_with_supporting_points.get(vid):
                    plan_id = self._vid_to_plan_id[vid]
                    res_plan = self._plan_id_to_off_plan[plan_id]
                    fullplan = veh_plan.copy()
                    fullplan.list_plan_stops.pop()
                    fullplan.list_plan_stops += res_plan.list_plan_stops
                    self.fleetctrl.compute_VehiclePlan_utility(sim_time, self.fleetctrl.sim_vehicles[vid], fullplan)
                    self._current_global_veh_plans[vid] = fullplan
                else:
                    if veh_plan.get_utility() is None:
                        self.fleetctrl.compute_VehiclePlan_utility(sim_time, self.fleetctrl.sim_vehicles[vid], veh_plan)
                    self._current_global_veh_plans[vid] = veh_plan.copy()
            self._last_request_time = sim_time
        # test for insertion
        tuple_list = reservation_insertion_with_heuristics(sim_time, prq, self.fleetctrl, force_feasible_assignment=True, veh_plans=self._current_global_veh_plans)
        if len(tuple_list) > 0:
            best_tuple = min(tuple_list, key=lambda x:x[2])
            best_vid, best_plan, _ = best_tuple
            offer = self.fleetctrl._create_user_offer(prq, sim_time, assigned_vehicle_plan=best_plan)
            self._assign_full_vehicleplan_after_insertion(best_vid, best_plan, sim_time) # plans will be split in online and offline part here again
            self._current_global_veh_plans[best_vid] = best_plan
            LOG.debug(f"offer for reservation request {rid}: {offer}")
        else:
            offer = self.fleetctrl._create_rejection(prq, sim_time)
            LOG.debug(f"no offer for reservation request {rid}")
        return offer
    
    def _assign_full_vehicleplan_after_insertion(self, vid : int, fullvehplan : VehiclePlan, sim_time : int):
        """ this methods handles the correct assignment of a full vehicle plan which includes the online and offline part of the vehicle plan
        only the online part will be assigned to the vehicle (defined by assignment horizen)
        the offline part is stored again in this module and will be revealed later
        :param vid: vehicle id
        :param fullvehplan: full vehicle plan (online and offline part)
        :param sim_time: current simulation time
        """
        # split full plan in online and offline part
        list_online_plan_stops = self._split_plan_until_revelation(vid, sim_time, fullvehicleplan=fullvehplan)
        veh_plan = VehiclePlan(self.fleetctrl.sim_vehicles[vid], sim_time, self.routing_engine, list_online_plan_stops)
        veh_plan.update_tt_and_check_plan(self.fleetctrl.sim_vehicles[vid], sim_time, self.routing_engine, keep_feasible=True)
        # assign to vehicle
        self.fleetctrl.assign_vehicle_plan(self.fleetctrl.sim_vehicles[vid], veh_plan, sim_time)

    def _split_plan_until_revelation(self, vid : int, sim_time, fullvehicleplan : VehiclePlan = None) -> List[PlanStopBase]:
        """ this method is used to split the vehicleplan until the current point of revelation and sets corresponding entries in revealed requests
        and when requests should be considered for global optimisation
        it returns two parts of the Quasivehicleplan
        the first part has to be assigned to vehicles (revealed)
        the second part is stored in the current _plan_id_to_off_plan dictionary
        :param vid: vehicle id assigned to corresponding plan in _plan_id_to_off_plan to split and store in
        :param sim_time: current simulation time
        :param fullvehicleplan: (optional) if given this plan is split (i.e. after performing a full insertion) if not the quasivehicleplan in _plan_id_to_off_plan is used
        :return: list of planstops to be assigned (and maybe added to current vehicle plan) to vehicle (offline part is stored in _plan_id_to_off_plan dictionary"""
        plan_id = self._vid_to_plan_id.get(vid)
        if fullvehicleplan is None:
            if plan_id is None:
                raise KeyError(f"no assignment of offline plans for vehicle {vid} found!")
            qvp = self._plan_id_to_off_plan[plan_id]
        else:
            qvp = fullvehicleplan
        LOG.debug(f"split for vid {vid} : {qvp}")

        current_occ = 0
        second_start_time = None
        current_rids = {}
        forced_stop = False
        index = len(qvp.list_plan_stops)
        split_index = index
        for ps in reversed(qvp.list_plan_stops):
            index -= 1
            current_occ -= ps.get_change_nr_pax()
            for rid in ps.get_list_boarding_rids():
                current_rids[rid] = 1
            if ps.is_locked() or ps.is_empty(): # rebalancing or locked parts should always be in online list
                forced_stop = True
            if current_occ == 0:
                possible_second_start_time, departure_time = ps.get_planned_arrival_and_departure_time()
                est = ps.get_earliest_start_time()
                if est > possible_second_start_time:
                    possible_second_start_time = est
                part_of_rid_revealed = False
                for rid in current_rids.keys():
                    if self._reavealed_rids.get(rid) or not self.active_reservation_requests.get(rid):
                        part_of_rid_revealed = True
                        break
                if not part_of_rid_revealed and not forced_stop and possible_second_start_time > sim_time + self.assignment_horizon:    # possible checkpoint
                    second_start_time = possible_second_start_time
                    split_index = index
                    LOG.debug(f"possible break at {second_start_time} {split_index}")
                else:    # split detected
                    break
                current_rids = {}  
        LOG.debug(f" -> split index {split_index}")
        first_plan_stops = [qvp.list_plan_stops[i].copy() for i in range(split_index)] 
        second_plan_stops =[qvp.list_plan_stops[i].copy() for i in range(split_index, len(qvp.list_plan_stops))]
        LOG.debug(f"first plan stops: {[str(x) for x in first_plan_stops]}")
        LOG.debug(f"second plan stops: {[str(x) for x in second_plan_stops]}")
        #check for new revelations
        for ps in first_plan_stops:
            for rid in ps.get_list_boarding_rids():
                if self.active_reservation_requests.get(rid) and not self._reavealed_rids.get(rid):
                    plan_request = self.active_reservation_requests[rid]
                    self._sorted_rids_with_epa.append( (plan_request.get_rid_struct(), plan_request.get_o_stop_info()[1]))
                    self._reavealed_rids[rid] = 1
                    LOG.debug(f"reveal soon: {plan_request.get_rid_struct()} at {plan_request.get_o_stop_info()[1]}")

        # no more reservation part
        if second_start_time is None:
            if plan_id is not None:
                del self._plan_id_to_off_plan[plan_id]
                del self._plan_id_to_vid[plan_id]
                del self._vid_to_plan_id[vid]
                del self._current_vids_with_supporting_points[vid]
        else:   # reservation part
            sup_point_pos = second_plan_stops[0].get_pos()
            first_plan_stops.append( RoutingTargetPlanStop(sup_point_pos, earliest_start_time=second_start_time, latest_start_time=second_start_time, planstop_state=G_PLANSTOP_STATES.RESERVATION, duration=LARGE_INT, locked_end=True) )
            if plan_id is None:
                if len(self._plan_id_to_off_plan) == 0:
                    plan_id = 0
                else:
                    plan_id = 0
                    while True:
                        if self._plan_id_to_off_plan.get(plan_id) is None:
                            break
                        plan_id += 1
            self._plan_id_to_off_plan[plan_id] = QuasiVehiclePlan(self.routing_engine, second_plan_stops, self.vehicle_capacity, second_start_time)
            self._vid_to_plan_id[vid] = plan_id
            self._plan_id_to_vid[plan_id] = vid
            self._current_vids_with_supporting_points[vid] = 1
            LOG.debug(" -> reservation part: {}".format(self._plan_id_to_off_plan[plan_id]))
        self._sorted_rids_with_epa.sort(key=lambda x:x[1])  
        LOG.debug(f"new online part: {[x for x in first_plan_stops]}")
        return first_plan_stops
            
    def _create_batch_offers(self, sim_time) -> Dict[int, TravellerOffer]:
        """ this method creates new offers for reservation requests after the batch optimization updated the solution
        :param sim_time
        :return: dict reservation rid -> TravellerOffer"""
        rid_to_offer = {}
        for plan_id, offline_plan in self._plan_id_to_off_plan.items():
            for rid in offline_plan.get_involved_request_ids():
                prq = self.active_reservation_requests[rid]
                offer = self.fleetctrl._create_user_offer(prq, sim_time, offline_plan)
                rid_to_offer[rid] = offer
        for rid,prq in self.active_reservation_requests.items():
            if prq.get_current_offer() is None:
                offer = self.fleetctrl._create_rejection(prq, sim_time)
                rid_to_offer[rid] = offer
        return rid_to_offer
            
    @abstractmethod
    def _batch_optimisation(self, sim_time):
        """ this method creates a new solution for the reservation plans by reoptimzing including new requests
        - the task of this method is to fill the dictionary self._plan_id_to_off_plan
        - the method has to guarantee that a currently feasible matching vehicle id -> plan_id exists (reachable within timeconstraints)
        :param sim_time: current simulation time"""
        pass
    
    def _supporting_point_reassignment(self, sim_time, preceeding_reopt = False):
        """ this methods reassigns offline plans to currently assigned vehicle plans
        the objective is to minimize driven distance while using as few vehicles as possible while time constraints are fulfilled
        :param sim_time: current simulation time"""
        LOG.debug("reassign supp points at {}".format(sim_time))
        LOG.debug("assignments before: {} | {}".format(self._vid_to_plan_id, self._plan_id_to_vid))
        LOG.debug("current vids with supporting points: {}".format(self._current_vids_with_supporting_points))
        for plan_id, plan in self._plan_id_to_off_plan.items():
            LOG.debug(f"{plan_id} -> {plan}")
        # vehicle constraints
        vid_end_pos_time_dict, _ = self._get_vid_batch_constraints(sim_time)
        plan_id_start_constraints = self._get_offline_plan_start_constraints(sim_time)
        matching_dict = {}
        # ensure feasibility
        if not preceeding_reopt:   # no preceeding optimisation -> previous matches have to be feasible
            for vid, plan_id in self._vid_to_plan_id.items():
                try:
                    matching_dict[plan_id][vid] = LARGE_INT * 0.1
                except KeyError:
                    matching_dict[plan_id] = {vid : LARGE_INT * 0.1}
        else:   # a new solution has been created: plan_id refers to matched vehicle
            for plan_id in self._plan_id_to_off_plan.keys():
                try:
                    matching_dict[plan_id][plan_id] = LARGE_INT * 0.1
                except KeyError:
                    matching_dict[plan_id] = {plan_id : LARGE_INT * 0.1}
        for plan_id, s_pos_s_time in plan_id_start_constraints.items():
            if matching_dict.get(plan_id) is None:
                matching_dict[plan_id] = {}
            s_pos, s_time = s_pos_s_time
            for vid, e_pos_e_time in vid_end_pos_time_dict.items():
                e_pos, e_time = e_pos_e_time
                _, tt, dis = self.routing_engine.return_travel_costs_1to1(e_pos, s_pos)
                if e_time + tt <= s_time:
                    matching_dict[plan_id][vid] = dis   # TODO dis as objective?
        # match them together
        plan_id_vid_matches = self._match_plan_id_to_vid(matching_dict)
        # assign the plans
        new_assigned_vids = {}
        self._vid_to_plan_id = {}
        self._plan_id_to_vid = {}
        # set new assignments
        for plan_id, vid in plan_id_vid_matches:
            off_plan = self._plan_id_to_off_plan[plan_id]
            start_pos, start_time, _, _ = off_plan.get_start_end_constraints()
            r_ps = RoutingTargetPlanStop(start_pos, earliest_start_time=start_time, latest_start_time=start_time, duration=LARGE_INT, locked_end=True, planstop_state=G_PLANSTOP_STATES.RESERVATION)
            veh_plan = self.fleetctrl.veh_plans[vid]
            #LOG.debug(f"current assignment of vid {vid} : {veh_plan}")
            if self._current_vids_with_supporting_points.get(vid):
                veh_plan.list_plan_stops.pop()  # remove former routing target ps
            veh_plan.add_plan_stop(r_ps, self.fleetctrl.sim_vehicles[vid], sim_time, self.routing_engine)
            self.fleetctrl.assign_vehicle_plan(self.fleetctrl.sim_vehicles[vid], veh_plan, sim_time)
            new_assigned_vids[vid] = 1
            self._vid_to_plan_id[vid] = plan_id
            self._plan_id_to_vid[plan_id] = vid
        # remove rest old assignments
        for vid in self._current_vids_with_supporting_points.keys():
            if new_assigned_vids.get(vid) is None:
                veh_plan = self.fleetctrl.veh_plans[vid]
                veh_plan.list_plan_stops.pop()  # remove former routing target ps
                self.fleetctrl.assign_vehicle_plan(self.fleetctrl.sim_vehicles[vid], veh_plan, sim_time)
        self._current_vids_with_supporting_points = new_assigned_vids
        LOG.debug("assignments after: {} | {} | {}".format(self._vid_to_plan_id, self._plan_id_to_vid, self._current_vids_with_supporting_points))

    
    def _match_plan_id_to_vid(self, plan_id_to_vid_matches):
        """ this method solves the maximum matching problem of matching vehicles to reservation schedules
        # TODO # is another approach with not using as few vehicles as possible useful?
        :param plan_id_to_vid_matches: dict plan_vid -> vid -> objective value to minimize for feasible matches
        :return: list of tuples (plan_vid, vid) for assignment"""
        if self.solver == "Gurobi":
            return self._match_plan_id_to_vid_gurobi(plan_id_to_vid_matches)
        else:
            raise EnvironmentError(f"Solver {self.solver} not implemented for this class!")

    def _match_plan_id_to_vid_gurobi(self, plan_id_to_vid_matches):
        model_name = f"RevelationHorizonBase: _match_plan_id_to_vid_gurobi_{self.fleetctrl.sim_time}"

        import gurobipy as gurobi

        new_assignments = []    # (im_vid, vid)
        with gurobi.Env(empty=True) as env:
            if self.fleetctrl.log_gurobi:
                with open(os.path.join(self.fleetctrl.dir_names[G_DIR_OUTPUT], "gurobi_log.log"), "a") as f:
                    f.write(f"\n\n{model_name}\n\n")
                env.setParam('OutputFlag', 1)
                env.setParam('LogToConsole', 0)
                env.setParam('LogFile', os.path.join(self.fleetctrl.dir_names[G_DIR_OUTPUT], "gurobi_log.log") )
                env.start()
            else:
                env.setParam('OutputFlag', 0)
                env.setParam('LogToConsole', 0)
                env.start()

            m = gurobi.Model(model_name, env = env)

            m.setParam(gurobi.GRB.param.Threads, self.fleetctrl.n_cpu)

            variables = {}  # rtv_key -> gurobi variable
            
            expr = gurobi.LinExpr()   # building optimization objective
            key_to_varnames = {}
            varnames_to_key = {}
            im_vids_const = {}
            vids_const = {}
            c = 0
            for plan_id, vid_dict in plan_id_to_vid_matches.items():
                for vid, obj in vid_dict.items():
                    key_to_varnames[(plan_id, vid)] = str(c)
                    varnames_to_key[str(c)] = (plan_id, vid)
                    try:
                        vids_const[vid].append((plan_id, vid))
                    except:
                        vids_const[vid] = [(plan_id, vid)]
                    try:
                        im_vids_const[plan_id].append((plan_id, vid))
                    except:
                        im_vids_const[plan_id] = [(plan_id, vid)]
                    cost = -LARGE_INT + obj
                    #LOG.debug(f"imvid vid cost: {plan_id} {vid} {cost} | {obj}")
                    var = m.addVar(name = str(c), obj = cost, vtype = gurobi.GRB.BINARY)
                    variables[(plan_id, vid)] = var
                    #print("var {} -> {}".format((vid, rg), cost))
                    expr.add(var, cost)
                    c += 1
                
            m.setObjective(expr, gurobi.GRB.MINIMIZE)

            #vehicle constraint -> maximally one assignment (no constraint of a vehicle not yet available is chosen)
            for vid in vids_const.keys():
                expr = gurobi.LinExpr()
                for rtv in vids_const[vid]:
                    expr.add(variables[rtv], 1)
                m.addConstr(expr, gurobi.GRB.LESS_EQUAL, 1, "c_v_{}".format(vid))
                
            #requests constraint -> assign all
            for im_vid in im_vids_const.keys():
                expr = gurobi.LinExpr()
                for rtv in im_vids_const[im_vid]:
                    expr.add(variables[rtv], 1)
                m.addConstr(expr, gurobi.GRB.LESS_EQUAL, 1, "c_iv_{}".format(im_vid))
                

            m.optimize() #optimization

            #get solution
            varnames = m.getAttr("VarName", m.getVars())
            solution = m.getAttr("X",m.getVars())
            
            for x in range(len(solution)):
                if round(solution[x]) == 1:
                    key = varnames_to_key[varnames[x]]
                    plan_id = key[0]
                    vid = key[1]
                    new_assignments.append( (plan_id, vid) )
                    LOG.debug(f"assigning off plan {plan_id} -> vid {vid}")
        return new_assignments
    
    def _get_vid_batch_constraints(self, sim_time : int) -> Tuple[Dict[Any, Tuple[tuple, int]], int]:
        """ returns the constraints of the currently assigned plans for the vehicles that make it possible for reservation tasks
        thereby empty plan stops at the end of the plan are ignored (i.e. reservation and repositioning stops)
        :return: dict vid -> (pos, end_time) of last plan stop, end time of longest vehicle plan"""
        current_assignment_horizon = sim_time
        vid_batch_constraints = {}
            
        for vid, veh_plan in self.fleetctrl.veh_plans.items():
            LOG.debug(f"current assignment of vid {vid} : {veh_plan}")
            if len(veh_plan.list_plan_stops) > 0:
                found = False
                for i in range(len(veh_plan.list_plan_stops)-1, -1, -1):
                    ps = veh_plan.list_plan_stops[i]
                    LOG.debug(f"ps {ps}")
                    if not found and (not ps.is_empty() or ps.is_locked()):
                        p = ps.get_pos()
                        _, end_time = ps.get_planned_arrival_and_departure_time()
                        LOG.debug(" -> not empty")
                        vid_batch_constraints[vid] = (p, end_time)
                        found = True
                    for rid in ps.get_list_alighting_rids():
                        if self.active_reservation_requests.get(rid) is None:
                            _, online_end_time = ps.get_planned_arrival_and_departure_time()
                            if online_end_time > current_assignment_horizon:
                                current_assignment_horizon = online_end_time
                            break
                if not found:
                    vid_batch_constraints[vid] = (self.fleetctrl.sim_vehicles[vid].pos, sim_time)
            else:
                vid_batch_constraints[vid] = (self.fleetctrl.sim_vehicles[vid].pos, sim_time)
        LOG.debug("vid batch constraints: {}".format(vid_batch_constraints))
        return vid_batch_constraints, current_assignment_horizon
    
    def _get_offline_plan_start_constraints(self, sim_time : int) -> Dict[int, Tuple[tuple, int]]:
        """ returns the start constraints of the offline plans
        :return: dict plan_id -> (pos, latest_start_time) of first plan stop"""
        plan_id_start_constraints = {}
        for plan_id, off_plan in self._plan_id_to_off_plan.items():
            if off_plan is not None and len(off_plan.list_plan_stops) > 0:
                f_ps = off_plan.list_plan_stops[0]
                plan_id_start_constraints[plan_id] = (f_ps.get_pos(), f_ps.get_earliest_start_time())  # TODO other formulation??
        LOG.debug("plan_id start constraints: {}".format(plan_id_start_constraints))
        return plan_id_start_constraints
    
    def _get_assigned_subplans_per_request_from_off_plans(self) -> Dict[int, Tuple[QuasiVehiclePlan, QuasiVehiclePlan]]:
        """ this method iterates through all assigned offline plans and creates sub-schedules for each request (i.e. the parts where the request is scheduled and 
        the vehicle doesnt not get empty).
        this method can be used to infuse the last assigned solution into the matching problem to enforce feasiblity in case of a re-optimisation
        :return: dict rid -> (quasi-vehicle plan of rids, preceeding quasi-vehicle plan (or None) )"""
        rid_to_assigned_subplan = {}
        for plan_id, off_plan in self._plan_id_to_off_plan.items():
            current_occ = 0
            last_start_time = -1
            current_involved_rids = []
            current_ordered_planstops = []
            preceeding_qvp = None
            for i, ps in enumerate(off_plan.list_plan_stops):
                if current_occ == 0 and len(current_ordered_planstops) > 0:
                    sub_plan = QuasiVehiclePlan(self.routing_engine, current_ordered_planstops, self.vehicle_capacity, earliest_start_time=last_start_time)                    
                    for rid in current_involved_rids:
                        rid_to_assigned_subplan[rid] = (sub_plan, preceeding_qvp)
                    last_start_time = ps.get_planned_arrival_and_departure_time()[0]
                    current_ordered_planstops = []
                    current_involved_rids = []
                    preceeding_qvp = sub_plan
                    
                current_occ += ps.get_change_nr_pax()
                for rid in ps.get_list_boarding_rids():
                    current_involved_rids.append(rid)
                current_ordered_planstops.append(ps)    
                    
            if current_occ != 0:
                raise EnvironmentError("Occupancy is not zero hat end of plan??? {}".format(self))
            sub_plan = QuasiVehiclePlan(self.routing_engine, current_ordered_planstops, self.vehicle_capacity, earliest_start_time=last_start_time)                    
            for rid in current_involved_rids:
                rid_to_assigned_subplan[rid] = (sub_plan, preceeding_qvp)
            last_start_time = ps.get_planned_arrival_and_departure_time()[0]
            current_ordered_planstops = []
            current_involved_rids = []
            
        return rid_to_assigned_subplan