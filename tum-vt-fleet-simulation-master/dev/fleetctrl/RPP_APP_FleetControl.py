# -------------------------------------------------------------------------------------------------------------------- #
# standard distribution imports
# -----------------------------
from __future__ import annotations
from typing import Dict, List, Tuple, Any, TYPE_CHECKING
import logging
from abc import ABC, abstractmethod
import time

# additional module imports (> requirements)
# ------------------------------------------

# src imports
# -----------
from src.fleetctrl.FleetControlBase import LARGE_INT, FleetControlBase
from src.fleetctrl.planning.VehiclePlan import VehiclePlan, RoutingTargetPlanStop
from src.fleetctrl.planning.PlanRequest import PlanRequest
from src.simulation.Offers import TravellerOffer
from src.simulation.Legs import VehicleRouteLeg
from src.fleetctrl.RPPFleetControl import ParcelPlanRequest, RPPFleetControlFullInsertion
from src.fleetctrl.rideparcelpooling.objectives import return_parcel_pooling_objective_function
from src.fleetctrl.rideparcelpooling.immediate.insertion import insert_parcel_prq_in_selected_veh_list, insert_prq_in_selected_veh_list_route_with_parcels, insert_parcel_o_in_selected_veh_list_route_with_parcels, insert_parcel_d_in_selected_veh_list_route_with_parcels
from src.fleetctrl.pooling.immediate.insertion import simple_remove
from src.fleetctrl.planning.VehiclePlan import BoardingPlanStop
if TYPE_CHECKING:
    from src.demand.TravelerModels import RequestBase, ParcelRequestBase
    from src.routing.NetworkBase import NetworkBase
    from src.simulation.Vehicles import SimulationVehicle

# -------------------------------------------------------------------------------------------------------------------- #
# global variables
# ----------------
from src.misc.globals import *
LOG = logging.getLogger(__name__)

class RPP_APP_FleetControl(FleetControlBase):
    """ this class is used for the TEMPUS RPP App 
    it implements additional control structures i.e. manually activating and deactivating specific vehicles
    TODO specify correct mother class
    """
    def __init__(self, op_id: int, operator_attributes: Dict, list_vehicles: List[SimulationVehicle], routing_engine: NetworkBase, zone_system, scenario_parameters: Dict, dir_names: Dict, op_charge_depot_infra = None, list_pub_charging_infra = ...):
        super().__init__(op_id, operator_attributes, list_vehicles, routing_engine, zone_system, scenario_parameters, dir_names, op_charge_depot_infra, list_pub_charging_infra)
        self.rid_to_assigned_vid = {} # rid -> vid
        self.pos_veh_dict = {}  # pos -> list_veh
        self.vr_ctrl_f = return_parcel_pooling_objective_function(operator_attributes[G_OP_VR_CTRL_F])
        
        self.parcel_earliest_pickup_time = operator_attributes.get(G_OP_PA_EPT, 0)
        self.parcel_latest_pickup_time = operator_attributes.get(G_OP_PA_LPT, LARGE_INT)
        self.parcel_earliest_dropoff_time = operator_attributes.get(G_OP_PA_EDT, 0)
        self.parcel_latest_dropoff_time = operator_attributes.get(G_OP_PA_LDT, LARGE_INT)
        self.parcel_const_bt = operator_attributes[G_OP_PA_CONST_BT]
        self.parcel_add_bt = operator_attributes.get(G_OP_PA_ADD_BT, 0)
        
        self.allow_parcel_pu_with_ob_cust = operator_attributes[G_OP_PA_OBASS]
        
        self.tmp_assignment = {}  # rid -> VehiclePlan
        self._init_dynamic_fleetcontrol_output_key(G_FCTRL_CT_RQU)
        # parcels
        self.parcel_dict : Dict[Any, ParcelPlanRequest] = {} 
        
        self._reject_later = {} # vid -> rid -> bool reject rid from vid later (after boarding completed); bool : bring to depot or not
    
    def _call_time_trigger_request_batch(self, simulation_time):
        """ not time triggered optimisation in this fleet control"""
        return
    
    def _person_request(self, person_request: RequestBase, sim_time: int):
        """This method is triggered for a new incoming request. It generally adds the rq to the database. It has to
        return an offer to the user. This operator class only works with immediate responses and therefore either
        sends an offer or a rejection. this function uses a simple insertion heuristic to assign customers and create offers

        :param person_request: request object containing all request information
        :type person_request: RequestBase
        :param sim_time: current simulation time
        :type sim_time: int
        """
        t0 = time.perf_counter()
        LOG.debug(f"Incoming request {person_request.__dict__} at time {sim_time}")
        self.sim_time = sim_time
        prq = PlanRequest(person_request, self.routing_engine, min_wait_time=self.min_wait_time,
                          max_wait_time=self.max_wait_time,
                          max_detour_time_factor=self.max_dtf, max_constant_detour_time=self.max_cdt,
                          add_constant_detour_time=self.add_cdt, min_detour_time_window=self.min_dtw,
                          boarding_time=self.const_bt)

        rid_struct = person_request.get_rid_struct()
        self.rq_dict[rid_struct] = prq

        if prq.o_pos == prq.d_pos:
            LOG.debug(f"automatic decline for rid {rid_struct}!")
            self._create_rejection(prq, sim_time)
            return
        LOG.debug(f"current vehicle states:")
        for x in self.sim_vehicles:
            LOG.debug(f"{x}")
        for veh in self.sim_vehicles:
            self.veh_plans[veh.vid].update_tt_and_check_plan(veh, sim_time, self.routing_engine, keep_feasible=True)
            utility = self.vr_ctrl_f(sim_time, veh, self.veh_plans[veh.vid], self.rq_dict, self.routing_engine)
            self.veh_plans[veh.vid].set_utility(utility)
        list_tuples = insert_prq_in_selected_veh_list_route_with_parcels(self.sim_vehicles, self.veh_plans, prq, self.vr_ctrl_f,
                                                            self.routing_engine, self.rq_dict, sim_time, self.const_bt, self.add_bt,
                                                            allow_parcel_pu_with_ob_cust=self.allow_parcel_pu_with_ob_cust)
        #list_tuples = insertion_with_heuristics(sim_time, prq, self, force_feasible_assignment=True)
        LOG.debug("list insertion solutions:")
        for x in list_tuples:
            LOG.debug(f"{x[0]} | {x[1]} | {x[2]}")
        if len(list_tuples) > 0:
            (vid, vehplan, delta_cfv) = min(list_tuples, key=lambda x:x[2])
            self.tmp_assignment[rid_struct] = vehplan
            offer = self._create_user_offer(prq, sim_time, vehplan)
            LOG.debug(f"new offer for rid {rid_struct} : {offer}")
        else:
            LOG.debug(f"rejection for rid {rid_struct}")
            self._create_rejection(prq, sim_time)
        # record cpu time
        dt = round(time.perf_counter() - t0, 5)
        old_dt = self._get_current_dynamic_fleetcontrol_value(sim_time, G_FCTRL_CT_RQU)
        if old_dt is None:
            new_dt = dt
        else:
            new_dt = old_dt + dt
        output_dict = {G_FCTRL_CT_RQU: new_dt}
        self._add_to_dynamic_fleetcontrol_output(sim_time, output_dict)
        
    def _parcel_request(self, parcel_traveler : RequestBase, sim_time : int):
        """ this function should do the same as the usual user request function just for parcels
        no direct parcel customer - operator interaction is modelled, instead it is assumed that all parcels are already booked. 
        therefore parcels are just added to the operator database and a quasi-offer is created (an offer with fixed entries to enable the customer acceptance in the fleetsim class)
        :param parcel_traveler: request class representing a parcel
        :param sim_time: current simulation time"""
        t0 = time.perf_counter()
        LOG.debug(f"Incoming parcel request {parcel_traveler.__dict__} at time {sim_time}")
        parcel_prq = ParcelPlanRequest(parcel_traveler, self.routing_engine, earliest_pickup_time=self.parcel_earliest_pickup_time,
                                       latest_pickup_time=self.parcel_latest_pickup_time, earliest_drop_off_time=self.parcel_earliest_dropoff_time,
                                       latest_drop_off_time=self.parcel_latest_dropoff_time, boarding_time=self.parcel_const_bt)
        rid_struct = parcel_prq.get_rid_struct()
        self.parcel_dict[rid_struct] = parcel_prq
        self.rq_dict[rid_struct] = parcel_prq
        
        if parcel_prq.o_pos == parcel_prq.d_pos:
            LOG.debug(f"automatic decline for rid {rid_struct}!")
            self._create_rejection(parcel_prq, sim_time)
            return
        
        list_tuples = insert_parcel_prq_in_selected_veh_list(self.sim_vehicles, self.veh_plans, parcel_prq, self.vr_ctrl_f,
                                                            self.routing_engine, self.rq_dict, sim_time, self.const_bt, self.add_bt,
                                                            allow_parcel_pu_with_ob_cust=self.allow_parcel_pu_with_ob_cust)
        #list_tuples = insertion_with_heuristics(sim_time, prq, self, force_feasible_assignment=True)
        LOG.debug("list insertion solutions: {}".format(list_tuples))
        if len(list_tuples) > 0:
            (vid, vehplan, delta_cfv) = min(list_tuples, key=lambda x:x[2])
            self.tmp_assignment[rid_struct] = vehplan
            offer = self._create_parcel_offer(parcel_prq, sim_time, assigned_vehicle_plan=vehplan)
            LOG.debug(f"new offer for rid {rid_struct} : {offer}")
        else:
            LOG.debug(f"rejection for rid {rid_struct}")
            self._create_rejection(parcel_prq, sim_time)
        # record cpu time
        dt = round(time.perf_counter() - t0, 5)
        old_dt = self._get_current_dynamic_fleetcontrol_value(sim_time, G_FCTRL_CT_RQU)
        if old_dt is None:
            new_dt = dt
        else:
            new_dt = old_dt + dt
        output_dict = {G_FCTRL_CT_RQU: new_dt}
        self._add_to_dynamic_fleetcontrol_output(sim_time, output_dict)

    def user_confirms_booking(self, rid, simulation_time):
        """This method is used to confirm a customer booking. This can trigger some database processes.

        :param rid: request id
        :type rid: int
        :param simulation_time: current simulation time
        :type simulation_time: float
        """
        super().user_confirms_booking(rid, simulation_time)
        LOG.debug(f"user confirms booking {rid} at {simulation_time}")
        prq = self.rq_dict[rid]
        new_vehicle_plan = self.tmp_assignment[rid]
        vid = new_vehicle_plan.vid
        veh_obj = self.sim_vehicles[vid]
        self.assign_vehicle_plan(veh_obj, new_vehicle_plan, simulation_time)
        del self.tmp_assignment[rid]
        
    def _full_remove(self, veh_obj : SimulationVehicle, veh_plan : VehiclePlan, remove_rid, sim_time : int, 
                  routing_engine : NetworkBase, obj_function, rq_dict : Dict[Any, PlanRequest], std_bt : int, add_bt : int) -> VehiclePlan:
        new_veh_plan = simple_remove(veh_obj, veh_plan, remove_rid, sim_time, routing_engine, obj_function, rq_dict, std_bt, add_bt)
        planstops = []
        for ps in new_veh_plan.list_plan_stops:
            if len(ps.get_list_boarding_rids()) == 0 and len(ps.get_list_alighting_rids()) == 0:
                continue
            else:
                planstops.append(ps)
        new_new_vehplan = VehiclePlan(veh_obj, sim_time, routing_engine, planstops)
        return new_new_vehplan
    
    def _rid_is_in_boarding_process(self, rid, vid):
        """ checks if vehicle is currently boarding and the request involved in this boarding process
        :return: 0 -> not boarding; 1 -> boarding; -1 -> alighting"""
        if self.sim_vehicles[vid].status == VRL_STATES.BOARDING:
            is_ob = False
            for ob_traveller in self.sim_vehicles[vid].pax:
                if ob_traveller.get_rid_struct() == rid:
                    is_ob = True
                    break
            if is_ob:
                veh_plan = self.veh_plans[vid]
                if len(veh_plan.list_plan_stops) > 0:
                    current_stop = veh_plan.list_plan_stops[0]
                    if rid in current_stop.get_list_boarding_rids():    # driver does not load parcel
                        return 1
                    if rid in current_stop.get_list_alighting_rids():
                        return -1
        return 0           

    def user_cancels_request(self, rid, simulation_time, bring_to_depot_ext = False):
        """This method is used to confirm a customer cancellation. This can trigger some database processes.

        :param rid: request id
        :type rid: int
        :param simulation_time: current simulation time
        :type simulation_time: float
        :return: bool, True, if request should be deleted in demand class
        """
        LOG.debug(f"user cancels request {rid} at {simulation_time}")
        prq = self.rq_dict[rid]
        delete_rid = True
        ignore_next_endboarding = False
        to_deactivate = False
        if prq.is_parcel():
            if self.rid_to_assigned_vid.get(rid) is not None:
                LOG.info(f"late rejection of parcel request {rid} at {simulation_time}")
                vid = self.rid_to_assigned_vid[rid]
                currently_in_boarding = self._rid_is_in_boarding_process(rid, vid)
                if currently_in_boarding != 0:
                    LOG.info(" -> currently in boarding process: {}".format(currently_in_boarding))
                    if currently_in_boarding == 1:
                        LOG.info(" -> assume driver didnt load parcel")
                        bring_to_depot = False
                        delete_rid = True
                    else:
                        bring_to_depot = True
                        delete_rid = False
                    try:
                        self._reject_later[vid][rid] = bring_to_depot
                    except KeyError:
                        self._reject_later[vid] = {rid : bring_to_depot}
                    return delete_rid, ignore_next_endboarding
                
                if len(self.veh_plans[vid].list_plan_stops) > 0 and self.veh_plans[vid].list_plan_stops[-1].is_inactive():
                    to_deactivate = True
                    self.activate_vehicle(vid, simulation_time)
                veh_plan = self.veh_plans[vid]
                new_veh_plan = self._full_remove(self.sim_vehicles[vid], veh_plan, rid, simulation_time, self.routing_engine, self.vr_ctrl_f, self.rq_dict, self.const_bt, self.add_bt)
                
                bring_to_depot = False
                if bring_to_depot_ext:
                    for ob_traveller in self.sim_vehicles[vid].pax:
                        if ob_traveller.get_rid_struct() == rid:
                            bring_to_depot = True

                if not bring_to_depot:
                    for ob_traveller in self.sim_vehicles[vid].pax:
                        if ob_traveller.get_rid_struct() == rid:
                            self.sim_vehicles[vid].pax.remove(ob_traveller)
                if bring_to_depot:   # driver has to bring parcel to TUM
                    LOG.info(" -> bring to depot")
                    delete_rid = False
                    depot = self.op_charge_depot_infra.find_nearest_free_depot(self.sim_vehicles[vid].pos, check_free=False)
                    depot_pos = depot.pos
                    found = False
                    for i, ps in enumerate(new_veh_plan.list_plan_stops):
                        if ps.get_pos() == depot_pos: # insert deboarding
                            new_deboarding = ps.get_list_alighting_rids()[:] + [rid]
                            ept, lbt, mtt, lat = ps.get_boarding_time_constraint_dicts()
                            new_ps = BoardingPlanStop(ps.get_pos(), boarding_dict={1:ps.get_list_boarding_rids(), -1:new_deboarding},
                                                        max_trip_time_dict=mtt, latest_arrival_time_dict=lat, earliest_pickup_time_dict=ept, latest_pickup_time_dict=lbt,
                                                        change_nr_parcels=ps.get_change_nr_parcels()-prq.parcel_size, duration=self.parcel_const_bt, change_nr_pax=ps.get_change_nr_pax())
                            new_veh_plan.list_plan_stops[i] = new_ps 
                            found = True
                            LOG.debug(" -> depot already assigned")
                            break
                    if not found:
                        LOG.debug("add depot at end")
                        new_ps = BoardingPlanStop(depot_pos, boarding_dict={-1 : [rid]},
                                                    change_nr_pax=-prq.parcel_size, duration=self.parcel_const_bt)
                        if len(new_veh_plan.list_plan_stops) == 0 or new_veh_plan.list_plan_stops[-1].is_inactive():
                            new_veh_plan.list_plan_stops.append(new_ps)
                        else:   # add before inactive ps
                            new_ps.set_locked()
                            new_veh_plan.list_plan_stops = new_veh_plan.list_plan_stops[:-1] + [new_ps, new_veh_plan.list_plan_stops[-1]]
                                
                if self.sim_vehicles[vid].status == VRL_STATES.BOARDING:
                    LOG.info(" -> vid is boarding")
                    if bring_to_depot:
                        ignore_next_endboarding = True
                    if len(new_veh_plan.list_plan_stops) == 0 or veh_plan.list_plan_stops[0].get_pos() != new_veh_plan.list_plan_stops[0].get_pos():
                        LOG.info(" -> insert empty boarding stop!")
                        empty_boarding_ps = BoardingPlanStop(veh_plan.list_plan_stops[0].get_pos())
                        new_veh_plan.list_plan_stops = [empty_boarding_ps] + new_veh_plan.list_plan_stops[:]
                self.assign_vehicle_plan(self.sim_vehicles[vid], new_veh_plan, simulation_time)
                if to_deactivate:
                    self.deactivate_vehicle(vid, simulation_time)

            prev_assignment = self.tmp_assignment.get(rid)
            if prev_assignment:
                del self.tmp_assignment[rid]
            
            if delete_rid:
                del self.rq_dict[rid]
                try:
                    del self.parcel_dict[rid]
                except KeyError:
                    pass
        else:
            if self.rid_to_assigned_vid.get(rid) is not None:
                LOG.info(f"late rejection of request {rid} at {simulation_time}")
                vid = self.rid_to_assigned_vid[rid]
                currently_in_boarding = self._rid_is_in_boarding_process(rid, vid)
                if currently_in_boarding != 0:
                    LOG.info(" -> currently in boarding process: {}".format(currently_in_boarding))
                    try:
                        self._reject_later[vid][rid] = False
                    except KeyError:
                        self._reject_later[vid] = {rid : False}
                    return True, False
                
                if len(self.veh_plans[vid].list_plan_stops) > 0 and self.veh_plans[vid].list_plan_stops[-1].is_inactive():
                    to_deactivate = True
                    self.activate_vehicle(vid, simulation_time)
                veh_plan = self.veh_plans[vid]
                new_veh_plan = self._full_remove(self.sim_vehicles[vid], veh_plan, rid, simulation_time, self.routing_engine, self.vr_ctrl_f, self.rq_dict, self.const_bt, self.add_bt)
                for ob_traveller in self.sim_vehicles[vid].pax:
                    if ob_traveller.get_rid_struct() == rid:
                        self.sim_vehicles[vid].pax.remove(ob_traveller)
                if self.sim_vehicles[vid].status == VRL_STATES.BOARDING:
                    LOG.info(" -> vid is boarding")
                    if len(new_veh_plan.list_plan_stops) == 0 or veh_plan.list_plan_stops[0].get_pos() != new_veh_plan.list_plan_stops[0].get_pos():
                        LOG.info(" -> insert empty boarding stop!")
                        empty_boarding_ps = BoardingPlanStop(veh_plan.list_plan_stops[0].get_pos())
                        new_veh_plan.list_plan_stops = [empty_boarding_ps] + new_veh_plan.list_plan_stops[:]
                self.assign_vehicle_plan(self.sim_vehicles[vid], new_veh_plan, simulation_time)
                if to_deactivate:
                    self.deactivate_vehicle(vid, simulation_time)

            prev_assignment = self.tmp_assignment.get(rid)
            if prev_assignment:
                del self.tmp_assignment[rid]
            del self.rq_dict[rid]
            
        return delete_rid, ignore_next_endboarding
    
    def trigger_end_boarding(self, vehicle_id, simulation_time):
        reject_later_vid = self._reject_later.get(vehicle_id, {}).copy()
        self._reject_later[vehicle_id] = {}
        for rid , bring_to_depot in reject_later_vid.items():
            LOG.info("after boarding rejection of rid {} from {}".format(rid, vehicle_id))
            self.user_cancels_request(rid, simulation_time, bring_to_depot_ext=bring_to_depot)
        
        
    def acknowledge_boarding(self, rid, vid, simulation_time):
        """This method can trigger some database processes whenever a passenger is starting to board a vehicle.

        :param rid: request id
        :type rid: int
        :param vid: vehicle id
        :type vid: int
        :param simulation_time: current simulation time
        :type simulation_time: float
        """
        LOG.debug(f"acknowledge boarding {rid} in {vid} at {simulation_time}")
        self.rq_dict[rid].set_pickup(vid, simulation_time)

    def acknowledge_alighting(self, rid, vid, simulation_time):
        """This method can trigger some database processes whenever a passenger is finishing to alight a vehicle.

        :param rid: request id
        :type rid: int
        :param vid: vehicle id
        :type vid: int
        :param simulation_time: current simulation time
        :type simulation_time: float
        """
        LOG.debug(f"acknowledge alighting {rid} from {vid} at {simulation_time}")
        prq = self.rq_dict[rid]
        if prq.is_parcel():
            try:
                del self.parcel_dict[rid]
            except KeyError:
                pass
        del self.rq_dict[rid]
        del self.rid_to_assigned_vid[rid]
        
    def compute_VehiclePlan_utility(self, simulation_time, veh_obj, vehicle_plan):
        """This method computes the utility of a given plan and returns the value.

        :param simulation_time: current simulation time
        :type simulation_time: float
        :param veh_obj: vehicle object
        :type veh_obj: SimulationVehicle
        :param vehicle_plan: vehicle plan in question
        :type vehicle_plan: VehiclePlan
        :return: utility of vehicle plan
        :rtype: float
        """
        vehicle_plan.utility = self.vr_ctrl_f(simulation_time, veh_obj, vehicle_plan, self.rq_dict, self.routing_engine)
        return vehicle_plan.utility
    
    def _create_user_offer(self, prq : PlanRequest, simulation_time : int, assigned_vehicle_plan : VehiclePlan=None, offer_dict_without_plan : dict={}):
        """ creating the offer for a requests

        :param prq: plan request
        :type prq: PlanRequest obj
        :param simulation_time: current simulation time
        :type simulation_time: int
        :param assigned_vehicle_plan: vehicle plan of initial solution to serve this request
        :type assigned_vehicle_plan: VehiclePlan None
        :param offer_dict_without_plan: can be used to create an offer that is not derived from a vehicle plan
                    entries will be used to create/extend offer
        :type offer_dict_without_plan: dict or None
        :return: offer for request
        :rtype: TravellerOffer
        """
        if assigned_vehicle_plan is not None:
            pu_time, do_time = assigned_vehicle_plan.pax_info.get(prq.get_rid_struct())
            # offer = {G_OFFER_WAIT: pu_time - simulation_time, G_OFFER_DRIVE: do_time - pu_time,
            #          G_OFFER_FARE: int(prq.init_direct_td * self.dist_fare + self.base_fare)}
            offer = TravellerOffer(prq.get_rid_struct(), self.op_id, pu_time - prq.rq_time, do_time - pu_time,
                                   self._compute_fare(simulation_time, prq, assigned_vehicle_plan))
            prq.set_service_offered(offer)  # has to be called
        else:
            offer = self._create_rejection(prq, simulation_time)
        return offer
    
    def _create_parcel_offer(self, prq : ParcelPlanRequest, simulation_time : int, assigned_vehicle_plan : VehiclePlan=None, offer_dict_without_plan : dict={}):
        """ creating the offer for a parcel requests
        this is just a quasi-offer. entries cannot be used for mode choice. all offers just get standard values to enforce the customer to accept the service because
        customer-operator interactions are not included for these fleet controls

        :param prq: plan request
        :type prq: PlanRequest obj
        :param simulation_time: current simulation time
        :type simulation_time: int
        :param assigned_vehicle_plan: vehicle plan of initial solution to serve this request
        :type assigned_vehicle_plan: VehiclePlan None
        :param offer_dict_without_plan: can be used to create an offer that is not derived from a vehicle plan
                    entries will be used to create/extend offer
        :type offer_dict_without_plan: dict or None
        :return: offer for request
        :rtype: TravellerOffer
        """
        return self._create_user_offer(prq, simulation_time, assigned_vehicle_plan=assigned_vehicle_plan, offer_dict_without_plan=offer_dict_without_plan)

    def deactivate_vehicle(self, vid, sim_time):
        """ this function blocks all currently assigned route blocks and adds a routeblock to send
        the vehicle to the next depot to remove it from service 
        :param vid: vehicle id
        :param sim_time: current simulation time"""
        LOG.info(f"deactivate vehicle {vid} at time {sim_time}.")
        veh_obj = self.sim_vehicles[vid]
        final_veh_pos = veh_obj.pos
        if veh_obj.assigned_route:
            final_veh_pos = veh_obj.assigned_route[-1].destination_pos
        lock_ps = RoutingTargetPlanStop(final_veh_pos, duration=LARGE_INT, locked=True, planstop_state=G_PLANSTOP_STATES.INACTIVE)
        ass_plan = self.veh_plans[veh_obj.vid]
        ass_plan.add_plan_stop(lock_ps, veh_obj, sim_time, self.routing_engine)
        self.lock_current_vehicle_plan(veh_obj.vid)
        self.assign_vehicle_plan(veh_obj, ass_plan, sim_time)

    def activate_vehicle(self, vid, sim_time):
        """ this function activates the vehicle and removes blocking states to make it 
        available for service
        :param vid: vehicle id
        :param sim_time: current simulation time"""
        LOG.info(f"activating vehicle {vid} at time {sim_time}.")
        print(f"activate vehicle {vid} of vehicles {self.sim_vehicles}")
        veh_obj = self.sim_vehicles[vid]
        ass_plan = self.veh_plans[veh_obj.vid]
        new_ps_list = []
        for ps in ass_plan.list_plan_stops:
            if not ps.is_inactive():
                new_ps_list.append(ps)
        ass_plan.list_plan_stops = new_ps_list  # TODO not sure if this works correctly
        self.assign_vehicle_plan(veh_obj, ass_plan, sim_time, force_assign=True)
        
    def _lock_vid_rid_pickup(self, sim_time, vid, rid):
        return super()._lock_vid_rid_pickup(sim_time, vid, rid)
    
    def _prq_from_reservation_to_immediate(self, rid, sim_time):
        return super()._prq_from_reservation_to_immediate(rid, sim_time)
    
    def assign_vehicle_plan(self, veh_obj: SimulationVehicle, vehicle_plan: VehiclePlan, sim_time: int, force_assign: bool = False, assigned_charging_task: Tuple[Tuple[str, int], ChargingProcess] = None, add_arg: Any = None):
        #vehicle_plan, _ = self._lock_next_planstop(vehicle_plan)
        vehicle_plan.update_tt_and_check_plan(veh_obj, sim_time, self.routing_engine, keep_feasible=True)
        self.compute_VehiclePlan_utility(sim_time, veh_obj, vehicle_plan)
        return super().assign_vehicle_plan(veh_obj, vehicle_plan, sim_time, force_assign, assigned_charging_task, add_arg)
    
    def change_prq_time_constraints(self, sim_time: int, rid: Any, new_lpt: int, new_ept: int = None):
        return super().change_prq_time_constraints(sim_time, rid, new_lpt, new_ept)
    
    def lock_current_vehicle_plan(self, vid: int):
        return super().lock_current_vehicle_plan(vid)
    
    def receive_status_update(self, vid: int, simulation_time: int, list_finished_VRL, force_update: bool = True):
        r = super().receive_status_update(vid, simulation_time, list_finished_VRL, force_update)
        locked_vp, lock_set = self._lock_next_planstop(self.veh_plans[vid])
        if lock_set:
            self.assign_vehicle_plan(self.sim_vehicles[vid], locked_vp, simulation_time)
    
    def user_request(self, rq: RequestBase, simulation_time: int):
        if rq.is_parcel:    # request of parcel
            return self._parcel_request(rq, simulation_time)
        else:   # request of person
            return self._person_request(rq, simulation_time)
        
    def _lock_next_planstop(self, veh_plan : VehiclePlan):
        lock_set = False
        if len(veh_plan.list_plan_stops) > 0:
            if not veh_plan.list_plan_stops[0].is_locked():
                veh_plan.list_plan_stops[0].set_locked(True)
                lock_set = True
        return veh_plan, lock_set
    
    def rebuild_vehicle_route(self, veh_id, route_list, associated_traveler_objects, sim_time):
        """ this function is used to rebuild a route for a vehicle after the fleetctrl crashed to reinitialize the fleetctrl and
        sync its state to the backend
        :param veh_id: vehicle id
        :param route_list: list of tuple (network_node, boarding dict {1: list boarding rids, -1: list deboarding rids}
        :param associated_traveler_objects: list of traveler objectes associated to the vehicle
        :param sim_time: current simulation time"""
        for traveler in associated_traveler_objects:
            if traveler.is_parcel:
                prq = ParcelPlanRequest(traveler, self.routing_engine, earliest_pickup_time=self.parcel_earliest_pickup_time,
                                            latest_pickup_time=self.parcel_latest_pickup_time, earliest_drop_off_time=self.parcel_earliest_dropoff_time,
                                            latest_drop_off_time=self.parcel_latest_dropoff_time, boarding_time=self.parcel_const_bt)
                rid_struct = prq.get_rid_struct()
                self.parcel_dict[rid_struct] = prq
                self.rq_dict[rid_struct] = prq
            else:
                prq = PlanRequest(traveler, self.routing_engine, min_wait_time=self.min_wait_time,
                                max_wait_time=self.max_wait_time,
                                max_detour_time_factor=self.max_dtf, max_constant_detour_time=self.max_cdt,
                                add_constant_detour_time=self.add_cdt, min_detour_time_window=self.min_dtw,
                                boarding_time=self.const_bt)
                rid_struct = prq.get_rid_struct()
                self.rq_dict[rid_struct] = prq
        list_plan_stops = []
        for node, boarding_dict in route_list:
            max_trip_time_dict = {}
            latest_arrival_time_dict = {}
            earliest_pickup_time_dict = {}
            latest_pickup_time_dict = {}
            change_nr_persons = 0
            change_nr_parcels = 0
            duration = 0
            for bd_rid in boarding_dict.get(1, []):
                prq = self.rq_dict[bd_rid]
                prq_o_stop_pos, prq_t_pu_earliest, prq_t_pu_latest = prq.get_o_stop_info()
                earliest_pickup_time_dict[bd_rid] = prq_t_pu_earliest
                latest_pickup_time_dict[bd_rid] = prq_t_pu_latest
                if prq.is_parcel():
                    change_nr_parcels += prq.parcel_size
                    if duration < self.parcel_const_bt:
                        duration = self.parcel_const_bt
                else:
                    change_nr_persons += prq.nr_pax
                    if duration < self.const_bt:
                        duration = self.const_bt
            for bd_rid in boarding_dict.get(-1, []):
                prq = self.rq_dict[bd_rid]
                d_stop_pos, prq_t_do_latest, prq_max_trip_time = prq.get_d_stop_info()
                max_trip_time_dict[bd_rid] = prq_max_trip_time
                latest_arrival_time_dict[bd_rid] = prq_t_do_latest
                if prq.is_parcel():
                    change_nr_parcels -= prq.parcel_size
                    if duration < self.parcel_const_bt:
                        duration = self.parcel_const_bt
                else:
                    change_nr_persons -= prq.nr_pax
                    if duration < self.const_bt:
                        duration = self.const_bt
            ps = BoardingPlanStop(node, boarding_dict=boarding_dict, max_trip_time_dict=max_trip_time_dict, latest_arrival_time_dict=latest_arrival_time_dict,
                                  earliest_pickup_time_dict=earliest_pickup_time_dict, latest_pickup_time_dict=latest_pickup_time_dict, change_nr_parcels=change_nr_parcels, change_nr_pax=change_nr_persons)
            list_plan_stops.append(ps)
        veh_plan = VehiclePlan(self.sim_vehicles[veh_id], sim_time, self.routing_engine, list_plan_stops)
        self.assign_vehicle_plan(self.sim_vehicles[veh_id], veh_plan, sim_time, force_assign=True)
        for rid in veh_plan.get_involved_request_ids():
            if self.parcel_dict.get(rid):
                self._create_parcel_offer(self.rq_dict[rid], sim_time, assigned_vehicle_plan=veh_plan)       
            else:
                self._create_user_offer(self.rq_dict[rid], sim_time, assigned_vehicle_plan=veh_plan)        
        
    def _build_VRLs(self, vehicle_plan : VehiclePlan, veh_obj : SimulationVehicle, sim_time : int) -> List[VehicleRouteLeg]:
        """This method builds VRL for simulation vehicles from a given Plan. Since the vehicle could already have the
        VRL with the correct route, the route from veh_obj.assigned_route[0] will be used if destination positions
        are matching

        :param vehicle_plan: vehicle plan to be converted
        :param veh_obj: vehicle object to which plan is applied
        :param sim_time: current simulation time
        :return: list of VRLs according to the given plan
        """
        LOG.debug("build VRLs from RPP_APP_FleetControl")
        list_vrl = []
        c_pos = veh_obj.pos
        c_time = sim_time
        for i, pstop in enumerate(vehicle_plan.list_plan_stops):
            # TODO: The following should be made as default behavior to delegate the specific tasks (e.g. boarding,
            #  charging etc) to the StationaryProcess class. The usage of StationaryProcess class can significantly
            #  simplify the code
            # i dont think the following lines work
            # if pstop.stationary_task is not None:
            #     list_vrl.append(self._get_veh_leg(pstop))
            #     continue
            boarding_dict = {1: [], -1: []}
            stationary_process = None
            if len(pstop.get_list_boarding_rids()) > 0 or len(pstop.get_list_alighting_rids()) > 0:
                boarding = True
                for rid in pstop.get_list_boarding_rids():
                    boarding_dict[1].append(self.rq_dict[rid])
                for rid in pstop.get_list_alighting_rids():
                    boarding_dict[-1].append(self.rq_dict[rid])
            else:
                boarding = False
            if pstop.get_charging_power() > 0:
                charging = True
                stationary_process = self._active_charging_processes[pstop.get_charging_task_id()]
            else:
                charging = False
            #if pstop.get_departure_time(0) > LARGE_INT:
            if pstop.get_state() == G_PLANSTOP_STATES.INACTIVE:
                inactive = True
            else:
                inactive = False
            if pstop.is_locked_end():
                reservation = True
            else:
                reservation = False
            if pstop.get_departure_time(0) != 0:
                planned_stop = True
                repo_target = False
            else:
                planned_stop = False
                repo_target = True
            if not (i == 0 and veh_obj.status == VRL_STATES.BOARDING) or c_pos != pstop.get_pos():
                # driving vrl
                if boarding:
                    status = VRL_STATES.ROUTE
                elif charging:
                    status = VRL_STATES.TO_CHARGE
                elif inactive:
                    status = VRL_STATES.TO_DEPOT
                elif reservation:
                    status = VRL_STATES.TO_RESERVATION
                else:
                    # repositioning
                    status = VRL_STATES.REPOSITION
                # use empty boarding dict for this VRL, but do not overwrite boarding_dict!
                if pstop.get_earliest_start_time() - c_time > self.begin_approach_buffer_time \
                        and pstop.get_earliest_start_time() - c_time - self.routing_engine.return_travel_costs_1to1(c_pos, pstop.get_pos())[1] > self.begin_approach_buffer_time:
                    # wait at current postion until target can still be reached within self.begin_approach_buffer_time
                    list_vrl.append(VehicleRouteLeg(status, pstop.get_pos(), {1: [], -1: []}, locked=pstop.is_locked(), earliest_start_time=pstop.get_earliest_start_time() - self.begin_approach_buffer_time - self.routing_engine.return_travel_costs_1to1(c_pos, pstop.get_pos())[1]))
                else:
                    list_vrl.append(VehicleRouteLeg(status, pstop.get_pos(), {1: [], -1: []}, locked=pstop.is_locked()))
                c_pos = pstop.get_pos()
                _, c_time = pstop.get_planned_arrival_and_departure_time()
            # stop vrl
            if boarding and charging:
                status = VRL_STATES.BOARDING_WITH_CHARGING
            elif boarding:
                status = VRL_STATES.BOARDING
            elif charging:
                status = VRL_STATES.CHARGING
            elif inactive:
                status = VRL_STATES.OUT_OF_SERVICE
            elif planned_stop:
                status = VRL_STATES.PLANNED_STOP
            elif repo_target:
                status = VRL_STATES.REPO_TARGET
            else:
                # TODO # after ISTTT: add other states if necessary; for now assume vehicle idles
                status = VRL_STATES.IDLE
            if status != VRL_STATES.IDLE:
                dur, edep = pstop.get_duration_and_earliest_departure()
                earliest_start_time = pstop.get_earliest_start_time()
                #LOG.debug("vrl earliest departure: {} {}".format(dur, edep))
                if edep is not None:
                    LOG.warning("absolute earliest departure not implementen in build VRL!")
                    departure_time = edep
                else:
                    departure_time = -LARGE_INT
                if dur is not None:
                    stop_duration = dur
                else:
                    stop_duration = 0
                _, c_time = pstop.get_planned_arrival_and_departure_time()
                list_vrl.append(VehicleRouteLeg(status, pstop.get_pos(), boarding_dict, pstop.get_charging_power(),
                                                duration=stop_duration, earliest_start_time=earliest_start_time, earliest_end_time=departure_time,
                                                locked=pstop.is_locked(), stationary_process=stationary_process))
        return list_vrl