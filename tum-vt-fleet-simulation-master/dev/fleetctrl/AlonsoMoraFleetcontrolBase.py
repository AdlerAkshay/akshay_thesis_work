import logging
import time

from src.fleetctrl.FleetControlBase import FleetControlBase, VehiclePlan, PlanRequest
from src.fleetctrl.pooling.objectives import return_pooling_objective_function
from src.fleetctrl.pooling.batch.AlonsoMora.AlonsoMoraParallelization import ParallelizationManager
from src.fleetctrl.pooling.batch.AlonsoMora.AlonsoMoraAssignment import AlonsoMoraAssignment 
from src.simulation.Offers import TravellerOffer
from src.fleetctrl.pooling.GeneralPoolingFunctions import get_assigned_rids_from_vehplan
from src.misc.globals import *


LOG = logging.getLogger(__name__)
LARGE_INT = 100000


class AlonsoMoraFleetControlBase(FleetControlBase):
    def __init__(self, op_id, operator_attributes, list_vehicles, routing_engine, zone_system, scenario_parameters,
                 dir_names, charging_management=None):
        """The specific attributes for the fleet control module are initialized. Strategy specific attributes are
        introduced in the children classes.

        THIS CLASS IS FOR INHERITANCE ONLY.
        this class can be used for common ride-pooling studies using the alonso-mora algorithm for optimisation
        triggered in the time_trigger() method
        customers are introduced by the user_request() function, for each customer requesting a trip, either the method
        user_confirms_booking() or user_cancels_request has to be called!


        DEPENDING ON THE MODELLED CUSTOMER-FLEETOPERATOR-INTERACTION-FOLLOWING METHODS HAVE TO BE EXTENDED.
        - user_request()
        - user_confirms_booking()
        - user_cancels_request()
        - time_trigger()

        :param op_id: operator id
        :type op_id: int
        :param operator_attributes: dictionary with keys from globals and respective values
        :type operator_attributes: dict
        :param list_vehicles: simulation vehicles; their assigned plans should be instances of the VehicleRouteLeg class
        :type list_vehicles: list
        :param routing_engine: routing engine
        :type routing_engine: Network
        :param scenario_parameters: access to all scenario parameters (if necessary)
        :type scenario_parameters: dict
        """
        LOG.warning("DEBRECATED WARNING: AlonsoMoraFleetcontrolBase is no longer up to date: use RidePoolingBatchAssingmentFleetcontrol instead!")
        super().__init__(op_id, operator_attributes, list_vehicles, routing_engine, zone_system, scenario_parameters,
                         dir_names, charging_management=charging_management)
        self.sim_time = scenario_parameters[G_SIM_START_TIME]
        self.rid_to_assigned_vid = {}
        self.pos_veh_dict = {}  # pos -> list_veh
        # additional control scenario input parameters
        # define vr-assignment control objective function
        self.vr_ctrl_f = return_pooling_objective_function(operator_attributes[G_OP_VR_CTRL_F])

        self.optimisation_time_step = operator_attributes[G_RA_REOPT_TS]
        self.alonso_mora_timeout_per_veh_tree = operator_attributes.get(G_RA_TB_TO_PER_VEH, None)
        self.optimisation_timeout = operator_attributes.get(G_RA_OPT_TO, None)
        self.max_rv_con = operator_attributes.get(G_RA_MAX_VR, None)
        self.applied_heuristic = operator_attributes.get(G_RA_HEU, None)

        self.Parallelization_Manager = None
        n_cores = scenario_parameters[G_SLAVE_CPU]
        self.AM_Module = AlonsoMoraAssignment(self, self.routing_engine, self.sim_time,
                                              self.vr_ctrl_f, operator_attributes,
                                              optimisation_cores=n_cores)
        
        # dynamic dicts to update database
        self.new_requests = {}  # rid -> prq (new)
        self.requests_that_changed = {}  # rid -> prq (already here but new constraints)
        self.active_request_offers = {} #rid -> TravellerOffers
        self.new_travel_times_loaded = False    # indicates if new travel times have been loaded on the routing engine

        # init dynamic output -> children fleet controls should check correct usage
        self._init_dynamic_fleetcontrol_output_key(G_FCTRL_CT_RQU)
        self._init_dynamic_fleetcontrol_output_key(G_FCTRL_CT_RQB)

    def add_init(self, operator_attributes, scenario_parameters):
        super().add_init(operator_attributes, scenario_parameters)
        n_cores = scenario_parameters[G_SLAVE_CPU]
        LOG.info("add init: {}".format(n_cores))
        if n_cores > 1 and self.Parallelization_Manager is None:
            LOG.info("initialize Parallelization Manager")
            self.Parallelization_Manager = ParallelizationManager(n_cores, scenario_parameters, self.dir_names)
        if self.Parallelization_Manager is not None:
            self.AM_Module.register_parallelization_manager(self.Parallelization_Manager)

    def register_parallelization_manager(self, Parallelization_Manager):
        """ this method can be used within the add_init of the fleet simulation to define
        a Parallelization Manager that is shared between multiple operators 
        (add_init of the fleetcontrol has to be called after this one)
        :param Parallelization_Manager: object to manage parallelization in AM algorithm
        :type Parallelization_Manager: src.pooling.batch.AlonsoMora.AlonsoMoraParallelization.ParallelizationManager
        """
        self.Parallelization_Manager = Parallelization_Manager

    def receive_status_update(self, vid, simulation_time, list_finished_VRL, force_update=True):
        """This method can be used to update plans and trigger processes whenever a simulation vehicle finished some
         VehicleRouteLegs.

        :param vid: vehicle id
        :type vid: int
        :param simulation_time: current simulation time
        :type simulation_time: float
        :param list_finished_VRL: list of VehicleRouteLeg objects
        :type list_finished_VRL: list
        :param force_update: force vehicle plan update (can be turned off in normal update step)
        :type force_update: bool
        """
        self.sim_time = simulation_time
        veh_obj = self.sim_vehicles[vid]
        if simulation_time%self.optimisation_time_step == 0:
            force_update=True
        super().receive_status_update(vid, simulation_time, list_finished_VRL, force_update=force_update)
        # track done VRLs for updating DB in optimisation-step
        try:
            self.vid_finished_VRLs[vid] += list_finished_VRL
        except KeyError:
            self.vid_finished_VRLs[vid] = list_finished_VRL
        LOG.debug(f"veh {veh_obj} | after status update: {self.veh_plans[vid]}")

    def user_request(self, rq, sim_time):
        """This method is triggered for a new incoming request. It generally adds the rq to the database.
        WHEN INHERITING THIS FUNCTION AN ADDITIONAL CONTROL STRUCTURE TO CREATE OFFERS NEED TO BE IMPLEMENTED IF NEEDED
        (e.g. the functionality of creating an offer might be extended here)

        :param rq: request object containing all request information
        :type rq: RequestDesign
        :param sim_time: current simulation time
        :type sim_time: float

        """
        LOG.debug(f"Incoming request {rq.__dict__} at time {sim_time}")
        self.sim_time = sim_time
        prq = PlanRequest(rq, self.routing_engine, min_wait_time=self.min_wait_time, max_wait_time=self.max_wait_time,
                          max_detour_time_factor=self.max_dtf, max_constant_detour_time=self.max_cdt,
                          add_constant_detour_time=self.add_cdt, min_detour_time_window=self.min_dtw,
                          boarding_time=self.const_bt)

        rid_struct = rq.get_rid_struct()
        self.rq_dict[rid_struct] = prq
        self.AM_Module.add_new_request(rid_struct, prq)
        self.new_requests[rid_struct] = 1

        return {}

    def user_confirms_booking(self, rid, simulation_time):
        """This method is used to confirm a customer booking. This can trigger some database processes.

        WHEN INHERITING THIS FUNCTION ADDITIONAL CONTROL STRUCTURES WHICH DEFINE THE ASSIGNED PLAN MIGHT BE NEEDED
        DEPENDING ON WHERE OFFERS ARE CREATED THEY HAVE TO BE ADDED TO THE DICT self.active_request_offers

        :param rid: request id
        :type rid: int
        :param simulation_time: current simulation time
        :type simulation_time: float
        """
        LOG.debug(f"user confirms booking {rid} at {simulation_time}")
        super().user_confirms_booking(rid, simulation_time)
        self.sim_time = simulation_time
        vid = self.rid_to_assigned_vid.get(rid)
        prq = self.rq_dict.get(rid)
        if vid is not None and prq is not None and prq.get_reservation_flag():
            try:
                self.vid_with_reserved_rids[vid].append(rid)
            except KeyError:
                self.vid_with_reserved_rids[vid] = [rid]
        self.AM_Module.set_request_assigned(rid)
        try:
            del self.active_request_offers[rid]
        except KeyError:
            pass

    def user_cancels_request(self, rid, simulation_time):
        """This method is used to confirm a customer cancellation. This can trigger some database processes.

        WHEN INHERITING THIS FUNCTION AN ADDITIONAL CONTROL STRUCTURE DEFINING WHICH VEHICLE ROUTE SHOULD BE PICKED
        INSTEAD NEEDS TO BE IMPLEMENTED!
        if the currently assigned tour for the rid is needed, first retrieve it by selecting
        self.rid_to_assigned_vid.get(rid) after that call super().user_cancels_request
        additionally a new vehicle plan without the rid has to be registered, in case the optimisation (time_trigger)
        has been called since user_request(rid)
        -> use assign_vehicle_plan() to register the new tour for the optimisation problem

        :param rid: request id
        :type rid: int
        :param simulation_time: current simulation time
        :type simulation_time: float
        """
        self.sim_time = simulation_time
        LOG.debug(f"user cancels request {rid} at {simulation_time}")
        prev_vid = self.rid_to_assigned_vid.get(rid)
        prq = self.rq_dict.get(rid)
        if prev_vid is not None and prq is not None and prq.get_reservation_flag():
            list_reserved_rids = self.vid_with_reserved_rids.get(prev_vid, [])
            if rid in list_reserved_rids:
                list_reserved_rids.remove(rid)
                if list_reserved_rids:
                    self.vid_with_reserved_rids[prev_vid] = list_reserved_rids
                else:
                    del self.vid_with_reserved_rids[prev_vid]
        try:
            del self.rq_dict[rid]
        except KeyError:
            pass
        try:
            del self.rid_to_assigned_vid[rid]
        except KeyError:
            pass
        self.AM_Module.delete_request(rid)
        try:
            del self.active_request_offers[rid]
        except KeyError:
            pass

    def acknowledge_boarding(self, rid, vid, simulation_time):
        """This method can trigger some database processes whenever a passenger is starting to board a vehicle.

        :param rid: request id
        :type rid: int
        :param vid: vehicle id
        :type vid: int
        :param simulation_time: current simulation time
        :type simulation_time: float
        """
        self.sim_time = simulation_time
        LOG.debug(f"acknowledge boarding {rid} in {vid} at {simulation_time}")
        self.rq_dict[rid].set_pickup(vid, simulation_time)
        self.AM_Module.set_database_in_case_of_boarding(rid, vid)

    def acknowledge_alighting(self, rid, vid, simulation_time):
        """This method can trigger some database processes whenever a passenger is finishing to alight a vehicle.

        :param rid: request id
        :type rid: int
        :param vid: vehicle id
        :type vid: int
        :param simulation_time: current simulation time
        :type simulation_time: float
        """
        self.sim_time = simulation_time
        LOG.debug(f"acknowledge alighting {rid} from {vid} at {simulation_time}")
        self.AM_Module.set_database_in_case_of_alighting(rid, vid)
        del self.rq_dict[rid]
        try:
            del self.rid_to_assigned_vid[rid]
        except KeyError:
            pass

    def _prq_from_reservation_to_immediate(self, rid, sim_time):
        """This method is triggered when a reservation request becomes an immediate request.
        All database relevant methods can be triggered from here.

        :param rid: request id
        :param sim_time: current simulation time
        :return: None
        """
        if self.rid_to_assigned_vid.get(rid) is not None:
            self.AM_Module.add_new_request(rid, self.rq_dict[rid], is_allready_assigned=True)
        else:
            self.AM_Module.add_new_request(rid, self.rq_dict[rid])

    def _call_time_trigger_request_batch(self, simulation_time):
        """This method can be used to perform time-triggered processes, e.g. the optimization of the current
        assignments of simulation vehicles of the fleet.

        WHEN INHERITING THIS FUNCTION AN ADDITIONAL CONTROL STRUCTURE TO CREATE OFFERS NEED TO BE IMPLEMENTED IF NEEDED
        DEPENDING ON WHERE OFFERS ARE CREATED THEY HAVE TO BE ADDED TO THE DICT self.active_request_offers

        when overwriting this method super().time_trigger(simulation_time) should be called first

        :param simulation_time: current simulation time
        :type simulation_time: int
        """

        t0 = time.perf_counter()
        self.sim_time = simulation_time
        if self.sim_time % self.optimisation_time_step == 0:
            # LOG.info(f"time for new optimisation at {simulation_time}")
            self.AM_Module.compute_new_vehicle_assignments(self.sim_time, self.vid_finished_VRLs, build_from_scratch=False,
                                                        new_travel_times=self.new_travel_times_loaded)
            # LOG.info(f"new assignments computed")
            self.set_new_assignments()
            self.clear_databases()
            self.AM_Module.clear_databases()
            dt = round(time.perf_counter() - t0, 5)
            output_dict = {G_FCTRL_CT_RQB: dt}
            self._add_to_dynamic_fleetcontrol_output(simulation_time, output_dict)

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
        return self.vr_ctrl_f(simulation_time, veh_obj, vehicle_plan, self.rq_dict, self.routing_engine)

    def assign_vehicle_plan(self, veh_obj, vehicle_plan : VehiclePlan, sim_time : int, force_assign : bool=False
                            , assigned_charging_task=None , add_arg : bool=None):
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
        :param add_arg: set to True, if the vehicle plan is assigned internally by AM-assignment
        :type add_arg: not defined here
        """
        LOG.debug(f"assign vehicle plan for {veh_obj} addarg {add_arg} : {vehicle_plan}")
        super().assign_vehicle_plan(veh_obj, vehicle_plan, sim_time, force_assign=force_assign, assigned_charging_task=assigned_charging_task, add_arg=add_arg)
        if add_arg is None:
            veh_plan_without_rel = vehicle_plan.copy_and_remove_empty_planstops(veh_obj, sim_time, self.routing_engine)
            self.AM_Module.set_assignment(veh_obj.vid, veh_plan_without_rel, is_external_vehicle_plan=True)
        else:
            self.AM_Module.set_assignment(veh_obj.vid, vehicle_plan)

    def set_new_assignments(self):
        """ this function sets the new assignments computed in the alonso-mora-module
        """
        for vid, veh_obj in enumerate(self.sim_vehicles):
            assigned_plan = self.AM_Module.get_optimisation_solution(vid)
            LOG.debug("vid: {} {}".format(vid, assigned_plan))
            rids = get_assigned_rids_from_vehplan(assigned_plan)
            if len(rids) == 0 and len(get_assigned_rids_from_vehplan(self.veh_plans[vid])) == 0:
                LOG.debug("ignore assignment")
                self.AM_Module.set_assignment(vid, None)
                continue
            if assigned_plan is not None:
                LOG.debug(f"assigning new plan for vid {vid} : {assigned_plan}")
                self.assign_vehicle_plan(veh_obj, assigned_plan, self.sim_time, add_arg=True)
            else:
                LOG.debug(f"removing assignment from {vid}")
                assigned_plan = VehiclePlan(veh_obj, self.sim_time, self.routing_engine, [])
                self.assign_vehicle_plan(veh_obj, assigned_plan, self.sim_time, add_arg=True)

    def lock_rid_vid_assignments(self):
        """ this function locks all assignments of new assigned requests to the corresponding vid
        and prevents them from reassignment in the next opt-steps
        """
        for vid, veh_obj in enumerate(self.sim_vehicles):
            assigned_plan = self.AM_Module.get_optimisation_solution(vid)
            # LOG.debug("vid: {} {}".format(vid, assigned_plan))
            rids = get_assigned_rids_from_vehplan(assigned_plan)
            for rid in rids:
                if self.new_requests.get(rid):
                    # LOG.info("lock rid {} to vid {}".format(rid, vid))
                    self.AM_Module.lock_request_to_vehicle(rid, vid)

    def _clear_databases(self):
        """ this function clears dynamic data base entries in fleet control 
        should be called after the optimisation step
        """
        self.new_requests = {}
        self.requests_that_changed = {}
        self.vid_finished_VRLs = {}
        self.new_travel_times_loaded = False

    def inform_network_travel_time_update(self, simulation_time):
        """ triggered if new travel times are available;
        -> the AM database needs to be recomputed
        -> networks on parallel cores need to be synchronized
        """
        self.sim_time = simulation_time
        self.new_travel_times_loaded = True
        if self.Parallelization_Manager is not None:
            self.Parallelization_Manager.update_network(simulation_time)

    def lock_current_vehicle_plan(self, vid):
        super().lock_current_vehicle_plan(vid)
        if hasattr(self, "AM_Module"):
            LOG.debug(" -> also lock in AM")
            assigned_plan = self.veh_plans.get(vid, None)
            if assigned_plan is not None:
                self.AM_Module.set_assignment(vid, assigned_plan, is_external_vehicle_plan=True)
            self.AM_Module.delete_vehicle_database_entries(vid)
            for rid in get_assigned_rids_from_vehplan(assigned_plan):
                self.AM_Module.lock_request_to_vehicle(rid, vid)

    def _lock_vid_rid_pickup(self, sim_time, vid, rid):
        """This method constrains the pick-up of a rid. In the pooling case, the pick-up time is constrained to a very
        short time window. In the hailing case, the Task to serve rid is locked for the vehicle.

        :param sim_time: current simulation time
        :param vid: vehicle id
        :param rid: PlanRequest id
        :return: None
        """
        super()._lock_vid_rid_pickup(sim_time, vid, rid)
        self.AM_Module.lock_request_to_vehicle(rid, vid)

    def change_prq_time_constraints(self, sim_time, rid, new_lpt, new_ept=None):
        """this function registers if time constraints of a requests is changed during the simulation"""
        LOG.debug("change time constraints for rid {}".format(rid))
        prq = self.rq_dict[rid]
        exceed_tw = True
        if new_lpt <= prq.t_pu_latest:
            if new_ept is None or new_ept >= prq.t_pu_earliest:
                exceed_tw = False
        prq.set_new_pickup_time_constraint(new_lpt, new_earliest_pu_time=new_ept)
        ass_vid = self.rid_to_assigned_vid.get(rid)
        if ass_vid is not None:
            self.veh_plans[ass_vid].update_prq_hard_constraints(self.sim_vehicles[ass_vid], sim_time,
                                                                self.routing_engine, prq, new_lpt, new_ept=new_ept,
                                                                keep_feasible=True)
        self.AM_Module.register_change_in_time_constraints(rid, prq, assigned_vid=ass_vid,
                                                           exceeds_former_time_windows=exceed_tw)

    def _create_user_offer(self, prq, simulation_time, assigned_vehicle_plan=None, offer_dict_without_plan={}):
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
            #          G_OFFER_FARE: self._compute_fare(simulation_time, prq, assigned_vehicle_plan)}
            offer = TravellerOffer(prq.get_rid_struct(), self.op_id, pu_time - prq.rq_time, do_time - pu_time,
                                   self._compute_fare(simulation_time, prq, assigned_vehicle_plan))
            prq.set_service_offered(offer)  # has to be called
            self.active_request_offers[prq.get_rid_struct()] = offer
        else:
            offer = TravellerOffer(prq.get_rid(), self.op_id, None, None, None)
            self.active_request_offers[prq.get_rid_struct()] = offer
        return offer

    def get_current_offer(self, rid):
        """ this method returns the currently active offer for the request rid
        if a current offer is active:
            the current TravellerOffer is returned
        if the service is decline and the request didnt leave the system yet:
            a "service_declined" TravellerOffer is returned (at least offered_waiting_time is set to None in TravellerOffer init)
        if an offer is not evaluated yet:
            None is returned

        use the method "_create_user_offer" to create single user offers

        :param rid: request id
        :type rid: int
        :return: TravellerOffer or None for the request
        :rtype: TravellerOffer or None
        """
        return self.active_request_offers.get(rid, None)
