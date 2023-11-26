#### Test class is used to test the performance of two algorithms inheriting from BatchAssignmentAlgorithmDesign.py ####

import logging
from src.fleetctrl.FleetControlBase import FleetControlBase, VehiclePlan, PlanRequest
from src.fleetctrl.pooling.objectives import return_pooling_objective_function
from src.fleetctrl.pooling.batch.AlonsoMora.AlonsoMoraAssignment import AlonsoMoraAssignment
from dev.fleetctrl.pooling.batch.ParallelTempering.ParallelTemperingAssignment import ParallelTemperingAssignment
from src.fleetctrl.pooling.GeneralPoolingFunctions import get_assigned_rids_from_vehplan
from src.misc.globals import *

LOG = logging.getLogger(__name__)
LARGE_INT = 100000

class DoubleAlgorithmFleetcontrol(FleetControlBase):
    def __init__(self, op_id, operator_attributes, list_vehicles, routing_engine, zone_system, scenario_parameters, dir_names, charging_management=None):
        """The specific attributes for the fleet control module are initialized. Strategy specific attributes are
        introduced in the children classes.

        this class can be used to directly compare two batch assignment ridepooling algorithms
        one is used to compute new vehicle-plans that actually will be applied
        the other one solves excactly the same problem and solution and computation speed can be compared

        # TODO # update this class!

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
        super().__init__(op_id, operator_attributes, list_vehicles, routing_engine, zone_system, scenario_parameters, dir_names, charging_management=charging_management)
        self.sim_time = scenario_parameters[G_SIM_START_TIME]
        self.veh_plans = {} # vid -> VehiclePlan
        for veh_obj in self.sim_vehicles:
            vid = veh_obj.vid
            self.veh_plans[vid] = VehiclePlan(veh_obj, self.sim_time, routing_engine, [])
        self.rid_to_assigned_vid = {}
        self.pos_veh_dict = {}  # pos -> list_veh
        # additional control scenario input parameters
        self.max_rv_con = operator_attributes[G_RA_MAX_VR]
        # define vr-assignment control objective function
        self.vr_ctrl_f = return_pooling_objective_function(operator_attributes[G_OP_VR_CTRL_F])

        self.optimisation_time_step = operator_attributes[G_RA_REOPT_TS]
        self.alonso_mora_timeout_per_veh_tree = operator_attributes.get(G_RA_TB_TO_PER_VEH, None)
        self.optimisation_timeout = operator_attributes.get(G_RA_OPT_TO, None)

        self.Parallelisation_Manager = None
        N_cores = 1
        # N_cores = scenario_parameters["n_cpu_per_sim"]
        # if N_cores > 1:
        #     self.Parallelisation_Manager = ParallelizationManager(N_cores, scenario_parameters, dirnames, self.alonso_mora_timeout_per_veh_tree)
        self.AM_Module = AlonsoMoraAssignment(self, self.routing_engine, self.sim_time, self.const_bt, self.add_bt, self.vr_ctrl_f, alonsomora_parallelization_manager=self.Parallelisation_Manager, veh_tree_build_timeout=self.alonso_mora_timeout_per_veh_tree, optimisation_cores=N_cores, optimisation_timeout=self.optimisation_timeout)
        self.PT_Module = ParallelTemperingAssignment(self, self.routing_engine, self.sim_time, self.const_bt, self.add_bt, self.vr_ctrl_f)
        
        #dynamic dicts to update database
        self.new_requests = {}  #rid -> prq (new)
        self.requests_that_changed = {} #rid -> prq (allready here but new constraints)
        self.vid_finished_VRLs = {}

        self.unassigned_requests_1 = {}

    def receive_status_update(self, vid, simulation_time, list_finished_VRL):
        """This method can be used to update plans and trigger processes whenever a simulation vehicle finished some
         VehicleRouteLegs.

        :param vid: vehicle id
        :type vid: int
        :param simulation_time: current simulation time
        :type simulation_time: float
        :param list_finished_VRL: list of VehicleRouteLeg objects
        :type list_finished_VRL: list
        """
        veh_obj = self.sim_vehicles[vid]
        if simulation_time%self.optimisation_time_step == 0:
            force_update=True
        super().receive_status_update(vid, simulation_time, list_finished_VRL, force_update=force_update)
        #track done VRLs for upating DB in optimisation-step
        try:
            self.vid_finished_VRLs[vid] += list_finished_VRL
        except:
            self.vid_finished_VRLs[vid] = list_finished_VRL
        LOG.debug(f"veh {veh_obj} | after status update: {self.veh_plans[vid]}")
        am_assigned_plan = self.AM_Module.getOptimisationSolution(vid)
        if am_assigned_plan is not None:
            am_assigned_plan.update_plan(veh_obj, simulation_time, self.routing_engine, list_finished_VRL)

    def user_request(self, rq, sim_time):
        """This method is triggered for a new incoming request. It generally adds the rq to the database. It has to
        return an offer to the user. An empty dictionary means no offer is made!

        WHEN INHERITING THIS FUNCTION AN ADDITIONAL CONTROL STRUCTURE TO CREATE OFFERS NEED TO BE IMPLEMENTED IF NEEDED
        (e.g. the functionality of creating an offer might be extended here)

        :param rq: request object containing all request information
        :type rq: RequestDesign
        :param sim_time: current simulation time
        :type sim_time: float
        :return: offer
        :rtype: dict
        """
        LOG.debug(f"Incoming request {rq.__dict__} at time {sim_time}")
        # class PlanRequest:
        #     def __init__(self, rq, routing_engine, min_wait_time=0, max_wait_time=LARGE_INT, max_detour_time_factor=None,
        #                  max_constant_detour_time=None, add_constant_detour_time=None, min_detour_time_window=None,
        #                  boarding_time=0):
        prq = PlanRequest(rq, self.routing_engine, min_wait_time = self.min_wait_time, max_wait_time = self.max_wait_time, max_detour_time_factor = self.max_dtf, 
                            max_constant_detour_time = self.max_cdt, add_constant_detour_time = self.max_cdt, boarding_time = self.const_bt)

        rid_struct = rq.get_rid_struct()
        self.rq_dict[rid_struct] = prq
        self.AM_Module.addNewRequest(rid_struct, prq)
        self.new_requests[rid_struct] = 1

        # prq2 = PlanRequest(rq, self.routing_engine, min_wait_time = self.min_wait_time, max_wait_time = self.max_wait_time, max_detour_time_factor = self.max_dtf, 
        #                     max_constant_detour_time = self.max_cdt, add_constant_detour_time = self.max_cdt, boarding_time = self.const_bt)
        # self.PT_Module.addNewRequest(prq2.get_rid_struct(), prq2)
        self.PT_Module.addNewRequest(rid_struct, prq)

        return {}

    def user_confirms_booking(self, rid, simulation_time):
        """This method is used to confirm a customer booking. This can trigger some database processes.

        WHEN INHERITING THIS FUNCTION ADDITIONAL CONTROL STRUCTURES WHICH DEFINE THE ASSIGNED PLAN MIGHT BE NEEDED

        :param rid: request id
        :type rid: int
        :param simulation_time: current simulation time
        :type simulation_time: float
        """
        LOG.debug(f"user confirms booking {rid} at {simulation_time}")
        self.AM_Module.setRequestAssigned(rid)  # TODO #
        self.PT_Module.setRequestAssigned(rid)

    def user_cancels_request(self, rid, simulation_time):
        """This method is used to confirm a customer cancellation. This can trigger some database processes.

        WHEN INHERITING THIS FUNCTION AN ADDITIONAL CONTROL STRUCTURE DEFINING WHICH VEHICLE ROUTE SHOULD BE PICKED INSTEAD NEEDS TO BE IMPLEMENTED!

        :param rid: request id
        :type rid: int
        :param simulation_time: current simulation time
        :type simulation_time: float
        """
        LOG.debug(f"user cancels request {rid} at {simulation_time}")
        del self.rq_dict[rid]
        try:
            del self.rid_to_assigned_vid[rid]
        except KeyError:
            pass
        self.AM_Module.delRequest(rid)
        self.PT_Module.delRequest(rid)

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
        self.AM_Module.setDataBaseInCaseOfBoarding(rid, vid)
        self.PT_Module.setDataBaseInCaseOfBoarding(rid, vid)

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
        self.AM_Module.setDataBaseInCaseOfAlighting(rid, vid)
        self.PT_Module.setDataBaseInCaseOfAlighting(rid, vid)
        del self.rq_dict[rid]
        try:
            del self.rid_to_assigned_vid[rid]
        except KeyError:
            pass

    def _call_time_trigger_request_batch(self, simulation_time):
        """This method can be used to perform time-triggered proccesses, e.g. the optimization of the current
        assignments of simulation vehicles of the fleet.

        WHEN INHERITING THIS FUNCTION AN ADDITIONAL CONTROL STRUCTURE TO CREATE OFFERS NEED TO BE IMPLEMENTED IF NEEDED

        :param simulation_time: current simulation time
        :type simulation_time: float
        :return: rid -> offer dictionary (empty by default)
        :rtype: dict
        """
        self.sim_time = simulation_time
        if self.sim_time % self.optimisation_time_step == 0:
            LOG.info(f"time for new optimsation at {simulation_time}")
            self.PT_Module.computeNewVehicleAssignments(self.sim_time, self.vid_finished_VRLs)
            self.AM_Module.computeNewVehicleAssignments(self.sim_time, self.vid_finished_VRLs, build_from_scratch=False)
            LOG.info(f"new assignements computed")
            LOG.info(f"OPT SOL CFVS: PT : {self.PT_Module.current_best_cfv} | AM : {self.AM_Module.current_best_cfv}")
            from src.fleetctrl.pooling.batch.AlonsoMora.AlonsoMoraAssignment import getRTVkeyFromVehPlan
            for vid in range(len(self.sim_vehicles)):
                am_plan = self.AM_Module.getOptimisationSolution(vid)
                pt_plan = self.PT_Module.getOptimisationSolution(vid)
                LOG.debug(f"assignment for vid {vid} : PT: {getRTVkeyFromVehPlan(pt_plan)} | AM: {getRTVkeyFromVehPlan(am_plan)}")
            if self.PT_Module.current_best_cfv - self.AM_Module.current_best_cfv < -0.1:
                LOG.error("AM solution worse!")
                raise EnvironmentError
            self.set_new_assignments()
            self.clearDataBases()
            self.AM_Module.clearDataBases()
            self.PT_Module.clearDataBases()
            self.set_new_assignments_algorithm2()
        rid_to_offers = {}
        if self.sim_time % self.optimisation_time_step == 0:
            new_unassigned_requests_2 = {}
            # rids to be assigned in first try
            for rid in self.unassigned_requests_1.keys():
                assigned_vid = self.rid_to_assigned_vid.get(rid, None)
                prq = self.rq_dict[rid]
                if assigned_vid is None:
                    rid_to_offers[rid] = {}
                else:
                    assigned_plan = self.veh_plans[assigned_vid]
                    pu_time, do_time = assigned_plan.pax_info.get(rid)
                    offer = {}
                    pu_offer_tuple = self._get_offered_time_interval(rid)
                    if pu_offer_tuple is not None:
                        new_earliest_pu, new_latest_pu = pu_offer_tuple
                        offer[G_OFFER_PU_INT_START] = new_earliest_pu
                        offer[G_OFFER_PU_INT_END] = new_latest_pu
                    offer[G_OFFER_WAIT] = pu_time - prq.get_rq_time()
                    offer[G_OFFER_DRIVE] = do_time - pu_time
                    offer[G_OFFER_FARE] = int(prq.init_direct_td * self.dist_fare + self.base_fare)
                    rid_to_offers[rid] = offer

            self.unassigned_requests_1 = {}
            LOG.debug("end of opt:")
            LOG.debug("offers: {}".format(rid_to_offers))
        return rid_to_offers


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
            self.AM_Module.set_assignment(veh_obj.vid, vehicle_plan, is_external_vehicle_plan=True)
            self.PT_Module.set_assignment(veh_obj.vid, vehicle_plan.copy(), is_external_vehicle_plan=True)
        else:
            self.AM_Module.set_assignment(veh_obj.vid, vehicle_plan)
            self.PT_Module.set_assignment(veh_obj.vid, vehicle_plan.copy())

    def set_new_assignments(self):
        """ this function is no longer up to date -> look in am fleet control base TODO
        """
        raise NotImplemented("Function removed because no longer up to date!")



    def clearDataBases(self):
        """ this function clears dynamic data base entries in fleet control 
        should be called after the optimsation step
        """
        self.new_requests = {}
        self.requests_that_changed = {}
        self.vid_finished_VRLs = {}