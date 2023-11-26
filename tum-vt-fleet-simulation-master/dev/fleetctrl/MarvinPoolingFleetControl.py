import logging
# import os
# from src.fleetctrl.pooling.GeneralPoolingFunctions import get_assigned_rids_from_vehplan
from src.fleetctrl.planning.PlanRequest import PlanRequest, SoftConstraintPlanRequest
from src.fleetctrl.planning.VehiclePlan import VehiclePlan
# import numpy as np
# import pandas as pd
from IPython import embed
from src.fleetctrl.RidePoolingBatchOptimizationFleetControlBase import RidePoolingBatchOptimizationFleetControlBase
from dev.fleetctrl.AlonsoMoraFleetcontrolBase import AlonsoMoraFleetControlBase
from src.fleetctrl.pooling.immediate.insertion import simple_remove, single_insertion
from src.misc.globals import *
from src.simulation.Offers import TravellerOffer

LOG = logging.getLogger(__name__)
LARGE_INT = 100000


# TODO # check replacement AlonsoMoraFleetControlBase -> RidePoolingBatchOptimizationFleetControlBase
class MarvinPoolingFleetControl(AlonsoMoraFleetControlBase):
    def __init__(self, op_id, operator_attributes, list_vehicles, routing_engine, zone_system, scenario_parameters,
                 dir_names, charging_management=None):
        """Fleet control class for ...
            -> there can never be 2 requests at the same time waiting for an offer! 
        reoptimisation of solution after certain time interval

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
        :param dir_names: directories for output and input
        :type dir_names: dict
        """
        super().__init__(op_id, operator_attributes, list_vehicles, routing_engine, zone_system, scenario_parameters,
                         dir_names, charging_management=charging_management)
        self.current_vid_plan_tuple = None
        # (vid, veh_plan) veh_plan is current solution for offer which will be destroyed or accepted
        # once a request decides (only one can exist simultaneously!)
        self.operator_attributes = operator_attributes

    def add_init(self, operator_attributes, scenario_parameters):
        super().add_init(operator_attributes, scenario_parameters)

    def user_request(self, rq, sim_time):
        """This method is triggered for a new incoming request. It generally adds the rq to the database. It has to
        return an offer to the user. An empty dictionary means no offer is made!

        This method additionally evaluates the willingness of the operator to serve the request and sets the decision in
        the offer with attribute G_OFFER_WILLING_FLAG to provide the information to the fleet simulation.

        :param rq: request object containing all request information
        :type rq: RequestDesign
        :param sim_time: current simulation time
        :type sim_time: int
        :return: offer
        :rtype: dict
        """
        if self.current_vid_plan_tuple is not None:
            LOG.error(f"new user_request before old request is resolved! {self.current_vid_plan_tuple}")
            raise AssertionError
        LOG.debug(f"Incoming request {rq.__dict__} at time {sim_time}")
        self.sim_time = sim_time
        if self.update_soft_time_windows:
            prq = SoftConstraintPlanRequest(rq, self.routing_engine, min_wait_time=self.min_wait_time,
                                            max_wait_time=self.max_wait_time, boarding_time=self.const_bt,
                                            max_detour_time_factor=self.max_dtf,
                                            max_constant_detour_time=self.max_cdt,
                                            add_constant_detour_time=self.add_cdt, min_detour_time_window=self.min_dtw,
                                            ept_soft=self.min_wait_time, lpt_soft=self.max_wait_time)
        else:
            prq = PlanRequest(rq, self.routing_engine, min_wait_time=self.min_wait_time,
                              max_wait_time=self.max_wait_time, boarding_time=self.const_bt,
                              max_detour_time_factor=self.max_dtf, max_constant_detour_time=self.max_cdt,
                              add_constant_detour_time=self.add_cdt, min_detour_time_window=self.min_dtw)
        rid_struct = rq.get_rid_struct()
        self.rq_dict[rid_struct] = prq
        self.AM_Module.addNewRequest(rid_struct, prq)
        self.new_requests[rid_struct] = 1
        if prq.o_pos == prq.d_pos:
            LOG.debug("automatic decline!")
            return {}
        LOG.debug("new user request {}".format(rid_struct))
        assigned_vid, assigned_plan, change_in_objective_value = single_insertion(self.sim_vehicles, self.veh_plans,
                                                                                  prq, self.vr_ctrl_f,
                                                                                  self.routing_engine, self.rq_dict,
                                                                                  sim_time, self.const_bt, self.add_bt)
        if assigned_vid is not None:    # fleet operator can serve
            offer = self._create_user_offer(prq, sim_time, assigned_vehicle_plan=assigned_plan)
            LOG.debug(f"new offer for rid {rid_struct} : {offer}")
            if self.update_hard_time_windows:
                pu_time, _ = assigned_plan.pax_info.get(rid_struct)
                prq = self.rq_dict[rid_struct]
                self.change_prq_time_constraints(sim_time, rid_struct,
                                                 min(pu_time + (self.time_window_length / 2), prq.t_pu_latest),
                                                 max(pu_time - (self.time_window_length / 2), prq.t_pu_earliest))
            self.assign_vehicle_plan(self.sim_vehicles[assigned_vid], assigned_plan, sim_time)
            # use method self.change_prq_time_constraints to adopt pickup constraints if needed
            self.AM_Module.setRequestAssigned(rid_struct)
        else:
            LOG.debug(f"no offer for rid {rid_struct}")
            offer = self._create_user_offer(prq, sim_time, assigned_vehicle_plan=None)

    def user_confirms_booking(self, rid, simulation_time):
        """This method is used to confirm a customer booking. This can trigger some database processes.

        :param rid: request id
        :type rid: int
        :param simulation_time: current simulation time
        :type simulation_time: int
        """
        offer = self.active_request_offers[rid]
        super().user_confirms_booking(rid, simulation_time)
        self.active_request_offers[rid] = offer

    def user_cancels_request(self, rid, simulation_time):
        """This method is used to confirm a customer cancellation. This can trigger some database processes.

        :param rid: request id
        :type rid: int
        :param simulation_time: current simulation time
        :type simulation_time: int
        """
        # TODO # i added the stuff below because late rejections havent been treated correctly (havent been removed from vehplans)
        assigned_vid = self.rid_to_assigned_vid.get(rid, None)
        if assigned_vid is not None:
            veh_obj = self.sim_vehicles[assigned_vid]
            assigned_plan = self.AM_Module.get_current_assignment(assigned_vid)
            new_best_plan = self.AM_Module.getVehiclePlanWithoutRid(veh_obj, assigned_plan, rid, simulation_time)
            if new_best_plan is not None:
                self.assign_vehicle_plan(assigned_vid, new_best_plan, simulation_time, force_assign=True)
            else:
                assigned_plan = VehiclePlan(veh_obj, self.sim_time, self.routing_engine, [])
                self.assign_vehicle_plan(assigned_vid, assigned_plan, simulation_time, force_assign=True)
        super().user_cancels_request(rid, simulation_time)

    # def _call_time_trigger_request_batch(self, simulation_time):
    #     """ triggers optimisation every "op_reoptimisation_timestep"
    #     something else needs to be done here?
    #     """
    #     # TODO # all this prq_status stuff should happen at some point, probably here?
    #     # TODO # updating offers?
    #     super()._call_time_trigger_request_batch(simulation_time)
    #     if self.repo is not None and (simulation_time % self.repo_time_step) == 0:
    #         self.repo.determine_and_create_repositioning_plans(simulation_time, lock=False)
    
    def time_trigger(self, simulation_time):
        """ triggers optimisation every "op_reoptimisation_timestep"
        something else needs to be done here?
        """
        # TODO # all this prq_status stuff should happen at some point, probably here?
        # TODO # updating offers?
        super().time_trigger(simulation_time)
        if self.repo is not None and (simulation_time % self.repo_time_step) == 0:
            self.repo.determine_and_create_repositioning_plans(simulation_time, lock=False)

    def lock_current_vehicle_plan(self, vid):
        super().lock_current_vehicle_plan(vid)
        # TODO # if we want to include early locks, we need to add stuff here

    def change_prq_time_constraints(self, sim_time, rid, new_lpt, new_ept=None):
        """this function registers if time constraints of a requests is changed during the simulation
        :param sim_time: simulation time
        :param rid: plan request id
        :param new_lpt: new latest pickup time constraint for request rid
        :param new_ept: new earliest pickup time constraint for request rid (not necessary)
        """
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
        try:
            self.AM_Module.register_change_in_time_constraints(rid, prq, assigned_vid=ass_vid,
                                                               exceeds_former_time_windows=exceed_tw)
        except KeyError:  # TODO # should only happen if pickups are locked before first global optimization
            # TODO # nothing to do in this case because the pickup is locked in next step anyway (maybe there is a
            #  smoother way to take that into account, though)
            pass

    def _create_user_offer(self, prq, simulation_time, assigned_vehicle_plan=None, offer_dict_without_plan=None):
        """ creating the offer for a requests
        willing_flag will be set afterwards

        :param prq: plan request
        :type prq: PlanRequest obj
        :param simulation_time: current simulation time
        :type simulation_time: int
        :param assigned_vehicle_plan: vehicle plan of initial solution to serve this request
        :type assigned_vehicle_plan: VehiclePlan or None
        :param offer_dict_without_plan: can be used to create an offer that is not derived from a vehicle plan
        :type offer_dict_without_plan: dict or None
        :return: offer entity
        :rtype: TravellerOffer
        """
        if offer_dict_without_plan is None:
            offer_dict_without_plan = {}
        if assigned_vehicle_plan is not None:
            pu_time, do_time = assigned_vehicle_plan.pax_info.get(prq.get_rid_struct())
            offer = TravellerOffer(prq.get_rid_struct(), self.op_id, pu_time - prq.rq_time, do_time - pu_time,
                                   int(prq.init_direct_td * self.dist_fare + self.base_fare),
                                   additional_parameters=offer_dict_without_plan)
            prq.set_service_offered(offer)  # has to be called
            self.active_request_offers[prq.get_rid_struct()] = offer
            if self.update_soft_time_windows:
                prq.set_soft_pu_constraints(min(pu_time + (self.time_window_length / 2), prq.t_pu_latest),
                                            max(pu_time - (self.time_window_length / 2), prq.t_pu_earliest))
        elif offer_dict_without_plan:
            raise NotImplementedError
        else:
            offer = TravellerOffer(prq.get_rid(), self.op_id, None, None, None)
            self.active_request_offers[prq.get_rid_struct()] = offer
        return offer

    def acknowledge_boarding(self, rid, vid, simulation_time):
        super().acknowledge_boarding(rid, vid, simulation_time)
        prq = self.rq_dict[rid]
        prq.lock_request()
