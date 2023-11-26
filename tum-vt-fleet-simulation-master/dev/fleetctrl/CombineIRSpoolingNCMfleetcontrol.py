# -------------------------------------------------------------------------------------------------------------------- #
# standard distribution imports
# -----------------------------
import logging
# import importlib
# from abc import ABC, abstractmethod

# additional module imports (> requirements)
# ------------------------------------------
# from IPython import embed
# import time

# src imports
# -----------
from src.fleetctrl.planning.PlanRequest import PlanRequest, SoftConstraintPlanRequest
from src.simulation.Offers import TravellerOffer
from dev.fleetctrl.HailingFleetcontrolBase import HailingVehiclePlan, HailingTask
from dev.fleetctrl.HailingBatchOptimizationSumObjectives import HailingBatchOptimizationSumObjectivesBase
from dev.fleetctrl.hailing.immediate.searchAll import search_all_unlocked_rv_in_tw_one_unlocked
from dev.fleetctrl.hailing.immediate.searchNN import get_nearest_vehicle
from dev.fleetctrl.hailing.batch.optSumObj import *

# -------------------------------------------------------------------------------------------------------------------- #
# global variables
# ----------------
from src.misc.globals import *
LOG = logging.getLogger(__name__)
LARGE_INT = 100000


class CombineIRSpoolingNCMfleetcontrol(HailingBatchOptimizationSumObjectivesBase):
    """Test
    """

    def __init__(self, op_id, operator_attributes, list_vehicles, routing_engine, zone_system, scenario_parameters,
                 dir_names, charging_management):
        super().__init__(op_id, operator_attributes, list_vehicles, routing_engine, zone_system, scenario_parameters,
                         dir_names, charging_management)
        self.scenario_parameters = scenario_parameters
        # self.max_wait_time = scenario_parameters.get(G_AR_MAX_WT_2, G_AR_MAX_WT)
        self.blocked_vehicles = []
        self.active_request_offers = {}  # rid -> TravellerOffers

    def user_request(self, rq, sim_time):
        """This method is triggered for a new incoming request. It generates a PlanRequest or SoftConstraintPlanRequest,
        depending on the time window constraints. It returns an offer to the user. An empty dictionary means no offer
        is made! The plan stops are NOT yet assigned to the vehicle. Instead the vehicle used to generate the offer is
        blocked until the user is eventually assigned it.

        :param rq: request object containing all request information
        :type rq: RequestDesign
        :param sim_time: current simulation time
        :type sim_time: float
        :return: offer
        :rtype: dict
        """
        if not self.considered_veh:
            self._update_veh_av_from_scratch(sim_time)
        if self.update_soft_time_windows:
            prq = SoftConstraintPlanRequest(rq, self.routing_engine, min_wait_time=self.min_wait_time,
                                            max_wait_time=self.max_wait_time, boarding_time=self.const_bt)
        else:
            prq = PlanRequest(rq, self.routing_engine, min_wait_time=self.min_wait_time,
                              max_wait_time=self.max_wait_time, boarding_time=self.const_bt)
        rid_struct = prq.get_rid_struct()
        if prq.o_pos == prq.d_pos:
            LOG.debug("automatic decline!")
            return {}
        self.rq_dict[rid_struct] = prq
        best_veh_obj, best_pu_t, best_veh_obj_old_av_pos = get_nearest_vehicle(self, prq, sim_time,
                                                                               self.blocked_vehicles)
        # make assignment and create offer
        if best_veh_obj:
            self.blocked_vehicles.append(best_veh_obj.vid)
            # create offer
            # offer = {G_OFFER_WAIT: best_pu_t - sim_time, G_OFFER_DRIVE: prq.init_direct_tt,
            #          G_OFFER_FARE: self._compute_fare(prq), 'offer_vehicle': best_veh_obj.vid}
            offer = TravellerOffer(prq.get_rid_struct(), self.op_id, best_pu_t - prq.rq_time, prq.init_direct_tt,
                                   self._compute_fare(sim_time, prq), {'offer_vehicle': best_veh_obj.vid})
            LOG.debug(f"new offer for rid {rid_struct} : {offer}")
            self.offers[rid_struct] = offer
            prq.set_service_offered(offer)
            # set time windows
            if self.update_hard_time_windows:
                self.change_prq_time_constraints(sim_time, rq.rid, min(best_pu_t + (self.time_window_length / 2),
                                                                       prq.t_pu_latest),
                                                 new_ept=max(best_pu_t - (self.time_window_length / 2),
                                                             prq.t_pu_earliest))
                LOG.debug(f"Hard time window for request {rq.rid} set to [{best_pu_t - (self.time_window_length / 2)},"
                          f"{best_pu_t + (self.time_window_length / 2)}].")
            elif self.update_soft_time_windows:
                prq.set_soft_pu_constraints(min(best_pu_t + (self.time_window_length / 2), prq.t_pu_latest),
                                            max(best_pu_t - (self.time_window_length / 2), prq.t_pu_earliest))
                LOG.debug(f"Soft time window for request {rq.rid} set to [{best_pu_t - (self.time_window_length / 2)},"
                          f"{best_pu_t + (self.time_window_length / 2)}].")
            return offer
        # no offer could be made
        # self.offers[rid_struct] = {}
        self.offers[rid_struct] = TravellerOffer(prq.get_rid(), self.op_id, None, None, None)
        # self.prq_status[rid_struct] = -1
        return {}

    def _create_user_offer(self, prq, simulation_time, assigned_vehicle_plan=None, offer_dict_without_plan=None):
        """ This method creates an offer for the request rid.

        :param prq: plan request
        :type prq: PlanRequest obj
        :param simulation_time: current simulation time
        :type simulation_time: int
        :param assigned_vehicle_plan: vehicle plan of initial solution to serve this request
        :type assigned_vehicle_plan: VehiclePlan or None
        :param offer_dict_without_plan: can be used to create an offer that is not derived from a vehicle plan
        :type offer_dict_without_plan: dict or None
        :return: offer dictionary
        :rtype: dictionary
        """
        if offer_dict_without_plan is None:
            offer_dict_without_plan = {}
        # TODO # maybe add some lines for soft constraints
        return super()._create_user_offer(prq, simulation_time, assigned_vehicle_plan=assigned_vehicle_plan,
                                          offer_dict_without_plan=offer_dict_without_plan)

    def _call_time_trigger_request_batch(self, simulation_time):
        """This method can be used to perform time-triggered processes, e.g. the optimization of the current
        assignments of simulation vehicles of the fleet.

        :param simulation_time: current simulation time
        :type simulation_time: int
        :return: {}: rid -> offer
        """
        self._update_surge_pricing(simulation_time)
        self._update_veh_av_from_scratch(simulation_time, consider_only_locked=True)
        if simulation_time % self.optimisation_time_step == 0:
            # 1a) old requests remain in existing vehicle plans that are updated in status
            # 1b) search veh-plans for new requests from last locked position/time
            list_prq_status_for_search = [G_PRQS_ACC_OFFER]
            # TODO # change to input: list of possible new assignments
            list_new_hailing_plans = search_all_unlocked_rv_in_tw_one_unlocked(self, list_prq_status_for_search,
                                                                               simulation_time, self.blocked_vehicles)
            # 2) create new data base entries
            for hailing_plan in list_new_hailing_plans:
                self._add_hailing_plan_to_db(hailing_plan)
            # 3) create possible HailingVehiclePlan for vehicles (extra loop to include old data base entries as well)
            v_ass_obj_dict = {}
            for vid, assign_dict in self.v2a.items():
                veh_obj = self.sim_vehicles[vid]
                for poss_assignment in assign_dict.keys():
                    self.compute_VehiclePlan_utility(simulation_time, veh_obj, poss_assignment)
                    try:
                        v_ass_obj_dict[vid].append(poss_assignment)
                    except KeyError:
                        v_ass_obj_dict[vid] = [poss_assignment]
            # 4) call optimization
            solver_key = self.solver
            opt_problem = return_sum_objective_solver(solver_key)
            prob_instance = opt_problem(v_ass_obj_dict, simulation_time, self)
            new_dict_vid_assignment = prob_instance.solve()
            # 5) go through old assignments -> compare with new ones
            remove_old_assignments = []
            new_assignments = list(new_dict_vid_assignment.values())
            new_assigned_vehicles = [x.vid for x in new_assignments]
            for new_assignment in new_assignments:
                old_is_also_new_assignment_is_also_new_one = True
                for new_ass_rid in new_assignment.pax_info.keys():
                    if new_ass_rid not in self.current_r_assignment.keys():
                        old_is_also_new_assignment_is_also_new_one = False
                    # elif self.current_r_assignment[new_ass_rid].vid != new_assignment.vid:
                    elif new_assignment != self.current_r_assignment[new_ass_rid]:
                        if self.current_r_assignment[new_ass_rid].vid not in new_assigned_vehicles:
                            if self.veh_plans[self.current_r_assignment[new_ass_rid].vid] != \
                                    self.current_r_assignment[new_ass_rid]:
                                LOG.info(f"At some point the current_r_assignment of {new_ass_rid} was not updated!")
                                # TODO # rare issue that makes it necessary to append the veh_plan assignment instead
                            remove_old_assignments.append(self.veh_plans[self.current_r_assignment[new_ass_rid].vid])
                        elif new_assignment not in remove_old_assignments:
                            remove_old_assignments.append(new_assignment)
                        if self.current_r_assignment[new_ass_rid] not in remove_old_assignments:
                            remove_old_assignments.append(self.current_r_assignment[new_ass_rid])
                        old_is_also_new_assignment_is_also_new_one = False
                if old_is_also_new_assignment_is_also_new_one:
                    del new_dict_vid_assignment[new_assignment.vid]
            new_assignments = list(new_dict_vid_assignment.values())
            old_assignments = []
            for assignment in remove_old_assignments:
                if assignment.vid not in [x.vid for x in new_assignments]:
                    old_assignments.append(assignment)
            old_pos = {}  # TODO # necessary in order to update considered_vehicles at the end ot the time_trigger
            for assignment in remove_old_assignments + new_assignments:
                try:
                    pos = self.veh_plans[assignment.vid].list_plan_stops[-1].get_pos()
                except IndexError:
                    pos = self.sim_vehicles[assignment.vid].pos
                old_pos[assignment.vid] = pos
            self._change_assignments(simulation_time, old_assignments, new_assignments)
            # 6) reduce data bases if not updated
            if not self.update_db:
                self._reset_db()
                for assigned_plan in self.veh_plans.values():
                    self._add_hailing_plan_to_db(assigned_plan)
            # for assignment in remove_old_assignments + new_assignments:
            for changed_assignments in old_assignments + new_assignments:
                assignment = self.veh_plans[changed_assignments.vid]
                for rid in assignment.return_all_rids():
                    self.current_r_assignment[rid] = assignment
                self._update_single_veh_av(simulation_time, self.sim_vehicles[assignment.vid], old_pos[assignment.vid],
                                           assignment, False)
        if self.repo is not None and (simulation_time % self.repo_time_step) == 0:
            self.repo.determine_and_create_repositioning_plans(simulation_time, lock=False)
        for rid in self.rq_dict:
            prq = self.rq_dict[rid]
            if prq.status == G_PRQS_ACC_OFFER:
                if prq.expected_pickup_time - simulation_time < self.early_lock_time:
                    self._create_user_offer(prq, simulation_time, self.current_r_assignment[rid], {"locked": True})
                    prq.status = G_PRQS_LOCKED
        return self.offers

    def user_confirms_booking(self, rid, simulation_time):
        """This method is used to confirm a customer booking. Depending on the current prq_status this triggers
        subsequent methods.

        :param rid: request id
        :type rid: int
        :param simulation_time: current simulation time
        :type simulation_time: float
        """
        prq = self.rq_dict[rid]
        if prq.status == G_PRQS_LOCKED:
            # user assigned and confirmed
            vid = self.current_r_assignment[rid].vid
            prq.lock_request()
            self.veh_plans[vid].lock_rid(rid, self.sim_vehicles[vid])
        elif prq.status == G_PRQS_INIT_OFFER:
            # user confirmed initial offer
            prq.set_service_accepted()
            self.assign_user(rid, self.sim_vehicles[self.offers[rid].additional_offer_parameters['offer_vehicle']],
                             simulation_time)

    def user_cancels_request(self, rid, simulation_time):
        """This method is used to confirm a customer cancellation. This can trigger some database processes.

        :param rid: request id
        :type rid: int
        :param simulation_time: current simulation time
        :type simulation_time: int
        :return assignment: adjusted assignment without the cancelled request
        """
        if self.update_db:
            # remove all plans; only change currently assigned plan
            for vid, veh_plan in self.r2a.get(rid, {}).keys():
                if veh_plan == self.veh_plans[vid]:
                    veh_obj = self.sim_vehicles[vid]
                    veh_plan.remove_prq(rid, veh_obj, simulation_time, self.routing_engine, self.const_bt)
                else:
                    self._remove_hailing_plan_from_db(veh_plan)
            self._delete_request(rid)
        else:
            # only change current assignment with rid (r2a and v2a are emptied after optimization)
            assignment = self.current_r_assignment.get(rid)
            prq = self.rq_dict[rid]
            if assignment is not None:
                vid = assignment.vid
                veh_obj = self.sim_vehicles[vid]
                assignment.remove_prq(rid, veh_obj, simulation_time, self.routing_engine, self.const_bt)
                self.assign_vehicle_plan(veh_obj, assignment, simulation_time, force_assign=True)
                for rid_in_plan in assignment.return_all_rids():
                    self.current_r_assignment[rid_in_plan] = assignment
            else:
                self._delete_request(rid)
            if prq.status == G_PRQS_INIT_OFFER:
                self.blocked_vehicles.remove(prq.offer.additional_offer_parameters['offer_vehicle'])
            return assignment

    def assign_user(self, rid, veh_obj, simulation_time):
        """
        This method is used to assign a customer to a vehicle after the customer eventually decided to accept the offer
        generated in user_request. It generates a new HailingTask and HailingVehiclePlan, changes the assignment of the
        vehicle and updates all relevant databases.

        :param rid: request ID
        :param veh_obj: vehicle object that was blocked in user_request
        :param simulation_time: current simulation time
        """
        if not self.considered_veh:
            self._update_veh_av_from_scratch(simulation_time)
        try:
            pos = self.veh_plans[veh_obj.vid].list_plan_stops[-1].get_pos()
        except IndexError:
            pos = veh_obj.pos
        # prq = self.rq_dict[rid]
        # create new assignment
        self.blocked_vehicles.remove(veh_obj.vid)
        old_veh_plan = self.veh_plans[veh_obj.vid]
        # TODO # this should not be necessary anymore
        # self._update_single_veh_av(simulation_time, veh_obj, pos, old_veh_plan, False)
        old_tasks = old_veh_plan.get_ordered_task_list()
        new_hailing_task = HailingTask(veh_obj.vid, {G_FCTRL_PRQ: self.rq_dict[rid], G_VR_LOCKED: False})
        new_tasks = old_tasks + [new_hailing_task]
        new_veh_plan = HailingVehiclePlan(veh_obj, simulation_time, self.routing_engine, self.const_bt, new_tasks)
        # make assignment and adapt vehicle availability
        self._change_assignments(simulation_time, [], [new_veh_plan])
        self._remove_hailing_plan_from_db(old_veh_plan, True)
        self._add_hailing_plan_to_db(new_veh_plan)
        for rid_in_plan in new_veh_plan.return_all_rids():
            self.current_r_assignment[rid_in_plan] = new_veh_plan
        # TODO # this should not be necessary anymore
        # self._update_single_veh_av(simulation_time, veh_obj, pos, new_veh_plan, False)

    def receive_status_update(self, vid, simulation_time, list_finished_VRL, force=False):
        """This method checks if an unlocked request in vid's veh_plan needs to be locked. It also updates plans
          and triggers processes whenever (a) vid finished some VehicleRouteLegs, (b) simulation_time is an optimization
          time step or (c) the update is forced for any other reason.
        :param vid: vehicle id
        :type vid: int
        :param simulation_time: current simulation time
        :type simulation_time: float
        :param list_finished_VRL: list of VehicleRouteLeg objects
        :type list_finished_VRL: list
        :param force: should force the updating of plans after optimization
        :type force: bool
        """
        '''
        for rid in self.veh_plans[vid].return_non_locked_rids():
            prq = self.rq_dict[rid]
            if prq.status == G_PRQS_ACC_OFFER:
                if prq.expected_pickup_time - simulation_time < self.early_lock_time:
                    if self.offers[rid]:
                        self.sim_vehicles[vid].rq_db[rid].receive_offer(self.op_id, self.offers[rid], simulation_time,
                                                                        self.scenario_parameters)
                        prq.status = G_PRQS_LOCKED
                    else:
                        LOG.info(f'no second offer for {rid}!')
                        self.user_cancels_request(rid, simulation_time)
                        self.sim_vehicles[vid].rq_db[rid].record_user(rid)
                        del self.sim_vehicles[vid].rq_db[rid]
        '''
        if list_finished_VRL or simulation_time % self.optimisation_time_step == 0 or force:
            self.veh_plans[vid].update_plan(self.sim_vehicles[vid], simulation_time, self.routing_engine,
                                            list_finished_VRL)

    def acknowledge_alighting(self, rid, vid, simulation_time):
        """This method can trigger some database processes whenever a passenger is finishing to alight a vehicle.
        It returns the assignment sans the alighted customer

        :param rid: request id
        :type rid: int
        :param vid: vehicle id
        :type vid: int
        :param simulation_time: current simulation time
        :type simulation_time: float
        :return assignment: adjusted assignment without the alighted request
        """
        self._delete_request(rid)
        assignment = self.current_r_assignment.get(rid)
        if assignment is not None:
            vid = assignment.vid
            veh_obj = self.sim_vehicles[vid]
            assignment.remove_prq(rid, veh_obj, simulation_time, self.routing_engine, self.const_bt)
            del self.current_r_assignment[rid]
            for rid_in_plan in assignment.return_all_rids():
                self.current_r_assignment[rid_in_plan] = assignment
            self._add_hailing_plan_to_db(assignment)
        return assignment

    def _lock_vid_rid_pickup(self, sim_time, vid, rid):
        super()._lock_vid_rid_pickup(sim_time, vid, rid)
