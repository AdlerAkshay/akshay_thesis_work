# -------------------------------------------------------------------------------------------------------------------- #
# standard distribution imports
# -----------------------------
import logging
import time

# additional module imports (> requirements)
# ------------------------------------------

# src imports
# -----------
from src.fleetctrl.planning.PlanRequest import PlanRequest, SoftConstraintPlanRequest
from dev.fleetctrl.HailingFleetcontrolBase import HailingFleetControlBase, HailingVehiclePlan, HailingTask
from dev.fleetctrl.hailing.immediate.searchAll import search_all_unlocked_rv_in_tw_one_unlocked, search_rv_for_new_prq
from dev.fleetctrl.hailing.batch.optSumObj import *

# -------------------------------------------------------------------------------------------------------------------- #
# global variables
# ----------------
from src.misc.globals import *
LOG = logging.getLogger(__name__)
LARGE_INT = 100000
DUMMY_OFFER = False  # TODO # think about alternative


class HailingBatchOptimizationSumObjectivesBase(HailingFleetControlBase):
    """This class uses an assignment problem for which the assignment objective is the sum of single assignments.
    It is the base class for all IRS study related fleet control methods.
    """
    def __init__(self, op_id, operator_attributes, list_vehicles, routing_engine, zone_system, scenario_parameters,
                 dir_names, op_charge_depot_infra=None,list_pub_charging_infra= []):
        self.current_r_assignment = {}  # rid -> current_assignment
        super().__init__(op_id, operator_attributes, list_vehicles, routing_engine, zone_system, scenario_parameters,
                         dir_names, op_charge_depot_infra, list_pub_charging_infra)
        self.solver = operator_attributes.get(G_RA_SOLVER, "Gurobi")
        self.optimisation_time_step = operator_attributes[G_RA_REOPT_TS]
        # TODO # think about parameters to set following attributes
        self.update_db = False
        self.wait_for_first_confirmation = False
        self.operator_attributes = operator_attributes
        self._init_dynamic_fleetcontrol_output_key(G_FCTRL_CT_RQU)
        self._init_dynamic_fleetcontrol_output_key(G_FCTRL_CT_RQB)

    def inform_network_travel_time_update(self, simulation_time):
        """ this method can be used to inform the operator that new travel times are available
        at simulation time
        :param simulation_time: time the new travel times are available
        """
        pass

    def user_request(self, rq, simulation_time):
        """This method is triggered for a new incoming request. It generally generates a PlanRequest from the rq and
        adds it to the database. It has to return an offer to the user. An empty dictionary means no offer is made!

        :param rq: request object containing all request information
        :type rq: RequestDesign
        :param simulation_time: current simulation time
        :type simulation_time: int
        :return: offer
        :rtype: dict
        """
        t0 = time.perf_counter()
        if self.update_soft_time_windows:
            prq = SoftConstraintPlanRequest(rq, self.routing_engine, min_wait_time=self.min_wait_time,
                                            max_wait_time=self.max_wait_time, boarding_time=self.const_bt)
            prq.set_soft_pu_constraints(prq.t_pu_latest, prq.t_pu_earliest)
        else:
            prq = PlanRequest(rq, self.routing_engine, min_wait_time=self.min_wait_time,
                              max_wait_time=self.max_wait_time, boarding_time=self.const_bt)
        if prq.o_pos == prq.d_pos:
            LOG.debug("automatic decline!")
            self._create_rejection(simulation_time)
        rid_struct = prq.get_rid_struct()
        self.rq_dict[rid_struct] = prq
        self.r2a[rid_struct] = {}
        #
        o_pos, t_pu_earliest, t_pu_latest = prq.get_o_stop_info()
        if t_pu_earliest - simulation_time > self.opt_horizon:
            prq.set_reservation_flag(True)
            self.reserved_base_rids[rid_struct] = t_pu_earliest
        # record cpu time
        dt = round(time.perf_counter() - t0, 5)
        old_dt = self._get_current_dynamic_fleetcontrol_value(simulation_time, G_FCTRL_CT_RQU)
        if old_dt is None:
            new_dt = dt
        else:
            new_dt = old_dt + dt
        output_dict = {G_FCTRL_CT_RQU: new_dt}
        self._add_to_dynamic_fleetcontrol_output(simulation_time, output_dict)

    def user_confirms_booking(self, rid, simulation_time):
        """This method is used to confirm a customer booking. This can trigger some database processes.
        -> customers can accept multiple times

        :param rid: request id
        :type rid: int
        :param simulation_time: current simulation time
        :type simulation_time: float
        """
        vid = self.rid_to_assigned_vid.get(rid)
        prq = self.rq_dict.get(rid)
        if vid is not None and prq is not None and prq.get_reservation_flag():
            try:
                self.vid_with_reserved_rids[vid].append(rid)
            except KeyError:
                self.vid_with_reserved_rids[vid] = [rid]
        if prq.status == G_PRQS_INIT_OFFER:
            # user confirmed initial offer
            prq.set_service_accepted()
            pu_time = simulation_time + prq.offer[G_OFFER_WAIT]
            # set time windows
            if self.update_soft_time_windows:
                prq.set_soft_pu_constraints(min(pu_time + (self.time_window_length / 2), prq.t_pu_latest),
                                            max(pu_time - (self.time_window_length / 2), prq.t_pu_earliest))
                LOG.debug(f"Soft time window for request {prq.rid} set to [{pu_time - (self.time_window_length / 2)},"
                          f"{pu_time + (self.time_window_length / 2)}].")
            elif self.update_hard_time_windows:
                self.change_prq_time_constraints(simulation_time, rid, min(pu_time + (self.time_window_length / 2),
                                                                           prq.t_pu_latest),
                                                 new_ept=max(pu_time - (self.time_window_length / 2),
                                                             prq.t_pu_earliest))
                LOG.debug(f"Hard time window for request {prq.rid} set to [{pu_time - (self.time_window_length / 2)},"
                          f"{pu_time + (self.time_window_length / 2)}].")

    def user_cancels_request(self, rid, simulation_time):
        """This method is used to confirm a customer cancellation. This can trigger some database processes.

        :param rid: request id
        :type rid: int
        :param simulation_time: current simulation time
        :type simulation_time: int
        """
        vid = None
        assignment = self.current_r_assignment.get(rid)
        if assignment is not None:
            vid = assignment.vid
        prq = self.rq_dict.get(rid)
        if vid is not None and prq is not None and prq.get_reservation_flag():
            list_reserved_rids = self.vid_with_reserved_rids.get(vid, [])
            if rid in list_reserved_rids:
                list_reserved_rids.remove(rid)
                if list_reserved_rids:
                    self.vid_with_reserved_rids[vid] = list_reserved_rids
                else:
                    del self.vid_with_reserved_rids[vid]
        try:
            del self.reserved_base_rids[rid]
        except KeyError:
            pass
        if self.update_db:
            # remove all plans; only change currently assigned plan
            for vid, veh_plan in self.r2a.get(rid, {}).keys():
                if veh_plan == self.veh_plans[vid]:
                    veh_obj = self.sim_vehicles[vid]
                    veh_plan.remove_prq(rid, veh_obj, simulation_time, self.routing_engine, self.const_bt)
                    self.assign_vehicle_plan(veh_obj, veh_plan, simulation_time, force_assign=True)
                else:
                    self._remove_hailing_plan_from_db(veh_plan)
            self._delete_request(rid)
        else:
            # only change current assignment with rid (r2a and v2a are emptied after optimization)
            self._delete_request(rid)
            assignment = self.current_r_assignment.get(rid)
            if assignment is not None:
                vid = assignment.vid
                veh_obj = self.sim_vehicles[vid]
                assignment.remove_prq(rid, veh_obj, simulation_time, self.routing_engine, self.const_bt)
                self.assign_vehicle_plan(veh_obj, assignment, simulation_time, force_assign=True)
                del self.current_r_assignment[rid]
                self._add_hailing_plan_to_db(assignment)


    def _lock_vid_rid_pickup(self, sim_time, vid, rid):
        """This method constrains the pick-up of a rid. In the pooling case, the pick-up time is constrained to a very
        short time window. In the hailing case, the Task to serve rid is locked for the vehicle.

        WHEN OVERWRITING: MIND DATABASE UPDATES!

        :param sim_time: current simulation time
        :param vid: vehicle id
        :param rid: PlanRequest id
        :return: None
        """
        if self.update_db:
            # only keep vehicle plans with rid in vid and lock rid in these
            for veh_plan in self.r2a.get(rid, {}).keys():
                if veh_plan.vid == vid:
                    veh_plan.lock_rid(rid, self.sim_vehicles[vid])
                else:
                    self._remove_hailing_plan_from_db(veh_plan)
        else:
            self.veh_plans[vid].lock_rid(rid, self.sim_vehicles[vid])

    def acknowledge_boarding(self, rid, vid, simulation_time):
        """This method can trigger some database processes whenever a passenger is starting to board a vehicle.

        :param rid: request id
        :type rid: int
        :param vid: vehicle id
        :type vid: int
        :param simulation_time: current simulation time
        :type simulation_time: float
        """
        if self.update_db:
            # only keep vehicle plans with rid in vid and lock rid in these
            for veh_plan in self.r2a.get(rid, {}).keys():
                if veh_plan.vid == vid:
                    veh_plan.lock_rid(rid, self.sim_vehicles[vid])
                else:
                    self._remove_hailing_plan_from_db(veh_plan)
        else:
            self.veh_plans[vid].lock_rid(rid, self.sim_vehicles[vid])
        prq = self.rq_dict[rid]
        prq.set_pickup(vid, simulation_time)

    def acknowledge_alighting(self, rid, vid, simulation_time):
        """This method can trigger some database processes whenever a passenger is finishing to alight a vehicle.

        :param rid: request id
        :type rid: int
        :param vid: vehicle id
        :type vid: int
        :param simulation_time: current simulation time
        :type simulation_time: float
        """
        self._delete_request(rid)
        assignment = self.current_r_assignment.get(rid)
        if assignment is not None:
            vid = assignment.vid
            veh_obj = self.sim_vehicles[vid]
            assignment.remove_prq(rid, veh_obj, simulation_time, self.routing_engine, self.const_bt)
            del self.current_r_assignment[rid]
            self._add_hailing_plan_to_db(assignment)

    def _call_time_trigger_request_batch(self, simulation_time):
        """This method can be used to perform time-triggered processes, e.g. the optimization of the current
        assignments of simulation vehicles of the fleet.

        :param simulation_time: current simulation time
        :type simulation_time: int
        :return: {}: rid -> offer
        """
        t0 = time.perf_counter()
        super()._call_time_trigger_request_batch(simulation_time)
        if simulation_time % self.optimisation_time_step == 0:
            self._update_veh_av_from_scratch(simulation_time, consider_only_locked=True)
            # 1a) old requests remain in existing vehicle plans that are updated in status
            # 1b) search veh-plans for new requests from last locked position/time
            if self.update_db:
                list_prq_status_for_search = [G_PRQS_ACC_OFFER, G_PRQS_NO_OFFER]
            else:
                list_prq_status_for_search = [G_PRQS_ACC_OFFER, G_PRQS_LOCKED, G_PRQS_NO_OFFER]
            # TODO # only assign confirmed requests?
            if not self.wait_for_first_confirmation:
                list_prq_status_for_search.append(G_PRQS_INIT_OFFER)
            # TODO # change to input: list of possible new assignments
            list_new_hailing_plans = search_all_unlocked_rv_in_tw_one_unlocked(self, list_prq_status_for_search,
                                                                               simulation_time)
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
            # TODO # add cases depending on solver and objective function
            solver_key = self.solver
            # problem_class = importlib.import_module("src.fleetctrl.hailing.batch.optSumObj")
            # opt_problem = problem_class.return_sum_objective_solver(solver_key)
            opt_problem = return_sum_objective_solver(solver_key)
            #
            prob_instance = opt_problem(v_ass_obj_dict, simulation_time, self)
            new_dict_vid_assignment = prob_instance.solve()
            # 5) go through old assignments -> compare with new ones
            remove_old_assignments = []
            new_assignments = []
            # old MARVIN
            # new_assignments = list(new_dict_vid_assignment.values())
            # for new_assignment in new_assignments:
            #     old_is_also_new_assignment_is_also_new_one = True
            #     for new_ass_rid in new_assignment.pax_info.keys():
            #         if new_ass_rid in self.current_r_assignment.keys() and self.current_r_assignment[new_ass_rid].vid != new_assignment.vid:
            #             remove_old_assignments.append(self.current_r_assignment[new_ass_rid])
            #             old_is_also_new_assignment_is_also_new_one = False
            #     if old_is_also_new_assignment_is_also_new_one:
            #         del new_dict_vid_assignment[new_assignment.vid]
            # new_assignments = list(new_dict_vid_assignment.values())
            # TODO # make comparison on task level
            all_new_rid = [rid for x in new_dict_vid_assignment.values() for rid in x.pax_info.keys()]
            for vid in range(self.nr_vehicles):
                if new_dict_vid_assignment.get(vid) is None:
                    if self.veh_plans[vid].return_non_locked_tasks():
                        remove_old_assignments.append(self.veh_plans[vid])
                        non_locked_rid = set(self.veh_plans[vid].return_non_locked_rids())
                        non_assigned_rid = non_locked_rid.difference(all_new_rid)
                        assert len(non_assigned_rid) == 0, f"removing rids {non_assigned_rid} from {vid} without " \
                                                           f"assigning them to new vehicle"
                else:
                    if new_dict_vid_assignment[vid] != self.veh_plans[vid]:
                        new_assignments.append(new_dict_vid_assignment[vid])
            self._change_assignments(simulation_time, remove_old_assignments, new_assignments)
            # 6) reduce data bases if not updated
            if not self.update_db:
                self._reset_db()
                for assigned_plan in self.veh_plans.values():
                    self._add_hailing_plan_to_db(assigned_plan)
            # 7) derive and return offers
            self.current_r_assignment = {}
            for assignment in self.veh_plans.values():
                for rid in assignment.return_all_rids():
                    prq = self.rq_dict[rid]
                    self.current_r_assignment[rid] = assignment
                    # create offers and update prq.offer
                    self._create_user_offer(prq, simulation_time, assignment)
            dt = round(time.perf_counter() - t0, 5)
            output_dict = {G_FCTRL_CT_RQB: dt}
            self._add_to_dynamic_fleetcontrol_output(simulation_time, output_dict)


class HailingBatchOptimizationSumObjectivesIRSAssignment(HailingBatchOptimizationSumObjectivesBase):
    """This class uses an assignment problem for which the assignment objective is the sum of single assignments.
    Customers receive a response immediately after their request based on an IRS and are subject to reassignments
    via global optimization. Both soft and hard time windows can be applied for global optimization.
    """

    def user_cancels_request(self, rid, simulation_time):
        """This method is used to confirm a customer cancellation. This can trigger some database processes.

        :param rid: request id
        :type rid: int
        :param simulation_time: current simulation time
        :type simulation_time: int
        """
        new_assignment = super().user_cancels_request(rid, simulation_time)
        if new_assignment:
            self._change_assignments(simulation_time, [], [new_assignment])

    def user_request(self, rq, sim_time):
        """This method is triggered for a new incoming request. It generally generates a PlanRequest from the rq and
        adds it to the database. It has to return an offer to the user. An empty dictionary means no offer is made!
        The plan stops are already assigned and considered in the vehicle availability for future requests.

        :param rq: request object containing all request information
        :type rq: RequestDesign
        :param sim_time: current simulation time
        :type sim_time: int
        :return: offer
        :rtype: dict
        """
        t0 = time.perf_counter()
        super().user_request(rq, sim_time)
        rid_struct = rq.get_rid_struct()
        prq = self.rq_dict[rid_struct]
        #
        list_new_hailing_plans = search_rv_for_new_prq(self, prq, sim_time)
        if list_new_hailing_plans:
            new_veh_plan = list_new_hailing_plans[0]
            # make assignment and adapt vehicle availability
            vid = new_veh_plan.vid
            veh_obj = self.sim_vehicles[vid]
            old_veh_plan = self.veh_plans[vid]
            old_av_pos = self._get_vid_av_pos(sim_time, vid)
            self._change_assignments(sim_time, [old_veh_plan], [new_veh_plan])
            self._remove_hailing_plan_from_db(old_veh_plan, True)
            self._add_hailing_plan_to_db(new_veh_plan)
            for rids_in_plan in new_veh_plan.return_all_rids():
                self.current_r_assignment[rids_in_plan] = new_veh_plan
            # create offer
            self._create_user_offer(prq, sim_time, new_veh_plan)
            # update vehicle availability
            self._update_single_veh_av(sim_time, veh_obj, old_av_pos)
        # record cpu time
        dt = round(time.perf_counter() - t0, 5)
        old_dt = self._get_current_dynamic_fleetcontrol_value(sim_time, G_FCTRL_CT_RQU)
        if old_dt is None:
            new_dt = dt
        else:
            new_dt = old_dt + dt
        output_dict = {G_FCTRL_CT_RQU: new_dt}
        self._add_to_dynamic_fleetcontrol_output(sim_time, output_dict)


class HailingBatchOptimizationSumObjectivesIRSInformation(HailingBatchOptimizationSumObjectivesIRSAssignment):
    """This class uses an assignment problem for which the assignment objective is the sum of single assignments.
    Customers will receive a response after the first batch optimization after their respective request.
    """

    def user_request(self, rq, sim_time):
        """This method is triggered for a new incoming request. It generally generates a PlanRequest from the rq and
        adds it to the database. It has to return an offer to the user. An empty dictionary means no offer is made!
        The plan stops are already assigned and considered in the vehicle availability for future requests.

        :param rq: request object containing all request information
        :type rq: RequestDesign
        :param sim_time: current simulation time
        :type sim_time: int
        :return: offer
        :rtype: dict
        """
        t0 = time.perf_counter()
        super().user_request(rq, sim_time)
        rid_struct = rq.get_rid_struct()
        prq = self.rq_dict[rid_struct]
        #
        if self.considered_veh_time != sim_time:
            self._update_veh_av_from_scratch(sim_time)
        #
        list_new_hailing_plans = search_rv_for_new_prq(self, prq, sim_time)
        if list_new_hailing_plans:
            new_veh_plan = list_new_hailing_plans[0]
            # create offer
            self._create_user_offer(prq, sim_time, new_veh_plan)
        # record cpu time
        dt = round(time.perf_counter() - t0, 5)
        old_dt = self._get_current_dynamic_fleetcontrol_value(sim_time, G_FCTRL_CT_RQU)
        if old_dt is None:
            new_dt = dt
        else:
            new_dt = old_dt + dt
        output_dict = {G_FCTRL_CT_RQU: new_dt}
        self._add_to_dynamic_fleetcontrol_output(sim_time, output_dict)


class HailingBatchOptimizationSumObjectivesIRSDecision(HailingBatchOptimizationSumObjectivesIRSAssignment):
    """This class uses an assignment problem for which the assignment objective is the sum of single assignments.
    Customers will be assigned immediately according to the chosen IRS. No global optimization.
    """
    def _call_time_trigger_request_batch(self, simulation_time):
        """This method can be used to perform time-triggered processes, e.g. the optimization of the current
        assignments of simulation vehicles of the fleet.

        :param simulation_time: current simulation time
        :type simulation_time: float
        :return: {}: rid -> offer
        """
        self._update_veh_av_from_scratch(simulation_time, consider_only_locked=True)
