# -------------------------------------------------------------------------------------------------------------------- #
# standard distribution imports
# -----------------------------
import logging
import time

# additional module imports (> requirements)
# ------------------------------------------

# src imports
# -----------
from src.fleetctrl.planning.PlanRequest import PlanRequest
from dev.fleetctrl.HailingFleetcontrolBase import HailingFleetControlBase, HailingVehiclePlan, HailingTask
from dev.fleetctrl.hailing.immediate.searchAll import search_rv_for_new_prq

# -------------------------------------------------------------------------------------------------------------------- #
# global variables
# ----------------
from src.misc.globals import *
# from IPython import embed
LOG = logging.getLogger(__name__)
LARGE_INT = 100000


class HailingNNIRSOnly(HailingFleetControlBase):
    """This class uses the Nearest-Neighbor Policy as immediate response system, which already makes assignments and
    considers these for future requests."""
    # TODO # add dependency on IRSDecision Simulation Flow
    def __init__(self, op_id : int, operator_attributes, list_vehicles,
                 routing_engine, zone_system, scenario_parameters,
                 dir_names, op_charge_depot_infra=None,
                 list_pub_charging_infra= []):
        """The general attributes for the fleet control module are initialized. Strategy specific attributes are
        introduced in the children classes.

        :param op_id: operator id
        :type op_id: int
        :param operator_attributes: dictionary with keys from globals and respective values
        :type operator_attributes: dict
        :param list_vehicles: simulation vehicles; their assigned plans should be instances of the VehicleRouteLeg class
        :type list_vehicles: list
        :param routing_engine: routing engine
        :type routing_engine: Network
        :param zone_system: zone system
        :type zone_system: ZoneSystem
        :param scenario_parameters: access to all scenario parameters (if necessary)
        :type scenario_parameters: dict
        :param dir_names: dictionary with references to data and output directories
        :type dir_names: dict
        :param charging_management: reference to a ChargingAndDepotManagement class (optional)
        :type charging_management: ChargingAndDepotManagement
        """
        self.rid_assignment = {}    # rid -> veh_assignment (HailingVehiclePlan)
        super().__init__(op_id, operator_attributes, list_vehicles, routing_engine, zone_system, scenario_parameters,
                         dir_names, op_charge_depot_infra=op_charge_depot_infra, list_pub_charging_infra=list_pub_charging_infra)
        self._init_dynamic_fleetcontrol_output_key(G_FCTRL_CT_RQU)
        # overwrite RV heuristic parameter to just receive best solution
        self.rv_heuristics[G_RA_MAX_VR] = 1

    def user_request(self, rq, sim_time):
        """This method is triggered for a new incoming request. It generally generates a PlanRequest from the rq and
        adds it to the database. It has to return an offer to the user. An empty dictionary means no offer is made!
        The plan stops are already assigned and considered in the vehicle availabilty for future requests.

        :param rq: request object containing all request information
        :type rq: RequestDesign
        :param sim_time: current simulation time
        :type sim_time: float
        :return: offer
        :rtype: dict
        """
        t0 = time.perf_counter()
        if self.considered_veh_time != sim_time:
            self._update_veh_av_from_scratch(sim_time)
        prq = PlanRequest(rq, self.routing_engine, min_wait_time=self.min_wait_time, max_wait_time=self.max_wait_time,
                          boarding_time = self.const_bt)
        rid_struct = prq.get_rid_struct()
        if prq.o_pos == prq.d_pos:
            LOG.debug("automatic decline!")
            self._create_rejection(prq, sim_time)
        #
        o_pos, t_pu_earliest, t_pu_latest = prq.get_o_stop_info()
        if t_pu_earliest - sim_time > self.opt_horizon:
            prq.set_reservation_flag(True)
            self.reserved_base_rids[rid_struct] = t_pu_earliest
        #
        self.rq_dict[rid_struct] = prq
        list_new_hailing_plans = search_rv_for_new_prq(self, prq, sim_time)
        if list_new_hailing_plans:
            new_veh_plan = list_new_hailing_plans[0]
            # make assignment and adapt vehicle availability
            vid = new_veh_plan.vid
            veh_obj = self.sim_vehicles[vid]
            old_veh_plan = self.veh_plans[vid]
            old_av_pos = self._get_vid_av_pos(sim_time, vid)
            self._change_assignments(sim_time, [old_veh_plan], [new_veh_plan])
            # create offer
            self._create_user_offer(prq, sim_time, new_veh_plan)
            # update vehicle availability
            self._update_single_veh_av(sim_time, veh_obj, old_av_pos)
        else:
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

    def user_confirms_booking(self, rid, simulation_time):
        """This method is used to confirm a customer booking. This can trigger some database processes.
        -> lock customer in assignments; no update of availability necessary

        :param rid: request id
        :type rid: int
        :param simulation_time: current simulation time
        :type simulation_time: float
        """
        prq = self.rq_dict[rid]
        prq.set_service_accepted()
        # automatically lock
        vid = self.rid_assignment[rid].vid
        self._lock_vid_rid_pickup(simulation_time, vid, rid)
        # treatment of reservation requests
        prq = self.rq_dict.get(rid)
        if vid is not None and prq is not None and prq.get_reservation_flag():
            try:
                self.vid_with_reserved_rids[vid].append(rid)
            except KeyError:
                self.vid_with_reserved_rids[vid] = [rid]

    def user_cancels_request(self, rid, simulation_time):
        """This method is used to confirm a customer cancellation. This can trigger some database processes.
        -> create new assignment without customer, update of availability

        :param rid: request id
        :type rid: int
        :param simulation_time: current simulation time
        :type simulation_time: float
        """
        #if rid == 226:
            #embed()
        old_assignment = self.rid_assignment.get(rid)
        if old_assignment:
            vid = old_assignment.vid
            prq = self.rq_dict.get(rid)
            if vid is not None and prq is not None and prq.get_reservation_flag():
                list_reserved_rids = self.vid_with_reserved_rids.get(vid, [])
                if rid in list_reserved_rids:
                    list_reserved_rids.remove(rid)
                    if list_reserved_rids:
                        self.vid_with_reserved_rids[vid] = list_reserved_rids
                    else:
                        del self.vid_with_reserved_rids[vid]
            #
            veh_obj = self.sim_vehicles[vid]
            # create new assignment without rid
            old_tasks = old_assignment.get_ordered_task_list()
            new_tasks = []
            for task in old_tasks:
                prq = task.task_info_dict.get(G_FCTRL_PRQ)
                if prq is None or prq.get_rid_struct() != rid:
                    new_tasks.append(task)
            new_assignment = HailingVehiclePlan(veh_obj, simulation_time, self.routing_engine, self.const_bt, new_tasks)
            # adapt vehicle assignment and vehicle availability
            self._change_assignments(simulation_time, [old_assignment], [new_assignment])
        try:
            del self.reserved_base_rids[rid]
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
        assignment = self.rid_assignment[rid]
        assignment.remove_alighting_prq(rid)
        del self.rq_dict[rid]
        del self.rid_assignment[rid]

    def _call_time_trigger_request_batch(self, simulation_time):
        """This method can be used to perform time-triggered processes, e.g. the optimization of the current
        assignments of simulation vehicles of the fleet. Here, it is used to update the surge pricing factor and
        vehicle availability, whereas all current assignments are considered, even if they are not yet locked (confirmed
        by customer).

        :param simulation_time: current simulation time
        :type simulation_time: float
        """
        self._update_veh_av_from_scratch(simulation_time, consider_only_locked=False)

    def change_prq_time_constraints(self, sim_time, rid, new_lpt, new_ept=None):
        """This method should be called when the hard time constraints of a customer should be changed.
        It changes the PlanRequest attributes. Moreover, this method called on child classes should adapt the
        PlanStops of VehiclePlans containing this PlanRequest and recheck feasibility. The VehiclePlan method
        update_prq_hard_constraints() can be used for this purpose.

        :param sim_time: current simulation time
        :param rid: request id
        :param new_lpt: new latest pickup time, None is ignored
        :param new_ept: new earliest pickup time, None is ignored
        :return: None
        """
        super().change_prq_time_constraints(self, sim_time, rid, new_lpt)

    def _change_assignments(self, sim_time, list_remove_assignments, list_new_assignments):
        """This method changes the assignments after decisions are made and translates the assignments into
        VehicleRouteLegs.

        :param sim_time: current simulation time
        :param list_remove_assignments: assignments to be removed
        :param list_new_assignments: new assignments to be made
        :return: None
        """
        for remove_assignment in list_remove_assignments:
            list_rids = remove_assignment.return_all_rids()
            for rid in list_rids:
                del self.rid_assignment[rid]
        super()._change_assignments(sim_time, list_remove_assignments, list_new_assignments)
        for new_assignment in list_new_assignments:
            list_rids = new_assignment.return_all_rids()
            for rid in list_rids:
                self.rid_assignment[rid] = new_assignment


class HailingNNOnlyBatchVariableSearchDirection(HailingFleetControlBase):
    """This class uses the Nearest-Neighbor Policy as for request batches. It searches around requests if the
    number of open requests is smaller than the number of available vehicles, and around vehicles if the number of
    open requests is larger than the number of available vehicles.
    """
    # TODO # HailingNNOnlyBatchVariableSearchDirection()

    def receive_status_update(self, vid, simulation_time, list_finished_VRL):
        pass

    def user_request(self, rq, simulation_time):
        pass

    def user_confirms_booking(self, rid, simulation_time):
        pass

    def user_cancels_request(self, rid, simulation_time):
        pass

    def acknowledge_boarding(self, rid, vid, simulation_time):
        pass

    def acknowledge_alighting(self, rid, vid, simulation_time):
        pass

    def _call_time_trigger_request_batch(self, simulation_time):
        pass
