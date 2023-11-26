# -------------------------------------------------------------------------------------------------------------------- #
# standard distribution imports
# -----------------------------
import logging
from abc import ABC, abstractmethod
import typing as tp

# additional module imports (> requirements)
# ------------------------------------------

# src imports
# -----------
from src.fleetctrl.FleetControlBase import FleetControlBase
from src.fleetctrl.planning.VehiclePlan import VehiclePlan, BoardingPlanStop, RoutingTargetPlanStop
from dev.fleetctrl.hailing.objectives import return_hailing_objective_function
from src.simulation.Offers import TravellerOffer

# -------------------------------------------------------------------------------------------------------------------- #
# global variables
# ----------------
from src.misc.globals import *
LOG = logging.getLogger(__name__)


# TODO # abolish class; instead use Pooling with special insertion (and vehicle capacity limit)
# TODO # define class that can contain (prq, lock) or (cunit, desired_soc, lock)
# TODO # instead of ordered_list_prq_lock_tuples -> use HailingTask
class VehicleTask(ABC):
    def __init__(self, vid, task_info_dict):
        self.vid = vid
        self.task_info_dict = task_info_dict
        # TODO # update for charging
        self.tuple_rep = (self.vid, self.task_info_dict.get(G_FCTRL_PRQ), self.task_info_dict.get(G_VR_LOCKED))

    def __eq__(self, other):
        if self.vid == other.vid and self.task_info_dict == other.task_info_dict:
            return True
        else:
            return False

    def __lt__(self, other):
        return self.tuple_rep < other.tuple_rep

    def __hash__(self):
        return hash(self.tuple_rep)

    def __str__(self):
        return ",".join([str(x) for x in self.tuple_rep])

    def return_task_infos(self):
        """This method returns the task infos.

        :return: plan-info dict
        """
        return self.task_info_dict

    @abstractmethod
    def create_plan_stops(self, sim_time, bt):
        """This method creates plan stops out of the minimum required information that are stored in the vehicle task.

        :return: list of PlanStops
        """
        return []


class HailingTask(VehicleTask):
    def create_plan_stops(self, sim_time, bt):
        """This method creates plan stops out of the minimum required information that are stored in the vehicle task.

        :return: list of PlanStops
        """
        prq = self.task_info_dict[G_FCTRL_PRQ]
        lock_status = self.task_info_dict[G_VR_LOCKED]
        list_plan_stops = []
        new_rid_struct = prq.get_rid_struct()
        if prq.pu_time is None or prq.pu_time + bt > sim_time:
            prq_o_stop_pos, prq_t_pu_earliest, prq_t_pu_latest = prq.get_o_stop_info()
            earliest_pickup_time_dict = {new_rid_struct: prq_t_pu_earliest}
            latest_pickup_time_dict = {new_rid_struct: prq_t_pu_latest}
            list_plan_stops.append(BoardingPlanStop(prq_o_stop_pos, boarding_dict={+1: [new_rid_struct]}, earliest_pickup_time_dict=earliest_pickup_time_dict,
                             latest_pickup_time_dict=latest_pickup_time_dict, change_nr_pax=1, duration=bt, locked=lock_status))
            list_plan_stops[-1].started_at = prq.pu_time
        d_stop_pos, prq_t_do_latest, prq_max_trip_time = prq.get_d_stop_info()
        list_plan_stops.append(BoardingPlanStop(d_stop_pos, boarding_dict={-1: [prq.get_rid_struct()]}, change_nr_pax=-1, duration=bt, locked=lock_status))
        return list_plan_stops


class RepositioningTask(VehicleTask):
    def create_plan_stops(self, sim_time, bt):
        """This method creates plan stops out of the minimum required information that are stored in the vehicle task.

        :return: list of PlanStops
        """
        destination_pos = self.task_info_dict[G_VR_LEG_END_POS]
        lock_status = self.task_info_dict[G_VR_LOCKED]
        return [RoutingTargetPlanStop(destination_pos, locked=lock_status)]


class ChargingTask(VehicleTask):
    def create_plan_stops(self, sim_time, bt):
        """This method creates plan stops out of the minimum required information that are stored in the vehicle task.

        :return: list of PlanStops
        """
        # TODO # ChargingTask()
        return []


class HailingVehiclePlan(VehiclePlan):
    def __init__(self, veh_obj, sim_time, routing_engine, bt, ordered_task_list):
        """The hailing operation assumes that people do not board at the same time. Therefore, there each request leads
        to one boarding and one alighting stop.

        :param veh_obj: simulation vehicle
        :param sim_time: current simulation time
        :param routing_engine: routing engine
        :param bt: boarding time
        :param ordered_task_list: list of (prq, lock_status) tuples
        """
        self.ordered_task_list: tp.List[VehicleTask] = ordered_task_list
        list_plan_stops: tp.List[PlanStop] = []
        for task in ordered_task_list:
            list_plan_stops.extend(task.create_plan_stops(sim_time, bt))
        super().__init__(veh_obj, sim_time, routing_engine, list_plan_stops, copy=True)
        self.vid = veh_obj.vid
        self.feasible = True
        # self.update_tt_and_check_plan(veh_obj, sim_time, routing_engine, keep_feasible=True)

    def __eq__(self, other):
        if self.vid == other.vid and self.get_ordered_task_list() == other.get_ordered_task_list():
            return True
        else:
            return False

    def __lt__(self, other):
        if self.vid < other.vid:
            return True
        elif self.vid > other.vid:
            return False
        else:
            if self.ordered_task_list < other.ordered_task_list:
                return True
            else:
                return False

    def __hash__(self):
        tuple_tuple_rep = tuple((task.tuple_rep for task in self.ordered_task_list))
        return hash(tuple_tuple_rep)

    def __str__(self):
        return "\n".join([str(task) for task in self.ordered_task_list])

    def return_all_rids(self):
        return [task.task_info_dict[G_FCTRL_PRQ].get_rid_struct() for task in self.ordered_task_list
                if type(task) == HailingTask]

    def return_non_locked_rids(self):
        return [task.task_info_dict[G_FCTRL_PRQ].get_rid_struct() for task in self.ordered_task_list
                if (type(task) == HailingTask and not task.task_info_dict.get(G_VR_LOCKED))]

    def remove_alighting_prq(self, rid):
        """This method adapts the ordered_list_prq_lock_tuples after a request is removed.

        :param rid: rid of PlanRequest to remove
        :return: None
        """
        new_ordered_task_list = []
        for task in self.get_ordered_task_list():
            if type(task) != HailingTask or task.task_info_dict[G_FCTRL_PRQ].get_rid_struct() != rid:
                new_ordered_task_list.append(task)
        self.ordered_task_list = new_ordered_task_list

    def remove_prq(self, rid, veh_obj, sim_time, routing_engine, bt):
        """This method re-initiates the HailingVehiclePlan without the tasks related to PlanRequest rid.

        :param rid: rid of PlanRequest
        :param veh_obj: Simulation Vehicle
        :param sim_time: current simulation time
        :param routing_engine: reference to routing engine
        :param bt: boarding time
        :return: True if new_ordered_task_list is not empty, False if empty
        """
        new_ordered_task_list = []
        for task in self.get_ordered_task_list():
            if type(task) != HailingTask or task.task_info_dict[G_FCTRL_PRQ].get_rid_struct() != rid:
                new_ordered_task_list.append(task)
        self.ordered_task_list = new_ordered_task_list
        if new_ordered_task_list:
            self.ordered_task_list = new_ordered_task_list
            list_plan_stops = []
            for task in self.ordered_task_list:
                list_plan_stops.extend(task.create_plan_stops(sim_time, bt))
            self.list_plan_stops = list_plan_stops
            self.update_tt_and_check_plan(veh_obj, sim_time, routing_engine, keep_feasible=True)
            return True
        else:
            self.list_plan_stops = []
            self.update_tt_and_check_plan(veh_obj, sim_time, routing_engine, keep_feasible=True)
            return False

    def return_non_locked_tasks(self):
        """This method returns the list, which serves as input for a new HailingVehiclePlan.

        :return: task list
        """
        # first check whether there are stops but not tasks

        return_list = []
        for task in self.get_ordered_task_list():
            if not task.task_info_dict.get(G_VR_LOCKED):
                return_list.append(task)
        return return_list

    def return_locked_tasks(self):
        """This method returns a list of all locked tasks.

        :return: task list
        """
        # first check whether there are stops but not tasks

        return_list = []
        for task in self.get_ordered_task_list():
            if task.task_info_dict.get(G_VR_LOCKED):
                prq = task.return_task_infos().get(G_FCTRL_PRQ)
                # make exception for reservation tasks
                if prq is not None:
                    if not prq.get_reservation_flag():
                        return_list.append(task)
                else:
                    return_list.append(task)
        return return_list

    def split_reservation_tasks(self, keep_non_locked_non_reservation_flag):
        """This method splits the task list in non-reserved and reserved tasks.

        :param keep_non_locked_non_reservation_flag: determines whether non-locked non-reservation tasks will be kept
        :return: (non_reserved_tasks, reserved_tasks)
        """
        non_reserved_tasks = []
        reserved_tasks = []
        for task in self.ordered_task_list:
            prq = task.return_task_infos().get(G_FCTRL_PRQ)
            if prq.get_reservation_flag():
                reserved_tasks.append(task)
            else:
                if keep_non_locked_non_reservation_flag or task.return_task_infos().get(G_VR_LOCKED):
                    non_reserved_tasks.append(task)
        return non_reserved_tasks, reserved_tasks

    def return_reservation_tasks(self):
        """This method returns a list of all tasks related to reservation requests (which have not yet been changed
        to immediate requests).

        :return: task list
        """
        return_list = []
        for task in self.ordered_task_list:
            prq = task.return_task_infos().get(G_FCTRL_PRQ)
            if prq.get_reservation_flag():
                return_list.append(task)
        return return_list

    def keep_locked_and_reserved_tasks(self, veh_obj, routing_engine, sim_time, const_bt):
        """This method removes all non locked PlanRequests, which are no reserved requests, from the current
        HailingVehiclePlan. It updates the PlanStops as well.

        :param veh_obj: simulation vehicle
        :param routing_engine: routing engine
        :param sim_time: current simulation time
        :param const_bt: boarding time
        :return: None
        """
        orig_list = self.ordered_task_list
        # tasks
        keep_list = []
        for task in self.get_ordered_task_list():
            prq = task.return_task_infos().get(G_FCTRL_PRQ)
            if task.task_info_dict.get(G_VR_LOCKED) or prq.get_reservation_flag():
                keep_list.append(task)
        self.ordered_task_list = keep_list
        # plan stops
        list_plan_stops = []
        for task in self.ordered_task_list:
            list_plan_stops.extend(task.create_plan_stops(sim_time, const_bt))
        self.list_plan_stops = list_plan_stops
        if self.ordered_task_list != orig_list:
            self.update_tt_and_check_plan(veh_obj, sim_time, routing_engine, keep_feasible=True)

    def remove_non_locked_tasks(self, veh_obj, routing_engine, sim_time, bt):
        """This method removes all non locked PlanRequests from the current HailingVehiclePlan. It updates the PlanStops
        as well.

        :param veh_obj: simulation vehicle
        :param routing_engine: routing engine
        :param sim_time: current simulation time
        :param bt: boarding time
        :return: None
        """
        # tasks
        keep_tasks = []
        for task in self.get_ordered_task_list():
            if task.task_info_dict.get(G_VR_LOCKED):
                keep_tasks.append(task)
        self.ordered_task_list = keep_tasks
        # plan stops
        list_plan_stops = []
        for task in self.ordered_task_list:
            list_plan_stops.extend(task.create_plan_stops(sim_time, bt))
        self.list_plan_stops = list_plan_stops
        self.update_tt_and_check_plan(veh_obj, sim_time, routing_engine, keep_feasible=True)

    def get_ordered_task_list(self):
        if self.list_plan_stops and not self.ordered_task_list:
            for pstop in self.list_plan_stops:
                # create Tasks from PlanStops
                task_info_dict = {G_VR_LEG_END_POS: pstop.get_pos(), G_VR_LOCKED: pstop.is_locked()}
                if pstop.get_charging_power() > 0:
                    # TODO # create ChargingTask()
                    pass
                else:
                    # repositioning
                    self.ordered_task_list.append(RepositioningTask(self.vid, task_info_dict))
        return self.ordered_task_list

    def lock_rid(self, rid, veh_obj):
        """This method locks tasks, plan_stops and route_legs.

        :param rid: request ID to lock
        :param veh_obj: vehicle object the assignment is supposed to be locked to
        :return: None
        """
        for task in self.ordered_task_list:
            prq = task.task_info_dict.get(G_FCTRL_PRQ)
            if prq and prq.get_rid_struct() == rid:
                prq.locked = True
                task.task_info_dict[G_VR_LOCKED] = True
                break
            elif prq and prq.get_rid_struct() != rid:
                if not prq.locked:
                    LOG.info(f"Try to lock an assignment that is scheduled after another UNLOCKED assignment.")
                    LOG.info(f"Would also lock the former one and lead to potential conflict in case it's later deleted")
                    LOG.info(f"Work around: locking neither of them, which means the latter one will never be locked...")
                    return
        for ps in self.list_plan_stops:
            if rid in ps.get_list_boarding_rids() or rid in ps.get_list_alighting_rids():
                ps.locked = True
        leg_in_route = False
        for route_leg in veh_obj.assigned_route:
            route_leg.update_lock_status()
            if rid in [x.rid for x in route_leg.rq_dict[-1]]:
                leg_in_route = True
                break
        if not leg_in_route:
            LOG.warning(f"Something went wrong before trying to lock a task."
                        f"The task ({rid}) to lock is not in the respective route of vehicle {veh_obj.vid}...")
            raise EnvironmentError

    def return_after_locked_availability(self, veh_obj, sim_time):
        """This method estimates the availability of a vehicle after all locked VRL are done.

        :param veh_obj: simulation vehicle
        :param sim_time: current simulation time
        :return: last_time, last_pos, last_soc
        """
        last_time = sim_time
        last_pos = veh_obj.pos
        last_soc = veh_obj.soc
        for pstop in reversed(self.list_plan_stops):
            if pstop.is_locked():
                last_pos = pstop.get_pos()
                _, last_soc = pstop.get_planned_arrival_and_departure_soc()
                _, last_time = pstop.get_planned_arrival_and_departure_time()
                break
        return last_time, last_pos, last_soc

    def return_after_unlocked_availability(self, veh_obj, sim_time):
        """This method treats unlocked VRLs as locked when it comes to vehicle availability. This can be useful for
        IRS systems that still wait for customer responses to lock a VRL, but already assigned the plan and do not want
        to change it unless a customer cancels.

        :param veh_obj: simulation_vehicle
        :param sim_time: current simulation time
        :return: last_time, last_pos, last_soc
        """
        last_time = sim_time
        last_pos = veh_obj.pos
        last_soc = veh_obj.soc
        if self.list_plan_stops:
            pstop = self.list_plan_stops[-1]
            last_pos = pstop.get_pos()
            _, last_soc = pstop.get_planned_arrival_and_departure_soc()
            _, last_time = pstop.get_planned_arrival_and_departure_time()
        return last_time, last_pos, last_soc

    def has_only_locked_stops(self):
        """This method checks if this HailingVehiclePlan contains only locked stops.

        :return: True/False
        """
        for pstop in self.list_plan_stops:
            if not pstop.is_locked():
                return False
        return True


class HailingFleetControlBase(FleetControlBase, ABC):
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
        super().__init__(op_id, operator_attributes, list_vehicles, routing_engine, zone_system, scenario_parameters,
                         dir_names, op_charge_depot_infra=op_charge_depot_infra, list_pub_charging_infra=list_pub_charging_infra)
        sim_start_time = scenario_parameters[G_SIM_START_TIME]
        self.veh_plans: tp.Dict[int, HailingVehiclePlan] = {}  # vid -> currently assigned HailingVehiclePlan
        # HailingVehiclePlans can be used as keys
        self.v2a = {}    # vid -> HailingVehiclePlan -> 1
        self.r2a = {}    # rid -> HailingVehiclePlan -> 1
        for veh_obj in self.sim_vehicles:
            vid = veh_obj.vid
            self.veh_plans[vid] = HailingVehiclePlan(veh_obj, sim_start_time, self.routing_engine, self.const_bt, [])
            self.v2a[vid] = {}
        # This attribute saves the updated vehicle information for vehicle assignments
        self.considered_veh: tp.Dict[tuple, tp.Dict[int, tuple]] = {}    # pos -> {}: vid -> (last_time, last_soc)
        self.considered_veh_time = None
        self.vr_ctrl_f = return_hailing_objective_function(operator_attributes[G_OP_VR_CTRL_F])
        self.current_surge_factor = 1.0

    def receive_status_update(self, vid, simulation_time, list_finished_VRL, force_update=False):
        """This method can be used to update plans and trigger processes whenever a simulation vehicle finished some
         VehicleRouteLegs.
        :param vid: vehicle id
        :type vid: int
        :param simulation_time: current simulation time
        :type simulation_time: float
        :param list_finished_VRL: list of VehicleRouteLeg objects
        :type list_finished_VRL: list
        :param force: should force the updating of plans after optimization
        :type force: bool
        """
        if list_finished_VRL or force_update:
            for vrl in list_finished_VRL:
                if vrl.status == VRL_STATES.REPO_TARGET:
                    hailing_plan = self.veh_plans[vid]
                    if hailing_plan.ordered_task_list and type(hailing_plan.ordered_task_list[0]) == RepositioningTask:
                        self.veh_plans[vid].ordered_task_list = self.veh_plans[vid].ordered_task_list[1:]
            self.veh_plans[vid].update_plan(self.sim_vehicles[vid], simulation_time, self.routing_engine,
                                            list_finished_VRL)

    def lock_current_vehicle_plan(self, vid):
        super().lock_current_vehicle_plan(vid)

    def _lock_vid_rid_pickup(self, sim_time, vid, rid):
        """This method constrains the pick-up of a rid. In the pooling case, the pick-up time is constrained to a very
        short time window. In the hailing case, the Task to serve rid is locked for the vehicle.

        WHEN OVERWRITING: MIND DATABASE UPDATES!

        :param sim_time: current simulation time
        :param vid: vehicle id
        :param rid: PlanRequest id
        :return: None
        """
        veh_obj = self.sim_vehicles[vid]
        self.veh_plans[vid].lock_rid(rid, veh_obj)
        prq = self.rq_dict[rid]
        prq.lock_request()

    def _create_user_offer(self, prq, simulation_time, assigned_vehicle_plan=None, offer_dict_without_plan={}):
        """This method should be overwritten to create an offer for the request rid
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
        :return: offer dictionary
        :rtype: dictionary
        """
        rid = prq.get_rid_struct()
        offer = {}  # should be filled by overwriting method
        if prq.status in [G_PRQS_NO_OFFER, G_PRQS_INIT_OFFER, G_PRQS_ACC_OFFER, G_PRQS_LOCKED]:
            if assigned_vehicle_plan is not None:
                offer = {}
                pu_time, do_time = assigned_vehicle_plan.pax_info.get(rid)
                # offer[G_OFFER_WAIT] = pu_time - simulation_time
                # TODO # this would mean customers could wait longer than their respective maximum waiting time
                wt = pu_time - prq.rq_time
                dt = do_time - pu_time
                fare = None
                if prq.offer is not None:
                    fare = prq.offer.get(G_OFFER_FARE)
                if prq.status <= G_PRQS_INIT_OFFER or not fare:
                    fare = self._compute_fare(simulation_time, prq)
                offer = TravellerOffer(rid, self.op_id, wt, dt, fare, additional_parameters=offer_dict_without_plan)
            elif offer_dict_without_plan:
                offer = TravellerOffer(rid, self.op_id, offer_dict_without_plan[G_OFFER_WAIT],
                                       offer_dict_without_plan[G_OFFER_DRIVE], offer_dict_without_plan[G_OFFER_FARE],
                                       additional_parameters=offer_dict_without_plan)
            else:
                offer = TravellerOffer(rid, self.op_id, None, None, None)
            prq.offer = offer
            prq.set_service_offered(offer)  # has to be called
        return offer

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
        prq = self.rq_dict[rid]
        prq.set_new_pickup_time_constraint(new_lpt, new_ept)

    def _prq_from_reservation_to_immediate(self, rid, sim_time):
        """This method is triggered when a reservation request becomes an immediate request.
        All database relevant methods can be triggered from here.

        :param rid: request id
        :param sim_time: current simulation time
        :return: None
        """
        for base_rid, epa in sorted(self.reserved_base_rids.items(), key=lambda x: x[1]):
            if epa - sim_time > self.opt_horizon:
                break
            else:
                LOG.debug(f"activate {base_rid} with epa {epa} for global optimisation at time {sim_time}!")
                del self.reserved_base_rids[base_rid]

    def _create_veh_av(self, sim_time, vid, consider_only_locked=True):
        """This method can be called by all sub

        :param sim_time: current simulation time
        :param vid: currently assigned of this vehicle will be used
        :param consider_only_locked: determination mode of availability
        :return: (last_pos, last_time, last_soc) | None if not available
        """
        # do not consider inactive vehicles
        veh_obj = self.sim_vehicles[vid]
        if veh_obj.status == VRL_STATES.OUT_OF_SERVICE:
            return None
        veh_plan = self.veh_plans[vid]
        if consider_only_locked:
            last_time, last_pos, last_soc = veh_plan.return_after_locked_availability(veh_obj, sim_time)
        else:
            try:
                last_time, last_pos, last_soc = veh_plan.return_after_unlocked_availability(veh_obj, sim_time)
            except:
                veh_plan.update_tt_and_check_plan(veh_obj, sim_time, self.routing_engine, keep_feasible=True)
                last_time, last_pos, last_soc = veh_plan.return_after_unlocked_availability(veh_obj, sim_time)
        if not self.max_wait_time or last_time <= sim_time + self.max_wait_time:
            return last_pos, last_time, last_soc
        else:
            return None

    def _get_vid_av_pos(self, sim_time, vid):
        """This function returns a possible location for the vehicle availability.

        :param sim_time: current simulation time
        :param vid: vehicle id
        :return: position tuple
        """
        av = self._create_veh_av(sim_time, vid, consider_only_locked=True)
        if av is not None:
            return av[0]
        else:
            return None

    def _update_veh_av_from_scratch(self, sim_time, consider_only_locked=True):
        """This method returns the availability of all vehicles. If self.max_wait_time is set, only vehicles with
        availability before sim_time + self.max_wait_time are considered. The availability is saved as dictionary
        {}: last_pos -> {}: vid -> (last_time, last_soc) in the self.considered_veh attribute.

        :param sim_time: current simulation time
        :param consider_only_locked: if True, only locked VRL are considered to determine the final availability
                    update of class attribute only in case of consider_only_locked!
        :return: considered-vehicle dictionary # pos -> {}: vid -> (last_time, last_soc, vid)
        """
        considered_veh = {}
        for veh_obj in self.sim_vehicles:
            vid = veh_obj.vid
            av = self._create_veh_av(sim_time, vid, consider_only_locked=consider_only_locked)
            if av is not None:
                last_pos, last_time, last_soc = av
                try:
                    considered_veh[last_pos][veh_obj.vid] = (last_time, last_soc)
                except KeyError:
                    considered_veh[last_pos] = {veh_obj.vid: (last_time, last_soc)}
        if consider_only_locked:
            self.considered_veh_time = sim_time
            self.considered_veh = considered_veh
        return considered_veh

    def _update_single_veh_av(self, sim_time, veh_obj, old_pos):
        """This method updates the availability of a single vehicle.

        :param sim_time: current simulation time
        :param old_pos: last available position of the vehicle
        :param consider_only_locked: if True, only locked VRL are considered to determine the final availability
        :return: None (class attribute is changed)
        """
        sync_flag = True
        try:
            if old_pos is not None:
                del self.considered_veh[old_pos][veh_obj.vid]
                if len(self.considered_veh[old_pos]) == 0:
                    del self.considered_veh[old_pos]
        except KeyError:
            sync_flag = False
        if sync_flag:
            av = self._create_veh_av(sim_time, veh_obj.vid, consider_only_locked=True)
            if av is not None:
                last_pos, last_time, last_soc = av
                try:
                    self.considered_veh[last_pos][veh_obj.vid] = (last_time, last_soc)
                except KeyError:
                    self.considered_veh[last_pos] = {veh_obj.vid: (last_time, last_soc)}
        else:
            LOG.info(f"_update_single_veh_av({sim_time, veh_obj.vid}) out of sync. Computing availability from scratch!"
                     f"\n veh_obj: {veh_obj}")
            self._update_veh_av_from_scratch(sim_time, True)

    def _change_assignments(self, sim_time, list_remove_assignments, list_new_assignments):
        """This method changes the assignments after decisions are made and translates the assignments into
        VehicleRouteLegs. It returns a list of (vid,assignment)-pairs in which non-locked tasks have been removed.
        :param sim_time: current simulation time
        :param list_remove_assignments: assignments to be removed
        :param list_new_assignments: new assignments to be made
        :return removed_assignments: list of (vid,assignment)-pairs in which non-locked tasks have been removed
        """
        update_vids = {}
        for remove_assignment in list_remove_assignments:
            veh_obj = self.sim_vehicles[remove_assignment.vid]
            if veh_obj.vid in self.vid_with_reserved_rids:
                remove_assignment.keep_locked_and_reserved_tasks(veh_obj, self.routing_engine, sim_time, self.const_bt)
            else:
                remove_assignment.remove_non_locked_tasks(veh_obj, self.routing_engine, sim_time, self.const_bt)
            self.assign_vehicle_plan(veh_obj, remove_assignment, sim_time, force_assign=False)
            update_vids[veh_obj] = 1
        for new_assignment in list_new_assignments:
            veh_obj = self.sim_vehicles[new_assignment.vid]
            self.assign_vehicle_plan(veh_obj, new_assignment, sim_time, force_assign=False)
            update_vids[veh_obj] = 1
        # for veh_obj in update_vids.keys():
        #     self.veh_plans[veh_obj.vid].update_tt_and_check_plan(veh_obj, sim_time, self.routing_engine,
        #                                                          keep_feasible=True)

    def assign_vehicle_plan(self, veh_obj, vehicle_plan, sim_time, force_assign=False, add_arg=None):
        tmp = super().assign_vehicle_plan(veh_obj, vehicle_plan, sim_time, force_assign=force_assign, add_arg=add_arg)
        return tmp

    def _delete_request(self, rid):
        """This method can be used to remove all standard data base entries of a request. Can be overwritten by
        subclass if necessary.

        :param rid: request id
        :return: None
        """
        new_r2a = list(self.r2a.get(rid, {}).keys())
        for veh_plan in new_r2a:
            self._remove_hailing_plan_from_db(veh_plan, True)
        del self.rq_dict[rid]

    def _reset_db(self):
        """This method resets the databases v2a and r2a.

        :return: None
        """
        self.v2a = {}
        for vid in range(self.nr_vehicles):
            self.v2a[vid] = {}
        self.r2a = {}

    def _add_hailing_plan_to_db(self, veh_plan):
        """This method adds a hailing plan to the respective request and vehicle data bases

        :param veh_plan: HailingPlan
        :return: None
        """
        vid = veh_plan.vid
        self.v2a[vid][veh_plan] = 1
        for rid in veh_plan.return_all_rids():
            try:
                self.r2a[rid][veh_plan] = 1
            except KeyError:
                self.r2a[rid] = {veh_plan: 1}

    def _remove_hailing_plan_from_db(self, veh_plan, reset=False):
        """This method removes a hailing plan from the request and vehicle data bases.

        :param veh_plan: HailingPlan
        :param reset: empties respective r2a and v2a if true
        :return: None
        """
        vid = veh_plan.vid
        for rid in veh_plan.return_all_rids():
            if reset:
                self.r2a[rid] = {}
            else:
                try:
                    del self.r2a[rid][veh_plan]
                except KeyError:
                    pass
        if reset:
            self.v2a[vid] = {}
        else:
            try:
                del self.v2a[vid][veh_plan]
            except KeyError:
                pass
