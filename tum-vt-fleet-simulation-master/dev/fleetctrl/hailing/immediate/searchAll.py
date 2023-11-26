import logging
from dev.fleetctrl.HailingFleetcontrolBase import HailingTask, HailingVehiclePlan
from src.misc.globals import *
LOG = logging.getLogger(__name__)
WARNING_FOR_SEARCH_RADIUS = 900


def search_all_unlocked_rv_in_tw_one_unlocked(fctrl, list_prq_status, sim_time, blocked_cars=None):
    """This function can be used in a batch to find all RV connection for all requests with status in 'list_prq_status'.
    It returns feasible HailingVehiclePlans, which are based on the locked vehicle plans and add one
    new unlocked HailingTask. An RV heuristic G_RA_MAX_VR to limit the number of RV connections can be applied.

    :param fctrl: reference to hailing fleet control; fctrl.considered_veh determines, which vehicles are considered
    :param list_prq_status: list of PlanRequest status, for which VR pairs should be searched
    :param sim_time: current simulation time
    :param blocked_cars: list to cars to be excluded from optimization
    :return: list of HailingPlans
    """
    # RV heuristic
    search_for_prq = {}
    # request <-> veh plans
    if blocked_cars is None:
        blocked_cars = []
    nr_locked = 0
    prqs = fctrl.rq_dict
    for rid, prq in prqs.items():
        if prq.get_reservation_flag():
            continue
        if prq.locked:
            nr_locked += 1
        if prq.status in list_prq_status and not prq.locked:
            try:
                search_for_prq[prq.o_pos].append(prq)
            except KeyError:
                search_for_prq[prq.o_pos] = [prq]
    # search vehicles in max-wait time radius
    av_dict = fctrl.considered_veh
    return_list = []
    for o_pos, list_prq in search_for_prq.items():
        route_infos = fctrl.routing_engine.return_travel_costs_Xto1(fctrl.considered_veh.keys(), o_pos,
                                                                    max_cost_value=fctrl.max_wait_time)
        for prq in list_prq:
            return_list.extend(sort_single_prq_rv(prq, route_infos, av_dict, blocked_cars, fctrl,
                                                  keep_non_locked_flag=False))
    # make task insertions and return
    return task_insertion(return_list, sim_time, fctrl)


def search_rv_for_new_prq(fctrl, prq, sim_time, blocked_cars=None, only_nn=True):
    """This function can be used to find RV connections for a new request.
    It returns feasible HailingVehiclePlans, which are based on the locked vehicle plans and add one
    new unlocked HailingTask. An RV heuristic G_RA_MAX_VR to limit the number of RV connections can be applied.

    :param fctrl: reference to hailing fleet control; fctrl.considered_veh determines, which vehicles are considered
    :param prq: PlanRequest, for which next vehicle should be found
    :param sim_time: current simulation time
    :param blocked_cars: list of cars excluded from search or None
    :param only_nn: only create insertions for vehicle that can pick-up prq first (only for immediate requests)
    :return: list of HailingPlans
    """
    o_pos, t_pu_earliest, t_pu_latest = prq.get_o_stop_info()
    if blocked_cars is None:
        blocked_cars = []
    # build availability dictionary for reservation (consider all unlocked tasks as well)
    av_dict = {}
    for vid in range(fctrl.nr_vehicles):
        av = fctrl._create_veh_av(sim_time, vid, consider_only_locked=False)
        if av is not None:
            last_pos, last_time, last_soc = av
            try:
                av_dict[last_pos][vid] = (last_time, last_soc)
            except KeyError:
                av_dict[last_pos] = {vid: (last_time, last_soc)}
    if prq.get_reservation_flag():
        max_routes = fctrl.rv_heuristics[G_RH_R_NWS]
        search_radius = fctrl.opt_horizon
    else:
        max_routes = fctrl.rv_heuristics.get(G_RH_I_NWS, None)
        search_radius = t_pu_latest - sim_time
    route_infos = fctrl.routing_engine.return_travel_costs_Xto1(av_dict, o_pos,
                                                                max_cost_value=search_radius, max_routes=max_routes)
    # create rv list
    rv_list = sort_single_prq_rv(prq, route_infos, av_dict, blocked_cars, fctrl, only_nn, keep_non_locked_flag=True)
    # make task insertions and return
    return task_insertion(rv_list, sim_time, fctrl)


def sort_single_prq_rv(prq, route_infos, av_dict, blocked_cars, fctrl, only_nn=False, keep_non_locked_flag=False):
    """This function sorts the RVs according to available routing information and utilizes a first RV heuristic.

    :param prq: PlanRequest
    :param route_infos: list of (veh_av_pos, routing_cfv, tt, td) tuples
    :param blocked_cars: list of vehicles that should not be considered
    :param fctrl: Fleetcontrol instance
    :param only_nn: only create insertions for vehicle that can pick-up prq first (only for immediate requests)
    :param keep_non_locked_flag: flag that states whether non-locked tasks should be kept
    :return: list of (vid, prq, expected_pu_time, driving_distance, reservation_flag) tuples
    """
    prq_return_list = []
    o_pos, t_pu_earliest, t_pu_latest = prq.get_o_stop_info()
    for veh_av_pos, _, tt, td in route_infos:
        for vid, veh_av_infos in av_dict[veh_av_pos].items():
            veh_obj = fctrl.sim_vehicles[vid]
            if vid in blocked_cars or veh_obj.status == 5:
                continue
            if fctrl.get_vid_reservation_list(vid):
                reservation_flag = True
            else:
                reservation_flag = False
            last_time, last_soc = veh_av_infos
            expected_pu_time = last_time + tt
            if last_soc - veh_obj.compute_soc_consumption(td) >= fctrl.min_aps_soc:
                if not prq.get_reservation_flag() and t_pu_earliest <= expected_pu_time <= t_pu_latest:
                    prq_return_list.append((vid, prq, expected_pu_time, td, reservation_flag, keep_non_locked_flag))
                elif prq.get_reservation_flag() and expected_pu_time <= t_pu_latest:
                    prq_return_list.append((vid, prq, expected_pu_time, td, reservation_flag, keep_non_locked_flag))
    # initial RV heuristic no inertial heuristic for reservation requests
    number_rv = fctrl.rv_heuristics.get(G_RA_MAX_VR, 0)
    if only_nn:
        number_rv = 1
    if not prq.get_reservation_flag() and number_rv > 0:
        sorted_return_list = sorted(prq_return_list, key=lambda x: x[2])
        prq_return_list = sorted_return_list[:number_rv]
    return prq_return_list


def task_insertion(rv_return_list, sim_time, fctrl):
    """This function creates the task insertions for both reservation and hailing requests.

    :param rv_return_list: list of (vid, prq, expected_pu_time, distance, reservation_flag) tuples
    :param sim_time: current simulation time
    :param fctrl: FleetControl instance
    :return: list of VehiclePlans
    :rtype: list
    """
    # create HailingPlans
    list_new_plan_dcfv = []
    for (vid, prq, expected_pu_time, dist, reservation_flag, keep_non_locked_flag) in rv_return_list:
        veh_obj = fctrl.sim_vehicles[vid]
        current_assignment = fctrl.veh_plans[vid]
        orig_cfv = current_assignment.get_utility()
        if orig_cfv is None:
            orig_cfv = fctrl.compute_VehiclePlan_utility(sim_time, veh_obj, current_assignment)

        # compare utility value of plans if there is more than one feasible (probably never happens, but still)
        def delta_objective_function(veh_plan):
            return fctrl.compute_VehiclePlan_utility(sim_time, veh_obj, veh_plan) - orig_cfv

        task_dict = {G_FCTRL_PRQ: prq, G_VR_LOCKED: False}
        new_task_lists = []
        new_as_list = [HailingTask(vid, task_dict)]
        if reservation_flag:
            keep_constant, res_tasks = current_assignment.split_reservation_tasks(keep_non_locked_flag)
            # test a task insertion in all possible places of reservation tasks

            for insert_pos in range(len(res_tasks) + 1):
                new_task_lists.append(keep_constant + res_tasks[:insert_pos] + new_as_list + res_tasks[insert_pos:])
        else:
            if keep_non_locked_flag:
                new_task_lists.append(current_assignment.get_ordered_task_list() + new_as_list)
            else:
                new_task_lists.append(current_assignment.return_locked_tasks() + new_as_list)
        #
        vid_plan_dcfv_list = []  # (VehPlan, delta_cfv)
        for new_tl in new_task_lists:
            new_veh_plan = HailingVehiclePlan(veh_obj, sim_time, fctrl.routing_engine, fctrl.const_bt, new_tl)
            if new_veh_plan == fctrl.veh_plans[veh_obj.vid]:
                keep = True
            else:
                keep = False
            if new_veh_plan.update_tt_and_check_plan(veh_obj, sim_time, fctrl.routing_engine, keep_feasible=keep):
                delta_cfv = delta_objective_function(new_veh_plan)
                vid_plan_dcfv_list.append((new_veh_plan, delta_cfv))

        vid_plan_dcfv_list = sorted(vid_plan_dcfv_list, key=lambda x: x[1])[:1]
        list_new_plan_dcfv.extend(vid_plan_dcfv_list)
    # final RV heuristic, which can also be applied for reservation requests
    number_rv = fctrl.rv_heuristics.get(G_RA_MAX_VR, 0)
    if number_rv > 0:
        list_new_plans = [x[0] for x in sorted(list_new_plan_dcfv, key=lambda x: x[1])[:number_rv]]
    else:
        list_new_plans = [x[0] for x in sorted(list_new_plan_dcfv, key=lambda x: x[1])]
    return list_new_plans
