def get_nearest_vehicle(fctrl, prq, sim_time, blocked_vehicles=None):
    """This function searches the nearest vehicle for a request in the hailing fleet control classes.

    :param fctrl: reference to hailing fleet control; fctrl.considered_veh determines, which vehicles are considered
    :param prq: PlanRequest, for which next vehicle should be found
    :param sim_time: current simulation time
    :param blocked_vehicles: list of cars excluded from search or None
    :return: best_veh_obj, best_pu_t, best_veh_obj_old_av_pos
    """
    # TODO # 1) remove functionality and instead use successor of searchAll with return list of length 1
    # TODO # 2) change function calls to new function
    if blocked_vehicles is None:
        blocked_vehicles = []

    # routing X -> pu
    o_pos, t_pu_earliest, t_pu_latest = prq.get_o_stop_info()
    route_infos = fctrl.routing_engine.return_travel_costs_Xto1(fctrl.considered_veh.keys(), prq.o_pos,
                                                                max_cost_value=t_pu_latest - sim_time)
    # choice of fastest vehicle to serve customer
    best_pu_t = t_pu_latest
    best_veh_obj = None
    best_veh_obj_old_av_pos = None
    for veh_av_pos, _, tt, td in route_infos:
        for vid, veh_av_infos in fctrl.considered_veh[veh_av_pos].items():
            if vid in blocked_vehicles:
                continue
            veh_obj = fctrl.sim_vehicles[vid]
            last_time, last_soc = veh_av_infos
            if last_time < sim_time:
                last_time = sim_time
            if last_soc - veh_obj.compute_soc_consumption(td) >= fctrl.min_aps_soc:
                if fctrl.get_vid_reservation_list(vid):
                    # TODO # check for feasibility with reservations
                    pass
                veh_pu_t = last_time + tt
                if veh_pu_t < best_pu_t and veh_pu_t <= t_pu_latest:
                    best_pu_t = veh_pu_t
                    best_veh_obj = veh_obj
                    best_veh_obj_old_av_pos = veh_av_pos
    return best_veh_obj, best_pu_t, best_veh_obj_old_av_pos


def get_nearest_request(fctrl, vid, list_new_prq, sim_time):
    """This function searches the nearest request for a vehicle in the hailing fleet control classes.

    :param fctrl: reference to hailing fleet control
    :param vid: vehicle id
    :param list_new_prq: list of PlanRequests to consider for matching
    :param sim_time: current simulation time
    :return: prq
    """
    # TODO # get_nearest_request()
    pass
