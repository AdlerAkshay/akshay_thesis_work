from src.misc.globals import *

LARGE_INT = 1000000

ASSIGNMENT_REWARDS = {G_PRQS_IN_VEH: LARGE_INT*1000, G_PRQS_LOCKED: LARGE_INT*100, G_PRQS_ACC_OFFER: LARGE_INT*10,
                      G_PRQS_INIT_OFFER: LARGE_INT*5, G_PRQS_NO_OFFER: LARGE_INT}

SOFT_TW_REWARDS = {"locked": LARGE_INT*1000, "in time window": LARGE_INT+100}
# this basically means a solution that reduces the global waiting time by more than x*100 seconds is preferred,
# even if x requests are picked up outside of their respective (soft) time windows

# -------------------------------------------------------------------------------------------------------------------- #
# main function
# -------------
def return_hailing_objective_function(vr_control_func_dict):
    """This function generates the control objective functions for vehicle-request assignment in pooling operation.
    The control objective functions contain an assignment reward of LARGE_INT and are to be
    ---------------
    -> minimized <-
    ---------------

    :param vr_control_func_dict: dictionary which has to contain "func_key" as switch between possible functions;
            additional parameters of a function can have additional keys.
    :type vr_control_func_dict: dict
    :return: objective function
    :rtype: function
    """
    func_key = vr_control_func_dict["func_key"]

    # ---------------------------------------------------------------------------------------------------------------- #
    # control objective function definitions
    # --------------------------------------
    # from pooling
    if func_key == "total_distance":
        def control_f(simulation_time, veh_obj, veh_plan, rq_dict, routing_engine):
            """This function evaluates the driven distance according to a vehicle plan.

            :param simulation_time: current simulation time
            :param veh_obj: simulation vehicle object
            :param veh_plan: vehicle plan in question
            :param rq_dict: rq -> Plan request dictionary
            :param routing_engine: for routing queries
            :return: objective function value
            """
            assignment_reward = len(veh_plan.pax_info) * LARGE_INT
            sum_dist = 0
            last_pos = veh_obj.pos
            for ps in veh_plan.list_plan_stops:
                pos = ps.get_pos()
                if pos != last_pos:
                    sum_dist += routing_engine.return_travel_costs_1to1(last_pos, pos)[2]
                    last_pos = pos
            return sum_dist - assignment_reward

    elif func_key == "total_msp_profit":

        var_fare = vr_control_func_dict["var_fare"]
        base_fare = vr_control_func_dict["base_fare"]
        var_cost = vr_control_func_dict["var_cost"]

        def control_f(simulation_time, veh_obj, veh_plan, rq_dict, routing_engine):
            """This function evaluates the Mobility Service Provider's profit.

            :param simulation_time: current simulation time
            :param veh_obj: simulation vehicle object
            :param veh_plan: vehicle plan in question
            :param rq_dict: rq -> Plan request dictionary
            :param routing_engine: for routing queries
            :return: objective function value
            """

            empty_distance, in_veh_distance = 0, 0
            last_pos = veh_obj.pos
            for i, ps in enumerate(veh_plan.list_plan_stops):
                if ps.pos != last_pos:
                    dist = routing_engine.return_travel_costs_1to1(last_pos, ps.pos)[2]
                    if i % 2 == 0:
                        # A pickup trip
                        empty_distance += dist
                    else:
                        in_veh_distance += dist
                    last_pos = ps.pos

            assignment_reward = len(veh_plan.pax_info) * base_fare

            for rid, boarding_info_list in veh_plan.pax_info.items():
                prq = rq_dict[rid]
                if prq.is_locked() or prq.status in {G_PRQS_INIT_OFFER, G_PRQS_ACC_OFFER, G_PRQS_LOCKED, G_PRQS_IN_VEH}:
                    assignment_reward += LARGE_INT
            profit = - assignment_reward - in_veh_distance*var_fare + empty_distance*var_cost
            return profit

    elif func_key == "total_system_time":
        def control_f(simulation_time, veh_obj, veh_plan, rq_dict, routing_engine):
            """This function evaluates the total spent time of a vehicle according to a vehicle plan.

            :param simulation_time: current simulation time
            :param veh_obj: simulation vehicle object
            :param veh_plan: vehicle plan in question
            :param rq_dict: rq -> Plan request dictionary
            :param routing_engine: for routing queries
            :return: objective function value
            """
            assignment_reward = len(veh_plan.pax_info) * LARGE_INT
            # end time (for request assignment purposes) defined by arrival at last stop
            if veh_plan.list_plan_stops:
                end_time = veh_plan.list_plan_stops[-1].get_planned_arrival_and_departure_time()[1]
            else:
                end_time = simulation_time
            # utility is negative value of end_time - simulation_time
            return end_time - simulation_time - assignment_reward

    elif func_key == "distance_and_user_times":
        traveler_vot = vr_control_func_dict["vot"]

        def control_f(simulation_time, veh_obj, veh_plan, rq_dict, routing_engine):
            """This function combines the total driving costs and the value of customer time.

            :param simulation_time: current simulation time
            :param veh_obj: simulation vehicle object
            :param veh_plan: vehicle plan in question
            :param rq_dict: rq -> Plan request dictionary
            :param routing_engine: for routing queries
            :return: objective function value
            """
            assignment_reward = len(veh_plan.pax_info) * LARGE_INT
            # distance term
            sum_dist = 0
            last_pos = veh_obj.pos
            for ps in veh_plan.list_plan_stops:
                pos = ps.get_pos()
                if pos != last_pos:
                    sum_dist += routing_engine.return_travel_costs_1to1(last_pos, pos)[2]
                    last_pos = pos
            # value of time term (treat waiting and in-vehicle time the same)
            sum_user_times = 0
            for rid, boarding_info_list in veh_plan.pax_info.items():
                rq_time = rq_dict[rid].rq_time
                drop_off_time = boarding_info_list[1]
                sum_user_times += (drop_off_time - rq_time)
            # vehicle costs are taken from simulation vehicle (cent per meter)
            # value of travel time is scenario input (cent per second)
            return sum_dist * veh_obj.distance_cost + sum_user_times * traveler_vot - assignment_reward

    elif func_key == "distance_and_user_times_with_walk":
        traveler_vot = vr_control_func_dict["vot"]

        def control_f(simulation_time, veh_obj, veh_plan, rq_dict, routing_engine):
            """This function combines the total driving costs and the value of customer time.

            :param simulation_time: current simulation time
            :param veh_obj: simulation vehicle object
            :param veh_plan: vehicle plan in question
            :param rq_dict: rq -> Plan request dictionary
            :param routing_engine: for routing queries
            :return: objective function value
            """
            assignment_reward = len(veh_plan.pax_info) * LARGE_INT
            # distance term
            sum_dist = 0
            last_pos = veh_obj.pos
            for ps in veh_plan.list_plan_stops:
                pos = ps.get_pos()
                if pos != last_pos:
                    sum_dist += routing_engine.return_travel_costs_1to1(last_pos, pos)[2]
                    last_pos = pos
            # value of time term (treat waiting and in-vehicle time the same)
            sum_user_times = 0
            for rid, boarding_info_list in veh_plan.pax_info.items():
                rq_time = rq_dict[rid].rq_time
                walking_time_end = rq_dict[rid].walking_time_end    #walking time start allready included in interval rq-time -> drop_off_time
                drop_off_time = boarding_info_list[1]
                sum_user_times += (drop_off_time - rq_time) + walking_time_end
            # vehicle costs are taken from simulation vehicle (cent per meter)
            # value of travel time is scenario input (cent per second)
            return sum_dist * veh_obj.distance_cost + sum_user_times * traveler_vot - assignment_reward

    elif func_key == "distance_and_user_vehicle_times":
        traveler_vot = vr_control_func_dict["vot"]

        def control_f(simulation_time, veh_obj, veh_plan, rq_dict, routing_engine):
            """This function combines the total driving costs, the value of customer time and vehicle waiting time.

            :param simulation_time: current simulation time
            :param veh_obj: simulation vehicle object
            :param veh_plan: vehicle plan in question
            :param rq_dict: rq -> Plan request dictionary
            :param routing_engine: for routing queries
            :return: objective function value
            """
            assignment_reward = len(veh_plan.pax_info) * LARGE_INT
            # distance term
            sum_dist = 0
            sum_veh_wait = 0
            last_pos = veh_obj.pos
            for ps in veh_plan.list_plan_stops:
                # penalize VehiclePlan if it is not planned through and raise warning
                planned_arrival_time, departure_time = ps.get_planned_arrival_and_departure_time()
                pos = ps.get_pos()
                if planned_arrival_time is None:
                    assignment_reward = -len(veh_plan.pax_info) * LARGE_INT
                else:
                    # compute vehicle stop time if departure is already planned
                    if departure_time is not None:
                        veh_wait_time = departure_time - planned_arrival_time
                        if veh_wait_time > 0:
                            sum_veh_wait += veh_wait_time
                if pos != last_pos:
                    sum_dist += routing_engine.return_travel_costs_1to1(last_pos, pos)[2]
                    last_pos = pos
            # value of time term (treat waiting and in-vehicle time the same)
            sum_user_times = 0
            for rid, boarding_info_list in veh_plan.pax_info.items():
                rq_time = rq_dict[rid].rq_time
                drop_off_time = boarding_info_list[1]
                sum_user_times += (drop_off_time - rq_time)
            # vehicle costs are taken from simulation vehicle (cent per meter)
            # value of travel time is scenario input (cent per second)
            return sum_dist * veh_obj.distance_cost + (sum_user_times + sum_veh_wait) * traveler_vot - assignment_reward

    # ---------------------------------------------------------------------------------------------------------------- #
    # specific for hailing
    elif func_key == "prioritized_unlocked_wait_times":
        def control_f(simulation_time, veh_obj, veh_plan, rq_dict, routing_engine):
            """This function tries to minimize the waiting time of unlocked users. Moreover, it uses different
            penalties for requests with different status.

            :param simulation_time: current simulation time
            :param veh_obj: simulation vehicle object
            :param veh_plan: vehicle plan in question
            :param rq_dict: rq -> Plan request dictionary
            :param routing_engine: for routing queries
            :return: objective function value
            """
            sum_user_wait_times = 0
            assignment_reward = 0
            for rid, boarding_info_list in veh_plan.pax_info.items():
                prq = rq_dict[rid]
                if prq.pu_time is None:
                    rq_time = rq_dict[rid].rq_time
                    pick_up_time = boarding_info_list[0]
                    sum_user_wait_times += (pick_up_time - rq_time)
                    '''
                    if prq.is_locked():
                        assignment_reward += ASSIGNMENT_REWARDS["locked"]
                    else:
                        assignment_reward += ASSIGNMENT_REWARDS.get(prq.status, LARGE_INT)
                    '''
                    # TODO # should be correct this way
                    assignment_reward += ASSIGNMENT_REWARDS.get(prq.status, LARGE_INT)
            return sum_user_wait_times - assignment_reward

    elif func_key == "IRS_study_standard":
        def control_f(simulation_time, veh_obj, veh_plan, rq_dict, routing_engine):
            """This function tries to minimize the waiting time of unlocked users.

            :param simulation_time: current simulation time
            :param veh_obj: simulation vehicle object
            :param veh_plan: vehicle plan in question
            :param rq_dict: rq -> Plan request dictionary
            :param routing_engine: for routing queries
            :return: objective function value
            """
            sum_user_wait_times = 0
            assignment_reward = 0
            sum_dist = 0
            last_pos = veh_obj.pos
            for ps in veh_plan.list_plan_stops:
                pos = ps.get_pos()
                if pos != last_pos and len(pos.get_list_boarding_rids()) > 0:
                    sum_dist += routing_engine.return_travel_costs_1to1(last_pos, pos)[2]
                    last_pos = pos
            for rid, boarding_info_list in veh_plan.pax_info.items():
                prq = rq_dict[rid]
                if prq.pu_time is None:
                    rq_time = rq_dict[rid].rq_time
                    pick_up_time = boarding_info_list[0]
                    sum_user_wait_times += (pick_up_time - rq_time)
                    if prq.is_locked():
                        assignment_reward += LARGE_INT*10000
                    elif prq.status < G_PRQS_LOCKED:
                        assignment_reward += LARGE_INT
                    else:
                        assignment_reward += LARGE_INT*100
            # 4 is the empirically found parameter to weigh saved dist against saved waiting time
            return sum_dist + sum_user_wait_times - assignment_reward

    elif func_key == "soft_time_windows":
        def control_f(simulation_time, veh_obj, veh_plan, rq_dict, routing_engine):
            """This function tries to minimize the waiting time of unlocked users. It penalizes assignments that imply
            pickups outside of the respective requests' time windows.

            :param simulation_time: current simulation time
            :param veh_obj: simulation vehicle object
            :param veh_plan: vehicle plan in question
            :param rq_dict: rq -> Plan request dictionary
            :param routing_engine: for routing queries
            :return: objective function value
            """
            sum_user_wait_times = 0
            assignment_reward = 0
            sum_dist = 0
            last_pos = veh_obj.pos
            for ps in veh_plan.list_plan_stops:
                pos = ps.get_pos()
                if pos != last_pos:
                    sum_dist += routing_engine.return_travel_costs_1to1(last_pos, pos)[2]
                    last_pos = pos
            for rid, boarding_info_list in veh_plan.pax_info.items():
                prq = rq_dict[rid]
                if prq.pu_time is None:
                    rq_time = rq_dict[rid].rq_time
                    pick_up_time = boarding_info_list[0]
                    _, t_pu_earliest, t_pu_latest = rq_dict[rid].get_soft_o_stop_info()
                    sum_user_wait_times += (pick_up_time - rq_time)
                    if prq.is_locked():
                        assignment_reward += SOFT_TW_REWARDS["locked"]
                    elif t_pu_earliest <= pick_up_time <= t_pu_latest:
                        assignment_reward += SOFT_TW_REWARDS["in time window"]
                    else:
                        assignment_reward += LARGE_INT
            return sum_dist + sum_user_wait_times - assignment_reward

    # elif func_key == "":
    #     def control_f(simulation_time, veh_obj, veh_plan, rq_dict, routing_engine):
    #         """This function ...
    #
    #         :param simulation_time: current simulation time
    #         :param veh_obj: simulation vehicle object
    #         :param veh_plan: vehicle plan in question
    #         :param rq_dict: rq -> Plan request dictionary
    #         :param routing_engine: for routing queries
    #         :return: objective function value
    #         """
    #         pass

    else:
        raise IOError(f"Did not find valid request assignment control objective string."
                      f" Please check the input parameter {G_OP_VR_CTRL_F}!")

    return control_f
