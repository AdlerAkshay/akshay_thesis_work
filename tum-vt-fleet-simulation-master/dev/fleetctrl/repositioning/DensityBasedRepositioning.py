import numpy as np
import os
import random
import traceback
import logging
from src.fleetctrl.repositioning.RepositioningBase import RepositioningBase
from src.misc.globals import *
LOG = logging.getLogger(__name__)

TIME_LIMIT = 120
LARGE_INT = 100000
WRITE_SOL = False
WRITE_PROBLEM = False


def gurobi_min_transport_problem(list_zones, delta_i_z_1plus, delta_i_z_1minus, sim_time, distance_cost,
                                 od_travel_info, output_dir, n_cpu, op_id):
    import gurobipy
    number_regions = len(list_zones)
    alpha_od = np.zeros((number_regions, number_regions))
    od_reposition_trips = []
    model = gurobipy.Model("MinTransportProblem")
    model.setParam('OutputFlag', False)
    model.setParam(gurobipy.GRB.param.Threads, n_cpu)
    # if self.optimisation_timeout:
    #     model.setParam('TimeLimit', self.optimisation_timeout)
    model.setParam('TimeLimit', TIME_LIMIT)
    model.setObjective(gurobipy.GRB.MINIMIZE)
    #
    j = 1
    beta_od_j = {}  # (o,d,j) -> var
    solution_var_list = []
    for z in list_zones:
        for d in list_zones:
            test_od = od_travel_info.get((z, d))
            # only create variables if t_od <= time horizon step
            if test_od is not None:
                t_od, d_od = od_travel_info.get((z, d))
                beta_od_j[(z, d, j)] = model.addVar(vtype=gurobipy.GRB.CONTINUOUS, name=f'beta_{z}_{d}_{j}',
                                                    obj=distance_cost * d_od)
                solution_var_list.append((z, d, j))
    #
    model.update()
    for z in list_zones:
        beta_in_1 = [(o, z, 1) for o in list_zones if (o, z, 1) in beta_od_j.keys()]
        beta_out_1 = [(z, d, 1) for d in list_zones if (z, d, 1) in beta_od_j.keys()]
        model.addConstr(gurobipy.quicksum(beta_od_j[oz1] for oz1 in beta_in_1) == delta_i_z_1plus[z])
        model.addConstr(gurobipy.quicksum(beta_od_j[zd1] for zd1 in beta_out_1) == delta_i_z_1minus[z])

    if output_dir and WRITE_PROBLEM:
        model_f = os.path.join(output_dir, f"70_repo_min_transport_problem_{sim_time}.lp")
        model.write(model_f)

    # solve problem
    model.update()
    model.optimize()

    # parse solution
    try:
        if model.status == "TIME_LIMIT":
            LOG.info(f"Repositioning optimization at time {sim_time} above time limit of {TIME_LIMIT} seconds!")
        # record solution
        if output_dir and WRITE_SOL:
            sol_f = os.path.join(output_dir, f"70_repo_lts_optimization_solution_{sim_time}.sol")
            model.write(sol_f)
        for odj in solution_var_list:
            o, d, _ = odj
            odj_solution = beta_od_j[odj].X
            round_solution_integer = int(np.round(odj_solution))
            if round_solution_integer > 0:
                o_region = list_zones[o]
                d_region = list_zones[d]
                alpha_od[o, d] = round_solution_integer
                od_reposition_trips.extend([(o_region, d_region)] * round_solution_integer)
    except:
        if output_dir and not WRITE_PROBLEM:
            model_f = os.path.join(output_dir, f"70_repo_lts_optimization_model_{sim_time}.lp")
            model.write(model_f)
        LOG.warning(f"Operator {op_id}: Problem in repositioning: problem formulation "
                    f"infeasible! model status:{model.status}\n{traceback.format_exc()}\ncontinueing without"
                    f"repositioning")
    return alpha_od, od_reposition_trips


class LinearWeightedSumRepositioning(RepositioningBase):
    """This class implements the Multi-Horizon Linear Weighted Sum Repositioning Strategy from PhD thesis by
    Florian Dandl."""
    def __init__(self, fleetctrl, operator_attributes, dir_names):
        """Initialization of repositioning class.

        :param fleetctrl: FleetControl class
        :param operator_attributes: operator dictionary that can contain additionally required parameters
        :param dir_names: directory structure dict
        :param solver: solver for optimization problems
        """
        super().__init__(fleetctrl, operator_attributes, dir_names)
        self.max_repo_time = self.list_horizons[1] - self.list_horizons[0]
        self.distance_cost = np.mean([veh_obj.distance_cost for veh_obj in fleetctrl.sim_vehicles])/1000
        self.zone_corr_matrix = self._return_zone_imbalance_np_array()
        self.gamma = operator_attributes.get(G_OP_REPO_GAMMA, 1.0)
        self.exp_trip_profit = operator_attributes[G_OP_REPO_EXP_TP]
        self.supply_fc_type = operator_attributes.get(G_OP_FC_SUPPLY)
        self.consider_tt = False
        self.consider_travel_costs = True
        self.quadratic_deficit = False
        self.quadratic_stack = False
        self.QBT = 1

    def determine_and_create_repositioning_plans(self, sim_time, lock=None):
        """This method determines and creates new repositioning plans. The repositioning plans are directly assigned
        to the vehicles.
        In order to allow further database processes, the vids of vehicles with new plans are returned.

        :param sim_time: current simulation time
        :param lock: indicates if vehplans should be locked
        :return: list[vid] of vehicles with changed plans
        """
        self.sim_time = sim_time
        if lock is None:
            lock = self.lock_repo_assignments
        orig_zone_imbalances = {}  # zone -> time -> imbalance without new repositioning vehicles
        list_zones_all = self.zone_system.get_complete_zone_list()
        #  TODO # extra treatment of this zone
        list_zones = sorted([zone for zone in list_zones_all if zone != -1])
        list_time_steps = []

        # 1) get required forecasts for step 1
        # ------------------------------------
        # -> supply side forecast from current system state
        t0 = sim_time + self.list_horizons[0]
        t1 = sim_time + self.list_horizons[1]
        demand_fc_dict = self._get_demand_forecasts(t0, t1)
        cplan_arrival_idle_dict = self._get_current_veh_plan_arrivals_and_repo_idle_vehicles(t0, t1)
        number_idle_vehicles = {k: len(v[2]) for k, v in cplan_arrival_idle_dict.items()}
        if self.supply_fc_type == "idle_plus_trip_fc":
            supply_fc_dict = self._get_historic_arrival_forecasts(t0, t1)
            # supply forecast based on idle vehicles and arrival forecast
            for zone in list_zones:
                orig_zone_imbalances[(zone, 1)] = number_idle_vehicles.get(zone, 0) + \
                                                  supply_fc_dict.get(zone, 0) * self.zone_sharing_rates.get(zone, 1) - \
                                                  demand_fc_dict.get(zone, 0) * self.zone_sharing_rates.get(zone, 1)
        else:
            # supply forecast based on current state
            # beware: number_current_own_vehicles include idle vehicle
            number_idle_plus_customer_arrival = {k: v[0] for k, v in cplan_arrival_idle_dict.items()}
            vehicles_repo_to_zone = {k: len(v[1]) for k,v in cplan_arrival_idle_dict.items()}
            for zone in list_zones:
                orig_zone_imbalances[(zone, 1)] = number_idle_plus_customer_arrival.get(zone, 0) + \
                                                  vehicles_repo_to_zone.get(zone, 0) - \
                                                  demand_fc_dict.get(zone, 0) * self.zone_sharing_rates.get(zone, 1)
        list_time_steps.append(1)

        # 2) get required forecasts for other steps
        # -----------------------------------------
        # -> supply side forecast from expected arrivals
        for j in range(2, len(self.list_horizons)):
            tj_start = sim_time + self.list_horizons[j-1]
            tj_end = sim_time + self.list_horizons[j]
            supply_fc_dict = self._get_historic_arrival_forecasts(tj_start, tj_end)
            demand_fc_dict = self._get_demand_forecasts(tj_start, tj_end)
            for zone in list_zones:
                orig_zone_imbalances[(zone, j)] = supply_fc_dict.get(zone, 0) * self.zone_sharing_rates.get(zone, 1) - \
                                                  demand_fc_dict.get(zone, 0) * self.zone_sharing_rates.get(zone, 1)
            list_time_steps.append(j)

        # 3) set up and solve optimization problem
        # ----------------------------------------
        # output for debug
        output_dir = self.fleetctrl.dir_names[G_DIR_OUTPUT]
        # output_dir = None
        if self.solver_key == "Gurobi":
            alpha_od, od_reposition_trips = self._optimization_gurobi(sim_time, list_zones, list_time_steps,
                                                                      orig_zone_imbalances, number_idle_vehicles,
                                                                      self.max_repo_time, self.distance_cost,
                                                                      self.zone_corr_matrix, self.gamma,
                                                                      self.exp_trip_profit, output_dir)
        else:
            raise NotImplementedError(f"Optimization problem in {self.__class__.__name__} only implemented for Gurobi!")

        # 4) creating vehicle trip assignments out of alpha_od
        # ----------------------------------------------------
        list_veh_with_changes = []
        if od_reposition_trips:
            # create assignments
            # ------------------
            random.seed(sim_time)
            random.shuffle(od_reposition_trips)
            for (origin_zone_id, destination_zone_id) in od_reposition_trips:
                list_idle_veh = cplan_arrival_idle_dict[origin_zone_id][2]
                list_veh_obj_with_repos = self._od_to_veh_plan_assignment(sim_time, origin_zone_id,
                                                                          destination_zone_id, list_idle_veh, lock=lock)
                list_veh_with_changes.extend([veh_obj.vid for veh_obj in list_veh_obj_with_repos])
                for veh_obj in list_veh_obj_with_repos:
                    cplan_arrival_idle_dict[origin_zone_id][2].remove(veh_obj)
        return list_veh_with_changes

    def _optimization_gurobi(self, sim_time, list_zones, list_time_steps, orig_zone_imbalances, number_idle_vehicles,
                             max_repo_time, distance_cost, zone_corr_matrix, gamma, exp_trip_profit, output_dir=None):
        import gurobipy
        model = gurobipy.Model("LinearWeightedSumRepositioning")
        model.setParam('OutputFlag', False)
        model.setParam(gurobipy.GRB.param.Threads, self.fleetctrl.n_cpu)
        # if self.optimisation_timeout:
        #     model.setParam('TimeLimit', self.optimisation_timeout)
        model.setParam('TimeLimit', TIME_LIMIT)
        model.setObjective(gurobipy.GRB.MINIMIZE)

        number_regions = len(list_zones)
        # setting up variables and objective
        # ----------------------------------
        squared_objective_terms = []
        #
        psi_z_j = {}    # (z,j) -> var
        Y_z_j = {}      # (z,j) -> var
        beta_od_j = {}  # (o,d,j) -> var
        I_z_j = {}          # (z,j) -> var
        I_j_vec = {}        # j -> I_j_vec [MVar]
        I_bar_z_j = {}      # (z,j) -> var
        I_bar_j_vec = {}    # j -> I_j_vec [MVar]
        solution_var_list = []  # list of (o,d,1) pairs
        zone_id_dict = {}   # zone_id -> counter
        counter = 0
        od_travel_info = {}  # (o,d) -> (t_od, d_od) only if t_od < max_repo_time
        od_red_av_f = {}     # (o,d) -> reduced_availability_factor
        for o in list_zones:
            for d in list_zones:
                if o == d:
                    continue
                t_od, d_od = self._get_od_zone_travel_info(sim_time, o, d)
                if t_od < max_repo_time:
                    od_travel_info[(o,d)] = (t_od, d_od)
                    od_red_av_f[(o,d)] = max_repo_time - t_od
        #
        for z in list_zones:
            zone_id_dict[z] = counter
            counter += 1
            for j in list_time_steps:
                if self.quadratic_deficit:
                    psi_factor = 0
                else:
                    psi_factor = (gamma**j * exp_trip_profit) / 3
                psi_z_j[(z,j)] = model.addVar(vtype=gurobipy.GRB.CONTINUOUS, name=f'psi_{z}_{j}',
                                              obj=psi_factor)
                if self.quadratic_deficit:
                    squared_objective_terms.append(psi_z_j[(z,j)] *
                                                   ((gamma**j * exp_trip_profit) / (3*self.QBT))**(1/2))
                # stack_factor = LARGE_INT
                if self.quadratic_stack:
                    stack_factor = 0
                else:
                    stack_factor = 2 * (gamma ** j * exp_trip_profit) / 3
                Y_z_j[(z,j)] = model.addVar(vtype=gurobipy.GRB.CONTINUOUS, name=f'Y_{z}_{j}',
                                            obj=stack_factor)
                if self.quadratic_stack:
                    squared_objective_terms.append(Y_z_j[(z,j)] *
                                                   (2 * (gamma ** j * exp_trip_profit) / (3*self.QBT))**(1/2))
                for d in list_zones:
                    test_od = od_travel_info.get((z,d))
                    # only create variables if t_od <= time horizon step
                    if test_od is not None:
                        t_od, d_od = od_travel_info.get((z,d))
                        if self.consider_travel_costs:
                            dist_cost_factor = distance_cost * d_od
                        else:
                            dist_cost_factor = 1
                        beta_od_j[(z, d, j)] = model.addVar(vtype=gurobipy.GRB.CONTINUOUS, name=f'beta_{z}_{d}_{j}',
                                                            obj=dist_cost_factor)
                        if j == 1:
                            solution_var_list.append((z,d,j))
        # special treatment of I as it requires matrix multiplication
        for j in list_time_steps:
            I_j_vec[j] = model.addMVar((number_regions,), vtype=gurobipy.GRB.CONTINUOUS, lb=-gurobipy.GRB.INFINITY)
            # MVar.tolist() required to access variables (BEWARE: requires Gurobi Version > 9.1)
            tmp_list = I_j_vec[j].tolist()
            for z in list_zones:
                array_index = zone_id_dict[z]
                tmp_list[array_index].setAttr("VarName", "I_j_z")
                I_z_j[(z, j)] = tmp_list[array_index]
            I_bar_j_vec[j] = model.addMVar((number_regions,), vtype=gurobipy.GRB.CONTINUOUS, lb=-gurobipy.GRB.INFINITY)
            # required to access variables (and not part of numpy array)
            tmp_list = I_bar_j_vec[j].tolist()
            for z in list_zones:
                array_index = zone_id_dict[z]
                tmp_list[array_index].setAttr("VarName", "I_bar_j_z")
                I_bar_z_j[(z, j)] = tmp_list[array_index]

        model.update()
        # build objective manually if squared terms are involved
        if squared_objective_terms:
            linear_obj = model.getObjective()
            new_obj = gurobipy.quicksum(sq_var_term*sq_var_term for sq_var_term in squared_objective_terms) + linear_obj
            model.setObjective(new_obj)
            model.update()

        # setting up constraints
        # ----------------------
        for z in list_zones:
            # b) imbalance in step 1
            beta_in_1 = [(o, z, 1) for o in list_zones if (o, z, 1) in beta_od_j.keys()]
            beta_out_1 = [(z, d, 1) for d in list_zones if (z, d, 1) in beta_od_j.keys()]
            model.addConstr(orig_zone_imbalances.get((z,1), 0) +
                            gurobipy.quicksum(beta_od_j[oz1] for oz1 in beta_in_1) -
                            gurobipy.quicksum(beta_od_j[zd1] for zd1 in beta_out_1) == I_z_j[(z,1)])
            # c) imbalance in other steps
            for j in list_time_steps:
                if j == 1:
                    continue
                beta_in_j = [(o, z, j) for o in list_zones if (o, z, j) in beta_od_j.keys()]
                beta_out_j = [(z, d, j) for d in list_zones if (z, d, j) in beta_od_j.keys()]
                if self.consider_tt:
                    model.addConstr(Y_z_j[(z, j)] + orig_zone_imbalances.get((z, j), 0) +
                                    gurobipy.quicksum(beta_od_j[ozj]*od_red_av_f[(ozj[0],ozj[1])] for ozj in beta_in_j)
                                    - gurobipy.quicksum(beta_od_j[zdj] for zdj in beta_out_j) == I_z_j[(z, j)])
                else:
                    model.addConstr(Y_z_j[(z, j)] + orig_zone_imbalances.get((z, j), 0) +
                                    gurobipy.quicksum(beta_od_j[ozj] for ozj in beta_in_j) -
                                    gurobipy.quicksum(beta_od_j[zdj] for zdj in beta_out_j) == I_z_j[(z, j)])
            # d) limit of outgoing vehicles for all steps
            for j in list_time_steps:
                beta_out_j = [(z, d, j) for d in list_zones if (z, d, j) in beta_od_j.keys()]
                model.addConstr(gurobipy.quicksum(beta_od_j[zdj] for zdj in beta_out_j) <= Y_z_j[(z, j)])
            # e) stack in step 1
            model.addConstr(Y_z_j[(z, 1)] == number_idle_vehicles.get(z, 0))
            # f) stack in other steps
            for j in list_time_steps:
                if j == 1:
                    continue
                if self.consider_tt:
                    # for stack of next forecast step, vehicles have to be considered full after arriving in zone!
                    beta_in_j = [(o, z, j-1) for o in list_zones if (o, z, j) in beta_od_j.keys()]
                    model.addConstr(Y_z_j[(z, j)] >= I_z_j[(z, j - 1)] +
                                    gurobipy.quicksum(beta_od_j[ozj] * (1 - od_red_av_f[(ozj[0],ozj[1])])
                                                      for ozj in beta_in_j))
                else:
                    model.addConstr(Y_z_j[(z, j)] >= I_z_j[(z, j - 1)])
                # g) stack cannot be smaller than 0 -> already handled by definition of variable
                model.addConstr(Y_z_j[(z, j)] >= 0)
        # h) and i) and j) computation of imbalance with zone-correlations
        for j in list_time_steps:
            red_zone_corr_matrix = zone_corr_matrix[np.ix_(list_zones, list_zones)]
            # beware of matrix multiplication in python! matrix is defined like this that row sum is 1!
            model.addConstr(I_bar_j_vec[j] == I_j_vec[j] @ red_zone_corr_matrix)
            for z in list_zones:
                model.addConstr(psi_z_j[(z,j)] >= - I_bar_z_j[(z, j)])
                model.addConstr(psi_z_j[(z, j)] >= 0)
        # k) beta >= 0
        for (o, d, j) in beta_od_j.keys():
            model.addConstr(beta_od_j[(o, d, j)] >= 0)
        # update
        model.update()
        # record model and solution
        if output_dir and WRITE_PROBLEM:
            model_f = os.path.join(output_dir, f"70_repo_lws_optimization_model_{sim_time}.lp")
            model.write(model_f)
        # optimize
        model.optimize()

        # if model is infeasible or unbound: check in more detail
        # if model.status == 4:
        #     model.setParam("DualReductions", 0)
        #     model.update()
        #     model.optimize()

        # retrieve solution and create od-vehicle list
        # --------------------------------------------
        alpha_od = np.zeros((number_regions, number_regions))
        od_reposition_trips = []
        try:
            if model.status == "TIME_LIMIT":
                LOG.info(f"Repositioning optimization at time {sim_time} above time limit of {TIME_LIMIT} seconds!")
            # record solution
            if output_dir and WRITE_SOL:
                sol_f = os.path.join(output_dir, f"70_repo_lws_optimization_solution_{sim_time}.sol")
                model.write(sol_f)
            for odj in solution_var_list:
                o, d, _ = odj
                odj_solution = beta_od_j[odj].X
                round_solution_integer = int(np.floor(odj_solution))
                if round_solution_integer > 0:
                    o_region = list_zones[o]
                    d_region = list_zones[d]
                    alpha_od[o, d] = round_solution_integer
                    od_reposition_trips.extend([(o_region, d_region)] * round_solution_integer)
        except:
            if output_dir and not WRITE_PROBLEM:
                model_f = os.path.join(output_dir, f"70_repo_lws_optimization_model_{sim_time}.lp")
                model.write(model_f)
            LOG.warning(f"Operator {self.fleetctrl.op_id}: Problem in repositioning: problem formulation "
                        f"infeasible! model status:{model.status}\n{traceback.format_exc()}\ncontinueing without"
                        f"repositioning")
        return alpha_od, od_reposition_trips


class LWSReducedAvailabilityRepositioning(LinearWeightedSumRepositioning):
    """This class implements the Multi-Horizon Reduced Availability Repositioning Strategy from PhD thesis by
    Florian Dandl."""
    def __init__(self, fleetctrl, operator_attributes, dir_names):
        super().__init__(fleetctrl, operator_attributes, dir_names)
        self.consider_tt = True


class LinearTwoStepRepositioning(LinearWeightedSumRepositioning):
    """This class implements the Multi-Horizon Linear Two-Step Repositioning Strategy from PhD thesis by
    Florian Dandl."""
    def __init__(self, fleetctrl, operator_attributes, dir_names):
        super().__init__(fleetctrl, operator_attributes, dir_names)
        self.consider_travel_costs = False

    def _optimization_gurobi(self, sim_time, list_zones, list_time_steps, orig_zone_imbalances, number_idle_vehicles,
                             max_repo_time, distance_cost, zone_corr_matrix, gamma, exp_trip_profit, output_dir=None):
        alpha_od_init, _ = super()._optimization_gurobi(sim_time, list_zones, list_time_steps, orig_zone_imbalances,
                                                        number_idle_vehicles, max_repo_time, distance_cost,
                                                        zone_corr_matrix, gamma, exp_trip_profit, output_dir)
        not_trivial = False
        delta_i_z_1plus = {}    # z -> delta_i_z_1plus (vehicles driving to zone z)
        delta_i_z_1minus = {}   # z -> delta_i_z_1minus (vehicles driving from zone z)
        for z in list_zones:
            delta_i_z_1plus[z] = alpha_od_init[:,z].sum()
            delta_i_z_1minus[z] = alpha_od_init[z,:].sum()
            if delta_i_z_1minus[z] != 0 or delta_i_z_1minus != 0:
                not_trivial = True
        #
        number_regions = len(list_zones)
        alpha_od = np.zeros((number_regions, number_regions))
        od_reposition_trips = []
        if not_trivial:
            od_travel_info = {}  # (o,d) -> (t_od, d_od) only if t_od < max_repo_time
            od_red_av_f = {}  # (o,d) -> reduced_availability_factor
            for o in list_zones:
                for d in list_zones:
                    if o == d:
                        continue
                    t_od, d_od = self._get_od_zone_travel_info(sim_time, o, d)
                    if t_od < max_repo_time:
                        od_travel_info[(o, d)] = (t_od, d_od)
                        od_red_av_f[(o, d)] = max_repo_time - t_od
            #
            alpha_od, od_reposition_trips = gurobi_min_transport_problem(list_zones, delta_i_z_1plus, delta_i_z_1minus,
                                                                         sim_time, distance_cost, od_travel_info,
                                                                         output_dir, self.fleetctrl.n_cpu,
                                                                         self.fleetctrl.op_id)
        return alpha_od, od_reposition_trips


class QuadraticTwoStepRepositioning(LinearWeightedSumRepositioning):
    """This class implements the Multi-Horizon Quadratic Two-Step Repositioning Strategy from PhD thesis by
    Florian Dandl."""
    def __init__(self, fleetctrl, operator_attributes, dir_names):
        super().__init__(fleetctrl, operator_attributes, dir_names)
        self.consider_travel_costs = False
        self.quadratic_deficit = True
        self.quadratic_stack = True

    def _optimization_gurobi(self, sim_time, list_zones, list_time_steps, orig_zone_imbalances, number_idle_vehicles,
                             max_repo_time, distance_cost, zone_corr_matrix, gamma, exp_trip_profit, output_dir=None):
        alpha_od_init, _ = super()._optimization_gurobi(sim_time, list_zones, list_time_steps, orig_zone_imbalances,
                                                        number_idle_vehicles, max_repo_time, distance_cost,
                                                        zone_corr_matrix, gamma, exp_trip_profit, output_dir)
        not_trivial = False
        delta_i_z_1plus = {}    # z -> delta_i_z_1plus (vehicles driving to zone z)
        delta_i_z_1minus = {}   # z -> delta_i_z_1minus (vehicles driving from zone z)
        for z in list_zones:
            delta_i_z_1plus[z] = alpha_od_init[:,z].sum()
            delta_i_z_1minus[z] = alpha_od_init[z,:].sum()
            if delta_i_z_1minus[z] != 0 or delta_i_z_1minus != 0:
                not_trivial = True
        #
        number_regions = len(list_zones)
        alpha_od = np.zeros((number_regions, number_regions))
        od_reposition_trips = []
        if not_trivial:
            od_travel_info = {}  # (o,d) -> (t_od, d_od) only if t_od < max_repo_time
            od_red_av_f = {}  # (o,d) -> reduced_availability_factor
            for o in list_zones:
                for d in list_zones:
                    if o == d:
                        continue
                    t_od, d_od = self._get_od_zone_travel_info(sim_time, o, d)
                    if t_od < max_repo_time:
                        od_travel_info[(o, d)] = (t_od, d_od)
                        od_red_av_f[(o, d)] = max_repo_time - t_od
            #
            alpha_od, od_reposition_trips = gurobi_min_transport_problem(list_zones, delta_i_z_1plus, delta_i_z_1minus,
                                                                         sim_time, distance_cost, od_travel_info,
                                                                         output_dir, self.fleetctrl.n_cpu,
                                                                         self.fleetctrl.op_id)
        return alpha_od, od_reposition_trips


class QuadraticDeficitLinearSurplusTwoStepRepositioning(LinearWeightedSumRepositioning):
    """This class implements the Multi-Horizon Quadratic & Linear Two-Step Repositioning Strategy from PhD thesis by
    Florian Dandl."""
    def __init__(self, fleetctrl, operator_attributes, dir_names):
        super().__init__(fleetctrl, operator_attributes, dir_names)
        self.consider_travel_costs = False
        self.quadratic_deficit = True
        self.quadratic_stack = False
        self.QBT = operator_attributes[G_OP_REPO_QBT]

    def _optimization_gurobi(self, sim_time, list_zones, list_time_steps, orig_zone_imbalances, number_idle_vehicles,
                             max_repo_time, distance_cost, zone_corr_matrix, gamma, exp_trip_profit, output_dir=None):
        alpha_od_init, _ = super()._optimization_gurobi(sim_time, list_zones, list_time_steps, orig_zone_imbalances,
                                                        number_idle_vehicles, max_repo_time, distance_cost,
                                                        zone_corr_matrix, gamma, exp_trip_profit, output_dir)
        not_trivial = False
        delta_i_z_1plus = {}    # z -> delta_i_z_1plus (vehicles driving to zone z)
        delta_i_z_1minus = {}   # z -> delta_i_z_1minus (vehicles driving from zone z)
        for z in list_zones:
            delta_i_z_1plus[z] = alpha_od_init[:,z].sum()
            delta_i_z_1minus[z] = alpha_od_init[z,:].sum()
            if delta_i_z_1minus[z] != 0 or delta_i_z_1minus != 0:
                not_trivial = True
        #
        number_regions = len(list_zones)
        alpha_od = np.zeros((number_regions, number_regions))
        od_reposition_trips = []
        if not_trivial:
            od_travel_info = {}  # (o,d) -> (t_od, d_od) only if t_od < max_repo_time
            od_red_av_f = {}  # (o,d) -> reduced_availability_factor
            for o in list_zones:
                for d in list_zones:
                    if o == d:
                        continue
                    t_od, d_od = self._get_od_zone_travel_info(sim_time, o, d)
                    if t_od < max_repo_time:
                        od_travel_info[(o, d)] = (t_od, d_od)
                        od_red_av_f[(o, d)] = max_repo_time - t_od
            #
            alpha_od, od_reposition_trips = gurobi_min_transport_problem(list_zones, delta_i_z_1plus, delta_i_z_1minus,
                                                                         sim_time, distance_cost, od_travel_info,
                                                                         output_dir, self.fleetctrl.n_cpu,
                                                                         self.fleetctrl.op_id)
        return alpha_od, od_reposition_trips
