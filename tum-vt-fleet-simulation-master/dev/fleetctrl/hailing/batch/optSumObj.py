"""
The following classes and methods can be used to solve Assignment Problems of the form
min. sum_{ij} c(x_ij) x_ij
s.t. sum_i x_ij <= 1
     sum_j x_ij <= 1
where i is the vehicle index and j is the customer index.

These classes can be applied if each possible assignment only contains one unlocked customer. Otherwise, they will
throw an Error.

The objective function should contain penalty terms for not assigning customers (with extra penalty for previously
assigned customers in case re-assignments are allowed). The objective should already be evaluated for each
possible x_ij before calling these classes.

Mind: imports should be function dependent (gurobi, cplex, tensorflow, ...) in order to guarantee compatibility on
      different computer systems.
"""
# import os
from dev.fleetctrl.hailing.batch.optAssignments import OptimizedAssignment


class GurobiMinimizeSumObjective(OptimizedAssignment):
    def __init__(self, v_ass_obj_dict, sim_time, fctrl, n_cpu=1, write_model=False, write_output=False):
        """This method is used to initialize the optimization problem, so that it can be solved in the next step.

        :param v_ass_obj_dict: {}: vid -> assignment contains costs directly related to assignment c(x_ij)
        :param sim_time: current simulation time
        :param fctrl: Hailing fleet control class; can offer further information, e.g. about vehicle positions or zone
                      forecasts, or access to the routing engine
        :param n_cpu: number of cpus the optimization process can use
        :param write_model: saves optimization model to file
        :param write_output: saves optimization solution to file
        """
        super().__init__(v_ass_obj_dict, sim_time, fctrl)
        import gurobipy

        # Build dictionaries
        # ------------------
        # k2assgn: k -> assignment
        self.k2assgn = {}
        # i2k : i -> list of k containing veh with vid i
        i2k = {}
        # j2k : j -> list of k containing rq with rid j
        j2k = {}
        # counter of variables k
        k_counter = 0
        # k2i : k -> i (by assignment.vid)
        # k2j : k -> list of j (by assignment.return_non_locked_rids())
        # c   : k -> objective function value of assignment k (by assignment.get_utility())
        for vid, list_poss_assignment in v_ass_obj_dict.items():
            for poss_assignment in list_poss_assignment:
                self.k2assgn[k_counter] = poss_assignment
                try:
                    i2k[vid].append(k_counter)
                except KeyError:
                    i2k[vid] = [k_counter]
                for rid in poss_assignment.return_non_locked_rids():
                    try:
                        j2k[rid].append(k_counter)
                    except KeyError:
                        j2k[rid] = [k_counter]
                k_counter += 1
        self.number_var = k_counter

        if self.number_var > 0:
            # Build optimization model
            # ------------------------
            model_name = f"grb_hailing_{sim_time}"
            self.m = gurobipy.Model(model_name)
            self.m.setParam('OutputFlag', False)
            self.m.setParam(gurobipy.GRB.param.Threads, n_cpu)
            self.m.setObjective(gurobipy.GRB.MINIMIZE)
            # decision variables, use objective function structure as sum of components
            z = [0 for _ in range(k_counter)]
            for k in range(k_counter):
                obj_contribution = self.k2assgn[k].get_utility()
                z[k] = self.m.addVar(vtype=gurobipy.GRB.CONTINUOUS, obj=obj_contribution, name=f'z[{k}]')
            # vehicle constraints
            for vid, list_k_of_veh in i2k.items():
                self.m.addConstr(gurobipy.quicksum(z[k] for k in list_k_of_veh) <= 1, name=f"vid {vid}")
            # request constraints
            for rid, list_k_of_rq in j2k.items():
                self.m.addConstr(gurobipy.quicksum(z[k] for k in list_k_of_rq) <= 1, name=f"rid {rid}")
            self.m.update()
            # write_model = False
            # if write_model:
                # record model and solution
                # TODO # set output dir
                # output_dir = None
                # model_f = os.path.join(output_dir, f"70_{sim_time}_hailing_model.lp")
                # model_f = os.path.join(output_dir, f"70_{sim_time}_hailing_model.lp")
                # self.m.write(model_f)

    def solve(self):
        """This method solves the optimization problem posed in the init method.

        :return: {}: vid -> new assignment
        """
        if self.number_var == 0:
            return {}
        import gurobipy
        self.m.optimize()

        # retrieve assignments
        # --------------------
        assignments = {}
        if self.m.status == gurobipy.GRB.Status.OPTIMAL:
            # record solution
            # self.write_output = False
            # if self.write_output:
                # TODO # set output dir
                # output_dir = None
                # sol_f = os.path.join(output_dir, f"70_{self.sim_time}_hailing_solution.sol")
                # self.m.write(sol_f)
            #
            solution = [var.X for var in self.m.getVars()]
            for k in range(self.number_var):
                if solution[k] >= 0.99:
                    vid = self.k2assgn[k].vid
                    assignments[vid] = self.k2assgn[k]
        else:
            raise Exception(f"Operator {self.fctrl.op_id}: No Optimal Solution!")
        return assignments


class CPLEXMinimizeSumObjective(OptimizedAssignment):
    def __init__(self, v_ass_obj_dict, sim_time, fctrl, n_cpu=1, write_model=False, write_output=False):
        """This method is used to initialize the optimization problem, so that it can be solved in the next step.

        :param v_ass_obj_dict: {}: vid -> assignment contains costs directly related to assignment c(x_ij)
        :param sim_time: current simulation time
        :param fctrl: Hailing fleet control class; can offer further information, e.g. about vehicle positions or zone
                      forecasts, or access to the routing engine
        :param n_cpu: number of cpus the optimization process can use
        :param write_model: saves optimization model to file
        :param write_output: saves optimization solution to file
        """
        super().__init__(v_ass_obj_dict, sim_time, fctrl)
        try:
            from docplex.mp.model import Model
        except:
            raise IOError(f"CPLEX/docplex cannot be imported!")
        try:
            import numpy as np
        except:
            raise IOError(f"numpy cannot be imported!")

        # Build dictionaries
        # ------------------
        # k2assgn: k -> assignment
        self.k2assgn = {}
        # i2k : i -> list of k containing veh with vid i
        i2k = {}
        # j2k : j -> list of k containing rq with rid j
        j2k = {}
        # counter of variables k
        k_counter = 0
        # k2i : k -> i (by assignment.vid)
        # k2j : k -> list of j (by assignment.return_non_locked_rids())
        # c   : k -> objective function value of assignment k (by assignment.get_utility())
        for vid, list_poss_assignment in v_ass_obj_dict.items():
            for poss_assignment in list_poss_assignment:
                self.k2assgn[k_counter] = poss_assignment
                try:
                    i2k[vid].append(k_counter)
                except KeyError:
                    i2k[vid] = [k_counter]
                for rid in poss_assignment.return_non_locked_rids():
                    try:
                        j2k[rid].append(k_counter)
                    except KeyError:
                        j2k[rid] = [k_counter]
                k_counter += 1
        self.number_var = k_counter

        if self.number_var > 0:
            # Build optimization model
            # ------------------------
            cList = []
            for a in self.k2assgn:
                cList.append(self.k2assgn[a].get_utility())
            # optimization problem
            # --------------------
            self.model = Model(name='st_ua_optimization')
            self.model.parameters.threads(n_cpu)
            # decision variables - use fact that the problem is unimodular (use continuous variables)
            self.z = [0 for _ in range(k_counter)]
            # TODO # is it always unimodular?
            for k in range(k_counter):
                self.z[k] = self.model.binary_var(name=f'z[{k}]')
            self.Z = np.array(self.z)
            self.C = np.array(cList)
            # constraints
            for list_k_of_veh in i2k.values():
                self.model.add_constraint(np.sum(self.z[k] for k in list_k_of_veh) <= 1)
            for list_k_of_rq in j2k.values():
                self.model.add_constraint(np.sum(self.z[k] for k in list_k_of_rq) <= 1)

    def solve(self):
        """This method solves the optimization problem posed in the init method.

        :return: {}: vid -> new assignment
        """
        if self.number_var == 0:
            return {}
        self.model.minimize(self.Z.dot(self.C))
        solution = self.model.solve()
        # record model and solution
        # model_f = os.path.join(operator.scenario_output_dir, f"70_{current_time}_st_ua_optimization_model.lp")
        # model.write(model_f)
        # retrieve assignments
        # --------------------
        assignments = {}
        for i in range(self.number_var):
            if solution._get_var_value(self.z[i]):
                vid = self.k2assgn[i].vid
                assignments[vid] = self.k2assgn[i]
        return assignments


def return_sum_objective_solver(solver_key):
    if solver_key == "Gurobi":
        return GurobiMinimizeSumObjective
    elif solver_key == "CPLEX":
        return CPLEXMinimizeSumObjective
        #raise IOError(f"CPLEX SumObjective not yet implemented!")
    else:
        raise IOError(f"Could not resolve SumObjective solver key {solver_key}")
