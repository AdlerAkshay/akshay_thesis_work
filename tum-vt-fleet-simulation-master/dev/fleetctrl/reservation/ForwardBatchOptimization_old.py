import numpy as np
import time
from src.fleetctrl.FleetControlBase import FleetControlBase
from dev.fleetctrl.reservation.RevelationHorizonBase import RevelationHorizonBase
from dev.fleetctrl.reservation.misc.RequestGroup import QuasiVehiclePlan
from dev.fleetctrl.reservation.ReservationRequestBatch import ReservationRequestBatch
from src.misc.globals import *

from typing import Any, Dict, List, Tuple

import logging
LOG = logging.getLogger(__name__)

NEW_VID_PENALTY = 1000000   # penalty for introducing a new vehicle in case a match between batches is not possible

def batch_requests(sorted_requests, number_requests_in_batch):
    """ this function is used to created request batches with fixed batch sizes according to their
    sorting order
    :param sorted_requests: sorted list of plan request objects
    :param number_requests_in_batch: int number of number objects per batch
    :return: list of list of request object batches. the length of the inner list does not exceed number_requests_in_batch"""
    batch_list = []
    current_batch = []
    for i, rq in enumerate(sorted_requests):
        if i != 0 and i % number_requests_in_batch == 0:
            batch_list.append(current_batch)
            current_batch = []
        current_batch.append(rq)
    if len(current_batch) != 0:
        batch_list.append(current_batch)
    return batch_list
        
class ForwardBatchOptimization(RevelationHorizonBase):
    """ this algorithm batches reservation requests in batch of sizes specified by the input parameter "op_res_batch_size"
    within the batches all possible requests groups are calculated (V2RB without a vehicle)
    in the assignement step, these batches are connected one after another starting with the first one
    the connection is made by solving a maximum priority matching problem to schedule request groups within batches
    the batches (requests) are in the current version sorted by their earliest pick-up time
    this method is currently only stable if all reservation requests are known beforehand (no update of solution possible)"""
    def __init__(self, fleetctrl : FleetControlBase, operator_attributes : dict, dir_names : dict, solver : str="Gurobi"):
        super().__init__(fleetctrl, operator_attributes, dir_names, solver=solver)

        self.solver = solver

        self.max_batch_size = int(operator_attributes[G_RA_MAX_BATCH_SIZE])
        self.N_batch_concat = int(operator_attributes[G_RA_MAX_BATCH_CONCAT])
        
    def _batch_optimisation(self, sim_time):
        LOG.debug("reservation fwd batch optimization!")
        if len(self._unprocessed_rids) != 0:
            # batch the requests
            request_batches = batch_requests(sorted([prq for rid, prq in self.active_reservation_requests.items() if self._reavealed_rids.get(rid) is None], key=lambda x:x.get_o_stop_info()[1]), self.max_batch_size)
            LOG.debug("{} batches with max size {} and {} requests created!".format(len(request_batches), self.max_batch_size, len(self.active_reservation_requests)))
            batch_rg_list = []
            for i, rq_list in enumerate(request_batches):
                LOG.debug(" -> create request batch {}/{}".format(i, len(request_batches)))
                rg = ReservationRequestBatch()
                for rq in rq_list:
                    rg.full_insertion_request_to_batch(rq, self.routing_engine, self.fleetctrl, self.vehicle_capacity)
                batch_rg_list.append(rg)
            # connect the batches
            if self.N_batch_concat == 1:
                self._forward_batch_optimization(sim_time, batch_rg_list)
            else:
                self._multi_forward_batch_optimization(sim_time, batch_rg_list, self.N_batch_concat)
            self._unprocessed_rids = {}
            
    def _forward_batch_optimization(self, sim_time: int, batch_rq_list: List[ReservationRequestBatch]):
        """ this function iterates through the list of batches and matches them together by solving a
        maximum priority matching problem. the expected start time of a request group has to exceed the expected end_time 
        of the former request group. as initial condition the vehicles and their currently assigned plans are used
        :param sim_time: current simulation time
        :param batch_rq_list: sorted list of request batches (only neighbouring batches are directly matched together)"""
        plan_id_to_part_best_plan : Dict[int, List[QuasiVehiclePlan]] = {}  # dict offline plan id to an ordered list of vehicle plans of the resulting assigned request groups
        # 1 to 1 ids after initial optimization
        plan_id_batch_constraints, current_assignment_horizon = self._get_vid_batch_constraints(sim_time)
                
        LOG.info("reoptimize reservation schedules completely")
        rid_to_prev_assigned_schedules = self._get_assigned_subplans_per_request_from_off_plans()
        LOG.debug(f"prev assigned schedules: {rid_to_prev_assigned_schedules}")
        # batch optimization
        for i in range(len(batch_rq_list)):
            current_batch = batch_rq_list[i]
            LOG.info(" ... process batch {}/{} with {} requests".format(i, len(batch_rq_list), current_batch.get_batch_size()))
            rg_objectives, rg_constraints = current_batch.get_rg_obj_const()
            for rid in current_batch.requests_in_batch.keys():
                if rid_to_prev_assigned_schedules.get(rid):
                    LOG.debug(f"prev assignment found for rid {rid} -> together with {rid_to_prev_assigned_schedules[rid][0].get_involved_request_ids()}")
            self._create_forced_init_connections(sim_time, plan_id_batch_constraints,)
            plan_id_assignments, number_rids, number_assigned_rids = self._match_rg_graph_to_start_constraints(rg_objectives, rg_constraints, plan_id_batch_constraints)
            if number_assigned_rids != number_rids:
                raise NotImplementedError(f"only {number_assigned_rids} of {number_rids} are assigned in this batch!")
            current_batch.set_assignments(plan_id_assignments)
            for plan_id, rgk in plan_id_assignments.items():
                LOG.debug(f" -> assign {plan_id} -> {rgk}")
                best_plan = current_batch.get_best_plan_of_rg(rgk, self.fleetctrl, self.routing_engine)
                _, _, end_pos, end_time = best_plan.get_start_end_constraints()
                plan_id_batch_constraints[plan_id] = (end_pos, end_time)
                try:
                    plan_id_to_part_best_plan[plan_id].append(best_plan)
                except KeyError:
                    plan_id_to_part_best_plan[plan_id] = [best_plan]
            LOG.info("  -> number of currently assigned vehicles: {}".format(len(plan_id_batch_constraints)))
            
        # create full offline plans
        self._plan_id_to_off_plan = {}
        for plan_id, list_batch_plans in plan_id_to_part_best_plan.items():
            full_off_list_ps = []
            for plan in list_batch_plans:
                full_off_list_ps += plan.list_plan_stops
            self._plan_id_to_off_plan[plan_id] = QuasiVehiclePlan(self.routing_engine, full_off_list_ps, self.vehicle_capacity)
                
    def _match_rg_graph_to_start_constraints(self, batch_rg_objectives : Dict[Any, float], batch_rg_constraints : Dict[Any, Tuple[float, float, float, float]],
                                             prev_batch_constaints : Dict[Any, Tuple[tuple, float]]) -> Tuple[Dict[Any, Any], int, int]:
        """ this function matches the current request groups of the batch to next one by solving
        a maximum priority matching problem by minimizing the travel time between consecutive request group schedules
        every request has to assigned, therby if a match is not possible, a new hypothetical vehicle is introduce
        # TODO # is another approach with not using as few vehicles as possible useful?
        :param batch_rg_objectives: dict rg key -> objective value
        :param batch_rg_constraints: dict rg key -> tuple of (start_pos, start_time, end_pos, end_time) of the plan
        :param prev_batch_constaints: dict vehicle id -> (end_pos, end_time) 
        :return: dict hypothetical vehicle id -> assigned request group id , number_rids, number_assigned_rids"""
        if self.solver == "Gurobi":
            return self._match_rg_graph_to_start_constraints_gurobi(batch_rg_objectives, batch_rg_constraints, prev_batch_constaints)
        else:
            raise EnvironmentError(f"Solver {self.solver} not implemented for this class!")

    def _match_rg_graph_to_start_constraints_gurobi(self, batch_rg_objectives : Dict[Any, float], batch_rg_constraints : Dict[Any, Tuple[float, float, float, float]],
                                             prev_batch_constaints : Dict[Any, Tuple[tuple, float]]) -> Tuple[Dict[Any, Any], int, int]:
        """ this function matches the current request groups of the batch to next one by solving
        a maximum priority matching problem using gurobi by minimizing the travel time between consecutive request group schedules
        every request has to assigned, therby if a match is not possible, a new hypothetical vehicle is introduce
        :param batch_rg_objectives: dict rg key -> objective value
        :param batch_rg_constraints: dict rg key -> tuple of (start_pos, start_time, end_pos, end_time) of the plan
        :param prev_batch_constaints: dict vehicle id -> (end_pos, end_time) 
        :return: dict hypothetical vehicle id -> assigned request group id , number_rids, number_assigned_rids"""

        import gurobipy as gurobi

        rg_to_vid = {}
        LOG.debug("opt input:")
        LOG.debug(f"batch rg obj : {batch_rg_objectives}")
        LOG.debug(f"batch rg constr : {batch_rg_constraints}")
        LOG.debug(f"prev batch constr: {prev_batch_constaints}")
        for rg, start_pos_time_end_pos_time in batch_rg_constraints.items():
            start_pos, start_time, _, _ = start_pos_time_end_pos_time
            for vid, lst_pos in prev_batch_constaints.items():
                vid_pos, last_end_time = lst_pos
                tt = self.routing_engine.return_travel_costs_1to1(vid_pos, start_pos)[1]
                #LOG.debug(f" u -> {rg} -> {vid} | {start_time} {last_end_time} {tt}")
                if start_time - last_end_time >= tt:
                    cost = batch_rg_objectives[rg] + start_time - last_end_time + tt #  TODO objective?
                    #LOG.debug(f" -> {rg} -> {vid} -> {cost}")
                    try:
                        rg_to_vid[rg][vid] = cost
                    except KeyError:
                        rg_to_vid[rg] = {vid : cost}

        new_assignments = {}    # hypothetical vid -> assigned rg group id
        number_rids = 0
        number_assigned_rids = 0
        with gurobi.Env(empty=True) as env:
            env.setParam('OutputFlag', 0)
            env.setParam('LogToConsole', 0)
            env.start()

            m = gurobi.Model("reverseBatchReservation", env = env)

            m.setParam(gurobi.GRB.param.Threads, self.fleetctrl.n_cpu)

            variables = {}  # rtv_key -> gurobi variable
            
            expr = gurobi.LinExpr()   # building optimization objective
            key_to_varnames = {}
            varnames_to_key = {}
            vids_const = {}
            rids_const = {}
            c = 0
            for rg, v_dict in rg_to_vid.items():
                for vid, cost in v_dict.items():
                    rids = list(rg)

                    key_to_varnames[(vid, rg)] = str(c)
                    varnames_to_key[str(c)] = (vid, rg)
                    try:
                        vids_const[vid].append((vid, rg))
                    except:
                        vids_const[vid] = [(vid, rg)]
                    for rid in rids:
                        try:
                            rids_const[rid].append((vid, rg))
                        except:
                            rids_const[rid] = [(vid, rg)]

                    var = m.addVar(name = str(c), obj = cost, vtype = gurobi.GRB.BINARY)
                    variables[(vid, rg)] = var
                    #print("var {} -> {}".format((vid, rg), cost))
                    expr.add(var, cost)
                    c += 1
                
            m.setObjective(expr, gurobi.GRB.MINIMIZE)

            #vehicle constraint -> maximally one assignment (no constraint of a vehicle not yet available is chosen)
            for vid in vids_const.keys():
                expr = gurobi.LinExpr()
                for rtv in vids_const[vid]:
                    expr.add(variables[rtv], 1)
                m.addConstr(expr, gurobi.GRB.LESS_EQUAL, 1, "c_v_{}".format(vid))
                
            #requests constraint -> assign (possibly) all
            for rid in rids_const.keys():
                expr = gurobi.LinExpr()
                for rtv in rids_const[rid]:
                    expr.add(variables[rtv], 1)
                if self._unprocessed_rids.get(rid):
                    m.addConstr(expr, gurobi.GRB.LESS_EQUAL, 1, "c_r_{}".format(rid))
                else:
                   m.addConstr(expr, gurobi.GRB.EQUAL, 1, "c_r_{}".format(rid)) 
                

            m.optimize() #optimization

            #get solution
            varnames = m.getAttr("VarName", m.getVars())
            solution = m.getAttr("X",m.getVars())
            
            sum_cfv = 0
            # vid, lst_pos
            for x in range(len(solution)):
                if round(solution[x]) == 1:
                    key = varnames_to_key[varnames[x]]
                    vid = key[0]
                    rg = key[1]
                    number_assigned_rids += len(rg)
                    cost = rg_to_vid[rg][vid]
                    sum_cfv += cost
                    new_assignments[vid] = rg
                    LOG.debug(f" -> {vid} -> {rg} : {cost}")
            number_rids = len(rids_const.keys())
        return new_assignments, number_rids, number_assigned_rids
    
    def _multi_forward_batch_optimization(self, sim_time, batch_rg_list, N_batch_concat):
        """ this function iterates through the list of batches and matches them together by solving a
        maximum priority matching problem. the expected start time of a request group has to exceed the expected end_time 
        of the former request group. as initial condition the vehicles and their currently assigned plans are used
        :param sim_time: current simulation time
        :param batch_rq_list: sorted list of request batches (only neighbouring batches are directly matched together)"""
        plan_id_to_part_best_plan : Dict[int, List[QuasiVehiclePlan]] = {}  # dict offline plan id to an ordered list of vehicle plans of the resulting assigned request groups
        # 1 to 1 ids after initial optimization
        plan_id_batch_constraints, current_assignment_horizon = self._get_vid_batch_constraints(sim_time)
                
        LOG.info("reoptimize reservation schedules completely")
        # batch optimization
        #for i in range(max(len(batch_rg_list) - N_batch_concat +1), 0):
        for i in range(len(batch_rg_list)):
            LOG.info(" ... process batch {}/{}".format(i, len(batch_rg_list)))
            t = time.time()
            current_batch = batch_rg_list[i]
            batch_rg_objectives, batch_rg_constraints = [], []
            for j in range( i, min( i+N_batch_concat, len(batch_rg_list) ) ):
                rg_objectives, rg_constraints = batch_rg_list[j].get_rg_obj_const()
                batch_rg_objectives.append(rg_objectives)
                batch_rg_constraints.append(rg_constraints)
            plan_id_assignments, number_rids, number_assigned_rids = self._match_batch_rg_graph_to_start_constraints(batch_rg_objectives, batch_rg_constraints, plan_id_batch_constraints)
            LOG.debug(" -> results: {}".format(plan_id_assignments))
            LOG.debug(" -> current batch: {}".format(batch_rg_objectives[0].keys()))
            LOG.debug(" -> {}/{} assigned".format(number_assigned_rids, number_rids))
            #raise NotImplementedError 
            current_batch.set_assignments(plan_id_assignments)
            for plan_id, rgk in plan_id_assignments.items():
                LOG.debug(f" -> assign {plan_id} -> {rgk}")
                best_plan = current_batch.get_best_plan_of_rg(rgk, self.fleetctrl, self.routing_engine)
                _, _, end_pos, end_time = best_plan.get_start_end_constraints()
                plan_id_batch_constraints[plan_id] = (end_pos, end_time)
                try:
                    plan_id_to_part_best_plan[plan_id].append(best_plan)
                except KeyError:
                    plan_id_to_part_best_plan[plan_id] = [best_plan]
            LOG.info("  -> took {}".format(time.time() - t))   
        # create full offline plans
        self._plan_id_to_off_plan = {}
        for plan_id, list_batch_plans in plan_id_to_part_best_plan.items():
            full_off_list_ps = []
            for plan in list_batch_plans:
                full_off_list_ps += plan.list_plan_stops
            self._plan_id_to_off_plan[plan_id] = QuasiVehiclePlan(self.routing_engine, full_off_list_ps, self.vehicle_capacity)
            
    def _match_batch_rg_graph_to_start_constraints(self, batch_list_rg_objectives : List[Dict[Any, float]], batch_list_rg_constraints : List[Dict[Any, Tuple[float, float, float, float]]],
                                                   plan_id_batch_constraints : Dict[Any, Tuple[tuple, float]]) -> Tuple[Dict[Any, Any], int, int]:
        """ TODO
        :param batch_rg_objectives: list batches dict rg key -> objective value
        :param batch_rg_constraints: list batches dict rg key -> tuple of (start_pos, start_time, end_pos, end_time) of the plan
        :param plan_id_batch_constraints: dict vehicle id -> (end_pos, end_time) 
        :return: dict hypothetical vehicle id -> assigned request group id , number_rids, number_assigned_rids"""
        if self.solver == "Gurobi":
            return self._match_batch_rg_graph_to_start_constraints_gurobi(batch_list_rg_objectives, batch_list_rg_constraints, plan_id_batch_constraints)
        else:
            raise EnvironmentError(f"Solver {self.solver} not implemented for this class!")
        
    def _match_batch_rg_graph_to_start_constraints_gurobi(self, batch_list_rg_objectives : List[Dict[Any, float]], batch_list_rg_constraints : List[Dict[Any, Tuple[float, float, float, float]]],
                                                   plan_id_batch_constraints : Dict[Any, Tuple[tuple, float]]) -> Tuple[Dict[Any, Any], int, int]:
        """ TODO
        :param batch_rg_objectives: list batches dict rg key -> objective value
        :param batch_rg_constraints: list batches dict rg key -> tuple of (start_pos, start_time, end_pos, end_time) of the plan
        :param plan_id_batch_constraints: dict vehicle id -> (end_pos, end_time) 
        :return: dict hypothetical vehicle id -> assigned request group id , number_rids, number_assigned_rids"""
        import gurobipy as gurobi
        with gurobi.Env(empty=True) as env:
            env.setParam('OutputFlag', 0)
            env.setParam('LogToConsole', 0)
            env.start()

            m = gurobi.Model("reverseBatchReservation", env = env)

            m.setParam(gurobi.GRB.param.Threads, self.fleetctrl.n_cpu)

            variables = {}  # var_key -> gurobi variable
            incoming_constr = {} # node -> var_key -> 1
            outgoing_constr = {}    # node -> var_key -> 1
            nodes = {}  # node -> 1
            rid_constr = {}  # rid -> var_key -> 1 
            
            last_end = {f"v_{vid}" : e_p_t for vid, e_p_t in plan_id_batch_constraints.items()}
            vids = list(last_end.keys())
            var_c = 0
            LOG.debug("optimizing batches")
            for i in range(len(batch_list_rg_constraints)):
                rg_obj = batch_list_rg_objectives[i]
                LOG.debug(f" -> batch {i} with {rg_obj.keys()}")
                rg_const = batch_list_rg_constraints[i]
                cur_last_end = list(last_end.items())
                for rg, obj in rg_obj.items():
                    start_pos, start_time, end_pos, end_time = rg_const[rg]
                    last_end[rg] = (end_pos, end_time)
                    for l_key, e_p_t in cur_last_end:
                        f_end_pos, f_end_time = e_p_t
                        _, tt, _ = self.routing_engine.return_travel_costs_1to1(f_end_pos, start_pos)
                        if tt <= start_time - f_end_time:
                            # TODO better way to do that?
                            # Define Variable and Cost
                            # qvp = QuasiVehiclePlan(self.routing_engine, [RoutingTargetPlanStop(f_end_pos), RoutingTargetPlanStop(start_pos)], self.vehicle_capacity)
                            # cfv = qvp.compute_obj_function(self.fleetctrl.vr_ctrl_f, self.routing_engine, self.active_reservation_requests)
                            cfv = self.driving_leg_obj(None, self.routing_engine, f_end_pos, start_pos, f_end_time, start_time)
                            var = m.addVar(name = "{}_{}".format(l_key, rg), obj = obj + cfv, vtype = gurobi.GRB.BINARY)
                            # LOG.debug(f"var {'{}_{}'.format(l_key, rg)} : {obj + cfv}")
                            var_key = (l_key, rg)
                            variables[var_key] = var
                            var_c += 1
                            # add nodes
                            nodes[l_key] = 1
                            nodes[rg] = 1
                            # add incoming constraint
                            try:
                                incoming_constr[rg][var_key] = 1
                            except KeyError:
                                incoming_constr[rg] = {var_key : 1}
                            # add outgoing constraint
                            try:
                                outgoing_constr[l_key][var_key] = 1
                            except KeyError:
                                outgoing_constr[l_key] = {var_key : 1}
                            # add rid constraint
                            for rid in rg:
                                try:
                                    rid_constr[rid][var_key] = 1
                                except KeyError:
                                    rid_constr[rid] = {var_key : 1}
            # reconnect the vehicles to form cycle
            for l_key in nodes.keys():
                for vid in vids:
                    if type(l_key) == str and l_key[0] == "v":
                        continue
                    var = m.addVar(name = "{}_{}".format(l_key, vid), obj = 0, vtype = gurobi.GRB.BINARY)
                    var_key = (l_key, vid)
                    variables[var_key] = var
                    var_c += 1
                    # add incoming constraint
                    try:
                        incoming_constr[vid][var_key] = 1
                    except KeyError:
                        incoming_constr[vid] = {var_key : 1}
                    # add outgoing constraint
                    try:
                        outgoing_constr[l_key][var_key] = 1
                    except KeyError:
                        outgoing_constr[l_key] = {var_key : 1}

            #define gurobi model
            
            #1) incoming constraints
            for node, var_dict in incoming_constr.items():
                m.addConstr(sum(variables[x] for x in var_dict.keys()) <= 1, name=f"in {node}")
            #2) outgoing constraints
            for node, var_dict in outgoing_constr.items():
                m.addConstr(sum(variables[x] for x in var_dict.keys()) <= 1, name=f"out {node}")
            #3) rid constraints
            for rid, var_dict in rid_constr.items():
                m.addConstr(sum(variables[x] for x in var_dict.keys()) <= 1, name=f"rid {rid}")
            #4) flow constraints
            for node in nodes.keys():
                m.addConstr(sum(variables[x] for x in incoming_constr.get(node, {}).keys()) - sum(variables[x] for x in outgoing_constr.get(node, {}).keys()) == 0, name = f"flow {node}" )
            
            #m.write(r'C:\Users\ge37ser\Documents\Coding\TUM_VT_FleetSimulation\tum-vt-fleet-simulation\FleetPy\studies\reservation\dev\model.lp')
            m.optimize()
            LOG.info("number variables: {}".format(var_c))
            
            vals = m.getAttr('X', variables)
            sols = {x : 1 for x, v in vals.items() if int(np.round(v)) == 1}
            LOG.debug("opt sols:")
            LOG.debug(f"{list(sols.keys())}")
            sol_graph = {}
            for s, e in sols.keys():
                sol_graph[s] = e
            sol_schedules = []
            plan_id_to_assigned_rg = {}
            n_assigned_rids = 0
            for vid in vids:
                schedule = []
                LOG.debug(f"vid {vid}")
                if sol_graph.get(vid):
                    cur = vid
                    while type(sol_graph[cur]) != str and sol_graph[cur][0] != "v":
                        cur = sol_graph[cur]
                        n_assigned_rids += len(cur)
                        schedule.append(cur)
                LOG.debug(f" -> schedule : {schedule}")
                sol_schedules.append(schedule)
                plan_id = int(vid.split("_")[1])
                first_rg = sol_graph.get(vid)
                if first_rg is not None:
                    if batch_list_rg_constraints[0].get(first_rg) is not None:
                        plan_id_to_assigned_rg[plan_id] = first_rg
            return plan_id_to_assigned_rg, len(rid_constr.keys()), n_assigned_rids
        
            
            