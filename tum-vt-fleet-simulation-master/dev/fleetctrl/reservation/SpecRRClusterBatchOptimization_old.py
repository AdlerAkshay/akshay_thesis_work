import os
import sys
import numpy as np
import pandas as pd
import time
from sklearn.cluster import SpectralClustering
from src.simulation.Offers import TravellerOffer
from src.fleetctrl.FleetControlBase import FleetControlBase
from src.routing.NetworkBase import NetworkBase
from src.fleetctrl.planning.VehiclePlan import PlanStopBase, RoutingTargetPlanStop
from src.fleetctrl.planning.PlanRequest import PlanRequest
from dev.fleetctrl.reservation.RevelationHorizonBase import RevelationHorizonBase
from dev.fleetctrl.reservation.misc.RequestGroup import RequestGroup, QuasiVehiclePlan, QuasiVehicle, VehiclePlanSupportingPoint, rg_key, get_lower_keys
from dev.fleetctrl.reservation.ReservationRequestBatch import ReservationRequestBatch, merge_reservationRequestBatches
from src.misc.globals import *

from typing import Any, Dict, List, Tuple

import logging
LOG = logging.getLogger(__name__)

NEW_VID_PENALTY = 1000000   # penalty for introducing a new vehicle in case a match between batches is not possible
SMALL_VALUE = 0.00000000001     # a small number to set np.floats to 0


def check_shared_rr(rq1 : PlanRequest, rq2 : PlanRequest, nw: NetworkBase, std_bt):
    """ this function checks if an shared rr route can be found between two requests
    i.e. it check if either the route o1 -> o2 -> d2 -> d1 or o1 -> o2 -> d1 -> d2 is feasible
    note that check_shared_rr(rq_1, rq_2, **) != check_shared_rr(rq_2, rq_1, **)
    :param rq_1: first plan request object (its origin is always the first in the possible routes to check)
    :param rq_2: second plan request object (its origin is always the second in the possible routes to check)
    :param nw: routing engine obj
    :param std_bt: boarding time
    :return: bool -> True, if a shared route is feasible where both requests are onboard at the same time"""
    if rq1.get_rid_struct() == rq2.get_rid_struct():
        return False
    o_pos_1, ept_1, lpt_1 = rq1.get_o_stop_info()
    o_pos_2, ept_2, lpt_2 = rq2.get_o_stop_info()
    d_pos_1, ldt_1, mtt_1 = rq1.get_d_stop_info()
    d_pos_2, ldt_2, mtt_2 = rq2.get_d_stop_info()
    if (ept_1 <= lpt_2 and ldt_1 >= ept_2) or (ept_2 <= lpt_1 and ldt_2 >= ept_1):
        # o1 -> o2 -> d1 -> d2
        cur_time = ept_1
        pu_t_1 = cur_time
        if o_pos_1 != o_pos_2:
            next_time = cur_time + std_bt + nw.return_travel_costs_1to1(o_pos_1, o_pos_2)[1]
        else:
            next_time = cur_time
        if next_time <= lpt_2:
            cur_time = max(next_time, ept_2)
            pu_t_2 = cur_time
            if o_pos_2 != d_pos_1:
                next_time = cur_time + std_bt + nw.return_travel_costs_1to1(o_pos_2, d_pos_1)[1]
            else:
                next_time = cur_time
            if next_time <= ldt_1 and next_time - pu_t_1 <= mtt_1:
                cur_time = next_time
                if d_pos_1 != d_pos_2:
                    next_time = cur_time + std_bt + nw.return_travel_costs_1to1(d_pos_1, d_pos_2)[1]
                else:
                    next_time = cur_time
                if next_time <= ldt_2 and next_time - pu_t_2 <= mtt_2:
                    return True
            # o1 -> o2 -> d2 -> d1
            cur_time = pu_t_2
            if o_pos_2 != d_pos_2:
                next_time = cur_time + std_bt + nw.return_travel_costs_1to1(o_pos_2, d_pos_2)[1]
            else:
                next_time = cur_time
            if next_time <= ldt_2:
                cur_time = next_time
                if d_pos_1 != d_pos_2:
                    next_time = cur_time + std_bt + nw.return_travel_costs_1to1(d_pos_2, d_pos_1)[1]
                else:
                    next_time = cur_time
                if next_time <= ldt_1 and next_time - pu_t_1 <= mtt_1:
                    return True
    return False   
                    
def split_adj_matrix(components_vector : np.array, orig_adj : np.array, rid_to_index : Dict[Any, int]) -> Tuple[List[np.array], Dict[Any, int]]:
    """ this method is used to split the adj matrix into smaller ones based on affiliations to compontents in the components vector
    :param components vector: vector with int entries x=components_vector[i] describes the affiliation of index i to component x
    :param orig_adj: adjacency matrix before splitting it
    :param rid_to_index: dict request id -> matrix index
    :return: Tuple of List of smaller adjency matrices (one for each component), List of rid_to_index dicts corresponding the new matrices"""
    index_to_rid = {v : k for k, v in rid_to_index.items()}
    n_components = int(np.round(max(components_vector))) + 1
    
    nodes_in_component = [ [] for i in range(n_components)]
    for i, c in enumerate(components_vector):
        nodes_in_component[int(np.round(c))].append(i)
    
    new_adjs = []
    new_rid_to_indexes = []
    for nodes in nodes_in_component:
        if len(nodes) == 0:
            continue
        new_adj = np.zeros( (len(nodes), len(nodes) ) )
        new_rid_to_index = {}
        for i, c1 in enumerate(nodes):
            new_rid_to_index[index_to_rid[c1]] = i
            for j, c2 in enumerate(nodes):
                new_adj[i, j] = orig_adj[c1, c2]
        new_adjs.append(new_adj)
        new_rid_to_indexes.append(new_rid_to_index)
        
    return new_adjs, new_rid_to_indexes
                    
def split_in_components(adj_mat : np.array, rid_to_index : Dict[Any, int]) -> Tuple[List[np.array], Dict[Any, int]]:
    """ this method is used to identify unconnected components in the adjacency matrix and returns
    a list of separated adjacency matrices corresponding to these components
    spectral decomposition is used to identify these components
    -> separate the nodes by entries in 0-eigenvalue eigenvectors
    (https://towardsdatascience.com/spectral-clustering-aba2640c0d5b)
    :param adj_mat: adjacency matrix
    :param rid_to_index: request-id -> matrix index
    :return: tuple (list of split adjacency matrices, list of rid_to_index-dicts for corresponding list of adjacency matrices)"""
    # compute laplacian matrix
    deg = np.diag(adj_mat.sum(axis=1))
    laplacian = deg - adj_mat
    # calculate eigenvals and eigenvectors
    eigenvals, eigenvecs = np.linalg.eig(laplacian)
    eigenvecs = eigenvecs[:,np.argsort(eigenvals)]
    eigenvals = eigenvals[np.argsort(eigenvals)]
    # identify unconnected componentes
    components = np.zeros(shape=(eigenvals.shape[0]))
    for i in range(1, len(eigenvals)):  # laplacian always has one zero eigenvalue -> skip this one
        val = abs(eigenvals[i])
        if val > SMALL_VALUE:   # all components found when first eigenvalue large than zeor
            break
        else:
            for j in range(len(eigenvals)):
                if abs(eigenvecs[j, i]) > SMALL_VALUE:  # non-zero entries used to identify the affiliation to a component
                    components[j] = i
    # split matrix
    new_adjs, new_rid_to_indexes = split_adj_matrix(components, adj_mat, rid_to_index)
    return new_adjs, new_rid_to_indexes

def spectral_clustering(adj_matrix : np.array, rid_to_index : Dict[Any, int], N_clusters : int) -> Tuple[List[np.array], Dict[Any, int]]:
    """ this method uses spectral clustering to compute clusters in the adjacency matrix (k-means clustering of eigenvectors with highest eigenvalues
    :param adj_matrix: adjancency matrix
    :param rid_to_index: request id -> matrix index
    :param N_clusters: number of clusters for the k-means algorithm
    :return: tuple (list of split adjacency matrices of the resulting clusters, list of rid_to_index-dicts for corresponding list of adjacency matrices)"""
    sc = SpectralClustering(N_clusters, affinity='precomputed', n_init=100, assign_labels='discretize')
    fit = sc.fit_predict(adj_matrix)    
    return split_adj_matrix(fit, adj_matrix, rid_to_index)

def batch_requests(rq_dict: Dict[Any, PlanRequest], number_requests_in_batch: int, nw: NetworkBase, fleet_ctrl_ref: FleetControlBase, vehicle_capacity: int) -> List[List[Any]]:
    """ this function is used to created request batches based on clusters in the rr-graph
    1) an shared-rr graph is created; i.e. all requests are connected that could be served in a route where part of the trip is shared
    2) the adjacency matrix of the graph is created
    3) recursivley, clusters are computed in this adjacency matrix
        3.1) unconnected parts of the rr-graph are found and split out
        3.2) spectral clustering is used to further split the adjacency matrix in subparts (clusters)
    :param rq_dict: dictonary rid -> plan request objects
    :param number_requests_in_batch: int number of number objects per batch
    :param nw: routing engine obj
    :param fleet_ctrl_ref: reference to fleetcontrol class
    :param vehicle_capacity: reference to vehicle capacity
    :return: list of list of rids. the length of the inner list does not exceed number_requests_in_batch"""
    
    # 1) create shared rr-graph and adjacency matrix
    t = time.time()
    rid_list = list(rq_dict.keys())
    rid_to_index = {}
    adj_mat = np.zeros(shape=(len(rq_dict), len(rq_dict)))
    for w, rid1 in enumerate(rid_list):
        prq1 = rq_dict[rid1]
        rid_to_index[rid1] = w
        for v in range(w, len(rid_list)):
            rid2 = rid_list[v]
            rid_to_index[rid2] = v
            prq2 = rq_dict[rid2]
            if check_shared_rr(prq1, prq2, nw, fleet_ctrl_ref.const_bt):
                adj_mat[w,v] = 1
                adj_mat[v,w] = 1
            elif check_shared_rr(prq2, prq1, nw, fleet_ctrl_ref.const_bt):
                    adj_mat[w,v] = 1
                    adj_mat[v,w] = 1
    t_rr = time.time() - t
    t = time.time()
    
    # 2) cluster the requests recursively based on the shared rr-graph
    final_adj_matrices = []
    final_rid_to_index = []
    def clustering_recursive(in_adj_matrix, in_rid_to_index, max_rids_per_cluster):
        """ this method recursively splits the adjency matrix until sizes smaller than max_number_rids_per_cluster are identified
        it first separates the matrix in components and the uses spectral clustering to further define clusters
        thereby the lists "final_adj_matrices" and "final_rid_to_index" are filled
        :param in_adj_matrix: input adjacency matrix
        :param in_rid_to_index: input rid to index dict
        :param max_rids_per_cluster: int break condition, i.e. largest cluster size"""
        comp_adjs, comp_rid_to_index = split_in_components(in_adj_matrix, in_rid_to_index)
        for c_adj, c_rid_to_index in zip(comp_adjs, comp_rid_to_index):
            #print("split", c_adj.shape)
            if c_adj.shape[0] <= max_rids_per_cluster:
                final_adj_matrices.append(c_adj)
                final_rid_to_index.append(c_rid_to_index)    
            else:
                #print(c_adj)
                N_clusters = max(2, int(np.ceil(c_adj.shape[0]/max_rids_per_cluster)))
                spec_adjs, spec_rid_to_indexs = spectral_clustering(c_adj, c_rid_to_index, N_clusters)
                for spec_adj, spec_rid_to_index in zip(spec_adjs, spec_rid_to_indexs):
                    #print("spec", spec_adj.shape)
                    #print(spec_adj)
                    clustering_recursive(spec_adj, spec_rid_to_index, max_rids_per_cluster)                                
    clustering_recursive(adj_mat, rid_to_index, number_requests_in_batch)
    t_cluster = time.time() - t
    t = time.time()
        
    list_request_batches = [list(cluster_rid_to_index.keys()) for cluster_rid_to_index in final_rid_to_index]
    
    LOG.info(f"reservation batching took: {t_rr}s for rr computation | {t_cluster}s for creating clusters")
        
    return list_request_batches


class SpecRRClusterBatchOptimization(RevelationHorizonBase):
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
            t = time.time()
            request_batches = batch_requests({rid : prq for rid, prq in self.active_reservation_requests.items() if self._reavealed_rids.get(rid) is None}, self.max_batch_size, self.routing_engine, self.fleetctrl, self.vehicle_capacity)
            LOG.debug("{} batches with max size {} and {} requests created! -> took {}".format(len(request_batches), self.max_batch_size, len(self.active_reservation_requests), time.time() - t))
            # create schedules in batches
            t = time.time()
            batch_rg_list = []
            for i, rid_list in enumerate(request_batches):
                LOG.debug(" -> create request batch {}/{} with {} requests".format(i, len(request_batches), len(rid_list)))
                rg = ReservationRequestBatch()
                for rid in rid_list:
                    rq = self.active_reservation_requests[rid]
                    rg.full_insertion_request_to_batch(rq, self.routing_engine, self.fleetctrl, self.vehicle_capacity)
                batch_rg_list.append(rg)
            LOG.debug("creating schedules in batches took {}".format(time.time() - t))
            # sort batches based on epa of rqs
            batch_rg_list = sorted(batch_rg_list, key = lambda x:x.get_batch_time_window()[0])
            # merge requests for assignment
            merged_batch_rg_list = []
            N_current_rids = 0
            current_batch_rg_list = []
            for batch in batch_rg_list:
                if N_current_rids + batch.get_batch_size() > self.max_batch_size:
                    merged_batch_rg_list.append(merge_reservationRequestBatches(current_batch_rg_list))
                    current_batch_rg_list = []
                    N_current_rids = 0
                current_batch_rg_list.append(batch)
                N_current_rids += batch.get_batch_size()
            if len(current_batch_rg_list) > 0:
                merged_batch_rg_list.append(merge_reservationRequestBatches(current_batch_rg_list))
                current_batch_rg_list = []
                N_current_rids = 0
            LOG.debug(f"merging batches reduced number from {len(batch_rg_list)} to {len(merged_batch_rg_list)}")
            # solve optimization problem to assign and connect schedules
            self._multi_forward_batch_optimization(sim_time, merged_batch_rg_list, self.N_batch_concat)
            #exit()
            self._unprocessed_rids = {}
            
    def _multi_forward_batch_optimization(self, sim_time, batch_rg_list : List[ReservationRequestBatch], N_batch_concat):
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
        
        N_current_rids = 0
        current_rg_objectives = {}
        current_rg_constraints = {}
        current_assignment_rgs_batch = {}
        for i in range(len(batch_rg_list)):
            LOG.info(" ... process batch {}/{} with times {}".format(i, len(batch_rg_list), batch_rg_list[i].get_batch_time_window()))
            current_batch = batch_rg_list[i]
            rg_objectives, rg_constraints = current_batch.get_rg_obj_const()
            current_rg_objectives.update(rg_objectives)
            current_rg_constraints.update(rg_constraints)
            for rg in rg_constraints.keys():
                current_assignment_rgs_batch[rg] = i
            N_current_rids += current_batch.get_batch_size()
            if i+1 == len(batch_rg_list) or N_current_rids + batch_rg_list[i+1].get_batch_size() > self.max_batch_size:
                # new optimization
                LOG.info("start optimization with {} requests and {} schedules".format(N_current_rids, len(current_rg_constraints)))
                if i + 1 < len(batch_rg_list):
                    for j in range(i+1, len(batch_rg_list)):
                        if j  == len(batch_rg_list) or N_current_rids + batch_rg_list[j].get_batch_size() > self.max_batch_size * N_batch_concat:
                            LOG.info(" -> while considering {} requests and {} schedules".format(N_current_rids, len(current_rg_constraints)))
                            break
                        LOG.debug(" add batch with tw {}".format(batch_rg_list[j].get_batch_time_window()))
                        follow_batch = batch_rg_list[j]
                        rg_objectives, rg_constraints = follow_batch.get_rg_obj_const()
                        current_rg_objectives.update(rg_objectives)
                        current_rg_constraints.update(rg_constraints)
                        N_current_rids += follow_batch.get_batch_size()
                        for rg in rg_objectives.keys():
                            current_assignment_rgs_batch[rg] = j
                        
                plan_id_to_assigned_rgs, n_requests, n_assigned_rids = self._match_batch_rg_graph_to_start_constraints(current_rg_objectives, current_rg_constraints, plan_id_batch_constraints, current_assignment_rgs_batch)
                for plan_id, assigned_rgs in plan_id_to_assigned_rgs.items():
                    for rg in assigned_rgs:
                        batch_index = current_assignment_rgs_batch.get(rg)
                        if batch_index is None or batch_index > i: # TODO cannot guarantee that this is a valid break always
                            break
                        best_plan = batch_rg_list[batch_index].get_best_plan_of_rg(rg, self.fleetctrl, self.routing_engine)
                        _, _, end_pos, end_time = best_plan.get_start_end_constraints()
                        plan_id_batch_constraints[plan_id] = (end_pos, end_time)
                        try:
                            plan_id_to_part_best_plan[plan_id].append(best_plan)
                        except KeyError:
                            plan_id_to_part_best_plan[plan_id] = [best_plan]
                            
                N_current_rids = 0
                current_rg_objectives = {}
                current_rg_constraints = {}
                current_assignment_rgs_batch = {}
                                       
        # create full offline plans
        self._plan_id_to_off_plan = {}
        for plan_id, list_batch_plans in plan_id_to_part_best_plan.items():
            full_off_list_ps = []
            for plan in list_batch_plans:
                full_off_list_ps += plan.list_plan_stops
            self._plan_id_to_off_plan[plan_id] = QuasiVehiclePlan(self.routing_engine, full_off_list_ps, self.vehicle_capacity)

        
    def _match_batch_rg_graph_to_start_constraints(self, batch_rg_objectives : Dict[Any, float], batch_rg_constraints : Dict[Any, Tuple[float, float, float, float]],
                                                   plan_id_batch_constraints : Dict[Any, Tuple[tuple, float]], current_assignment_rgs_batch : Dict[Any, int]) -> Tuple[Dict[Any, List[Any]], int, int]:
        """ TODO
        :param batch_rg_objectives: dict rg key -> objective value
        :param batch_rg_constraints: dict rg key -> tuple of (start_pos, start_time, end_pos, end_time) of the plan
        :param plan_id_batch_constraints: dict vehicle id -> (end_pos, end_time) 
        :param current_assignment_rgs_batch: dict rg-key -> batch index
        :return: dict hypothetical vehicle id -> list of assigned request group ids , number_rids, number_assigned_rids"""
        if self.solver == "Gurobi":
            return self._match_batch_rg_graph_to_start_constraints_gurobi(batch_rg_objectives, batch_rg_constraints, plan_id_batch_constraints, current_assignment_rgs_batch)
        else:
            raise EnvironmentError(f"Solver {self.solver} not implemented for this class!")
        
    def _match_batch_rg_graph_to_start_constraints_gurobi(self, batch_rg_objectives : Dict[Any, float], batch_rg_constraints : Dict[Any, Tuple[float, float, float, float]],
                                                   plan_id_batch_constraints : Dict[Any, Tuple[tuple, float]], current_assignment_rgs_batch : Dict[Any, int]) -> Tuple[Dict[Any, List[Any]], int, int]:
        """ TODO
        :param batch_rg_objectives: dict rg key -> objective value
        :param batch_rg_constraints: dict rg key -> tuple of (start_pos, start_time, end_pos, end_time) of the plan
        :param plan_id_batch_constraints: dict vehicle id -> (end_pos, end_time) 
        :param current_assignment_rgs_batch: dict rg-key -> batch index
        :return: dict hypothetical vehicle id -> list of assigned request group ids , number_rids, number_assigned_rids"""
        
        import gurobipy as gurobi
        with gurobi.Env(empty=True) as env:
            env.setParam('OutputFlag', 0)
            env.setParam('LogToConsole', 0)
            env.start()

            m = gurobi.Model("SpecRRClusterBatchOpt", env = env)

            m.setParam(gurobi.GRB.param.Threads, self.fleetctrl.n_cpu)

            variables = {}  # var_key -> gurobi variable
            incoming_constr = {} # node -> var_key -> 1
            outgoing_constr = {}    # node -> var_key -> 1
            nodes = {}  # node -> 1
            rid_constr = {}  # rid -> var_key -> 1 
            
            last_end = {f"v_{vid}" : end_pos_end_time for vid, end_pos_end_time in plan_id_batch_constraints.items()}
            vids = list(last_end.keys())
            var_c = 0
            LOG.debug("optimizing batches")
            sorted_batch_rgs = sorted( (k for k in batch_rg_constraints.keys()), key=lambda x:batch_rg_constraints[x][1] )
            #LOG.debug("sorted batch rgs : {}".format([(rg, batch_rg_constraints[rg][1]) for rg in sorted_batch_rgs]))
            for i, rg in enumerate(sorted_batch_rgs):
                start_pos, start_time, end_pos, end_time = batch_rg_constraints[rg]
                rg_obj = batch_rg_objectives[rg]
                batch_index = current_assignment_rgs_batch[rg]
                
                # with start nodes
                for l_key, f_end_pos_end_time in last_end.items():
                    f_end_pos, f_end_time = f_end_pos_end_time
                    if f_end_time <= start_time:
                        _, tt, _ = self.routing_engine.return_travel_costs_1to1(f_end_pos, start_pos)
                        if tt <= start_time - f_end_time:
                            # Define Variable and Cost
                            # qvp = QuasiVehiclePlan(self.routing_engine, [RoutingTargetPlanStop(f_end_pos), RoutingTargetPlanStop(start_pos)], self.vehicle_capacity)
                            # cfv = qvp.compute_obj_function(self.fleetctrl.vr_ctrl_f, self.routing_engine, self.active_reservation_requests)
                            cfv = self.driving_leg_obj(None, self.routing_engine, f_end_pos, start_pos, f_end_time, start_time)
                            var = m.addVar(name = "{}_{}".format(l_key, rg), obj = rg_obj + cfv, vtype = gurobi.GRB.BINARY)
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
                # with other rgs
                if i > 0:
                    for j in range(i):
                        f_rg = sorted_batch_rgs[j]
                        f_batch_index = current_assignment_rgs_batch[f_rg]
                        if batch_index < f_batch_index:
                            LOG.debug(f"forbid connection between {f_rg} and {rg} with batch indices {f_batch_index} and {batch_index}")
                            continue
                        l_key = f_rg
                        _, _, f_end_pos, f_end_time = batch_rg_constraints[f_rg]
                        #LOG.debug(f"{j} -> {i} : {f_end_pos} {f_end_time} -> {start_pos} {start_time} | {f_rg} -> {rg}")
                        if f_end_time <= start_time:
                            _, tt, _ = self.routing_engine.return_travel_costs_1to1(f_end_pos, start_pos)
                            if tt <= start_time - f_end_time:
                                # Define Variable and Cost
                                # qvp = QuasiVehiclePlan(self.routing_engine, [RoutingTargetPlanStop(f_end_pos), RoutingTargetPlanStop(start_pos)], self.vehicle_capacity)
                                # cfv2 = qvp.compute_obj_function(self.fleetctrl.vr_ctrl_f, self.routing_engine, self.active_reservation_requests)
                                cfv = self.driving_leg_obj(None, self.routing_engine, f_end_pos, start_pos, f_end_time, start_time)
                                #LOG.debug(f" -> cfv {cfv} {cfv2} | {rg_obj + cfv}")
                                var = m.addVar(name = "{}_{}".format(l_key, rg), obj = rg_obj + cfv, vtype = gurobi.GRB.BINARY)
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
                                        
            # base node and connections to vehicles
            base_name = "base"
            # base node to vid and vid to base node
            for vid in vids:
                var = m.addVar(name = "{}_{}".format(base_name, vid), obj = 0, vtype = gurobi.GRB.BINARY)
                var_key = (base_name, vid)
                variables[var_key] = var
                var_c += 1
                # add incoming constraint
                try:
                    incoming_constr[vid][var_key] = 1
                except KeyError:
                    incoming_constr[vid] = {var_key : 1}
                # add outgoing constraint
                try:
                    outgoing_constr[base_name][var_key] = 1
                except KeyError:
                    outgoing_constr[base_name] = {var_key : 1}
                var = m.addVar(name = "{}_{}".format(vid, base_name), obj = 0, vtype = gurobi.GRB.BINARY)
                var_key = (vid, base_name)
                variables[var_key] = var
                var_c += 1
                # add incoming constraint
                try:
                    incoming_constr[base_name][var_key] = 1
                except KeyError:
                    incoming_constr[base_name] = {var_key : 1}
                # add outgoing constraint
                try:
                    outgoing_constr[vid][var_key] = 1
                except KeyError:
                    outgoing_constr[vid] = {var_key : 1}
            # all nodes to base node
            for l_key in nodes.keys():
                if type(l_key) == str and l_key.startswith("v"):
                    continue
                var = m.addVar(name = "{}_{}".format(l_key, base_name), obj = 0, vtype = gurobi.GRB.BINARY)
                var_key = (l_key, base_name)
                variables[var_key] = var
                var_c += 1
                # add incoming constraint
                try:
                    incoming_constr[base_name][var_key] = 1
                except KeyError:
                    incoming_constr[base_name] = {var_key : 1}
                # add outgoing constraint
                try:
                    outgoing_constr[l_key][var_key] = 1
                except KeyError:
                    outgoing_constr[l_key] = {var_key : 1}
            nodes[base_name] = 1
                        
            #define constraints
            
            #1) incoming constraints
            for node, var_dict in incoming_constr.items():
                if type(node) == str and node == base_name:
                    m.addConstr(sum(variables[x] for x in var_dict.keys()) == len(vids), name=f"in {node}")
                else:
                    m.addConstr(sum(variables[x] for x in var_dict.keys()) <= 1, name=f"in {node}")
            #2) outgoing constraints
            for node, var_dict in outgoing_constr.items():
                if type(node) == str and node == base_name:
                    m.addConstr(sum(variables[x] for x in var_dict.keys()) == len(vids), name=f"out {node}")
                else:
                    m.addConstr(sum(variables[x] for x in var_dict.keys()) <= 1, name=f"out {node}")
            #3) rid constraints
            for rid, var_dict in rid_constr.items():
                m.addConstr(sum(variables[x] for x in var_dict.keys()) <= 1, name=f"rid {rid}")
            #4) flow constraints
            for node in nodes.keys():
                m.addConstr(sum(variables[x] for x in incoming_constr.get(node, {}).keys()) - sum(variables[x] for x in outgoing_constr.get(node, {}).keys()) == 0, name = f"flow {node}" )
            
            # optimize
            LOG.info("number variables: {}".format(var_c)) 
            m.optimize()
            
            # retrieve solution
            vals = m.getAttr('X', variables)
            sols = {x : 1 for x, v in vals.items() if int(np.round(v)) == 1}
            LOG.debug("opt sols:")
            LOG.debug(f"{list(sols.keys())}")

            sol_graph = {}
            for s, e in sols.keys():
                if type(s) == str and s == base_name:
                    continue
                sol_graph[s] = e
            sol_schedules = []
            plan_id_to_assigned_rgs = {}
            n_assigned_rids = 0
            for vid in vids:
                schedule = []
                LOG.debug(f"vid {vid}")
                cur = vid
                while type(sol_graph[cur]) != str and sol_graph[cur] != base_name:
                    cur = sol_graph[cur]
                    n_assigned_rids += len(cur)
                    schedule.append(cur)
                LOG.debug(f" -> schedule : {schedule}")
                sol_schedules.append(schedule)
                plan_id = int(vid.split("_")[1])
                plan_id_to_assigned_rgs[plan_id] = schedule
                
            return plan_id_to_assigned_rgs, len(rid_constr.keys()), n_assigned_rids
                