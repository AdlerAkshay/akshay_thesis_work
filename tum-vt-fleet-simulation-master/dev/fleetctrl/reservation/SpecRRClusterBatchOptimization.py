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
from dev.fleetctrl.reservation.BatchSchedulingRevelationHorizonBase import BatchSchedulingRevelationHorizonBase
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

def spectral_clustering(adj_matrix : np.array, rid_to_index : Dict[Any, int], N_clusters : int, n_threads : int=1) -> Tuple[List[np.array], Dict[Any, int]]:
    """ this method uses spectral clustering to compute clusters in the adjacency matrix (k-means clustering of eigenvectors with highest eigenvalues
    :param adj_matrix: adjancency matrix
    :param rid_to_index: request id -> matrix index
    :param N_clusters: number of clusters for the k-means algorithm
    :param n_threads: number of parallel threads
    :return: tuple (list of split adjacency matrices of the resulting clusters, list of rid_to_index-dicts for corresponding list of adjacency matrices)"""
    sc = SpectralClustering(N_clusters, affinity='precomputed', n_init=100, assign_labels='discretize', n_jobs=n_threads)
    fit = sc.fit_predict(adj_matrix)    
    return split_adj_matrix(fit, adj_matrix, rid_to_index)


class SpecRRClusterBatchOptimization(BatchSchedulingRevelationHorizonBase):
    """ this algorithm batches reservation requests in batch of sizes specified by the input parameter "op_res_batch_size"
    within the batches all possible requests groups are calculated (V2RB without a vehicle)
    in the assignement step, these batches are connected one after another starting with the first one
    the connection is made by solving a maximum priority matching problem to schedule request groups within batches
    the batches (requests) are in the current version sorted by their earliest pick-up time
    this method is currently only stable if all reservation requests are known beforehand (no update of solution possible)"""
    def __init__(self, fleetctrl : FleetControlBase, operator_attributes : dict, dir_names : dict, solver : str="Gurobi"):
        super().__init__(fleetctrl, operator_attributes, dir_names, solver=solver)

    def _create_sorted_request_batches(self, sim_time: int) -> List[ReservationRequestBatch]:
        LOG.debug("specrrcluster batch optimization!")
        # batch the requests
        t = time.time()
        request_batches = self._batch_requests(sim_time)
        LOG.debug("{} batches with max size {} and {} requests created! -> took {}".format(len(request_batches), self.max_batch_size, len(self.active_reservation_requests), time.time() - t))
        # create schedules in batches
        t = time.time()
        batch_rg_list = []
        last_routing_time = sim_time
        for i, rid_list in enumerate(request_batches):
            LOG.debug(" -> create request batch {}/{} with {} requests".format(i, len(request_batches), len(rid_list)))
            if self._use_own_routing_engine:
                batch_start_time = min([self.active_reservation_requests[rid].get_o_stop_info()[1] for rid in rid_list])
                if batch_start_time is not None:
                    for t in range(last_routing_time, batch_start_time):
                        self.routing_engine.update_network(t)
                    last_routing_time = batch_start_time
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
        
        if self._use_own_routing_engine:
            self.routing_engine.reset_network(sim_time)
        
        return merged_batch_rg_list
    
    def _batch_requests(self, sim_time) -> List[List[Any]]:
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
        #rid_list = sorted([rid for rid, rq inself.active_reservation_requests.keys())
        unrevealed_rqs = [prq for rid, prq in self.active_reservation_requests.items() if self._reavealed_rids.get(rid) is None]
        sorted_rqs = sorted(unrevealed_rqs, key=lambda x:x.get_o_stop_info()[1])
        rid_to_index = {}
        adj_mat = np.zeros(shape=(len(sorted_rqs), len(sorted_rqs)))
        
        last_routing_time = sim_time
        for w, prq1 in enumerate(sorted_rqs):
            rid1 = prq1.get_rid_struct()
            rid_to_index[rid1] = w
            
            if self._use_own_routing_engine:
                batch_start_time = prq1.get_o_stop_info()[1]
                if batch_start_time is not None:
                    for t in range(last_routing_time, batch_start_time):
                        self.routing_engine.update_network(t)
                    last_routing_time = batch_start_time            
            
            for v in range(w, len(sorted_rqs)):
                prq2 = sorted_rqs[v]
                rid2 = prq2.get_rid_struct()
                rid_to_index[rid2] = v
                if check_shared_rr(prq1, prq2, self.routing_engine, self.fleetctrl.const_bt):
                    adj_mat[w,v] = 1
                    adj_mat[v,w] = 1
                elif check_shared_rr(prq2, prq1, self.routing_engine, self.fleetctrl.const_bt):
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
                    spec_adjs, spec_rid_to_indexs = spectral_clustering(c_adj, c_rid_to_index, N_clusters, n_threads=self.fleetctrl.n_cpu)
                    for spec_adj, spec_rid_to_index in zip(spec_adjs, spec_rid_to_indexs):
                        #print("spec", spec_adj.shape)
                        #print(spec_adj)
                        clustering_recursive(spec_adj, spec_rid_to_index, max_rids_per_cluster)                                
        clustering_recursive(adj_mat, rid_to_index, self.max_batch_size)
        t_cluster = time.time() - t
        t = time.time()
            
        list_request_batches = [list(cluster_rid_to_index.keys()) for cluster_rid_to_index in final_rid_to_index]
        def earliest_batch_start_time(list_rids):
            return min( (self.active_reservation_requests[rid].get_o_stop_info()[1] for rid in list_rids) )
        list_request_batches = sorted(list_request_batches, key=lambda x:earliest_batch_start_time(x))
        
        LOG.info(f"reservation batching took: {t_rr}s for rr computation | {t_cluster}s for creating clusters")
            
        if self._use_own_routing_engine:
            self.routing_engine.reset_network(sim_time)
            
        return list_request_batches
    