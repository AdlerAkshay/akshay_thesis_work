import os
import sys
import numpy as np
import pandas as pd
import time
import networkx as nx
from src.simulation.Offers import TravellerOffer
from src.fleetctrl.FleetControlBase import FleetControlBase
from src.routing.NetworkBase import NetworkBase
from src.fleetctrl.planning.VehiclePlan import PlanStopBase, RoutingTargetPlanStop
from src.fleetctrl.planning.PlanRequest import PlanRequest
from dev.fleetctrl.reservation.RevelationHorizonBase import RevelationHorizonBase
from dev.fleetctrl.reservation.misc.RequestGroup import RequestGroup, QuasiVehiclePlan, QuasiVehicle, VehiclePlanSupportingPoint, rg_key, get_lower_keys
from src.misc.globals import *

from typing import Any, Dict, List, Tuple

import logging
LOG = logging.getLogger(__name__)

NEW_VID_PENALTY = 1000000   # penalty for introducing a new vehicle in case a match between batches is not possible

class SharedRequestGroup(RequestGroup):
    """ this class is a collection of possible routes serving a set of requests.
    It can test if a completely shared route has been found. i.e. a route where the occupancy would become 0 only at the start or the end of the route"""
    def __init__(self, request_to_insert, routing_engine, std_bt, add_bt, vehicle_capacity, earliest_start_time = -1, lower_request_group=None, list_quasi_vehicle_plans = None):
        super().__init__(request_to_insert, routing_engine, std_bt, add_bt, vehicle_capacity, earliest_start_time, lower_request_group, list_quasi_vehicle_plans)
        self.list_rqs = [request_to_insert]
        self.start_end_constraints = None
        if lower_request_group is not None:
            self.list_rqs += lower_request_group.list_rqs[:]
            
    def has_completely_shared_route(self):
        shared_found = False
        for veh_plan in self.list_quasi_vehicle_plans:
            occ = 0
            is_shared = True
            for i in range(len(veh_plan.list_plan_stops)-1):
                occ += veh_plan.list_plan_stops[i].get_change_nr_pax()
                if occ == 0:
                    is_shared = False
                    break
            if is_shared:
                shared_found = True
                break
        return shared_found
    
    def get_start_end_constraints(self, obj_fuct, routing_engine, rq_dict):
        if self.start_end_constraints is None:
            self.start_end_constraints = self.return_best_plan(obj_fuct, routing_engine, rq_dict).get_start_end_constraints()
        return self.start_end_constraints

class mTSP_Node():
    def __init__(self, n_id, **attr):
        self.n_id = n_id
        self.att_dict = attr
        self.successors = {}
        self.predecessors = {}
        self.successors_within_cut = {}
        self.predecessors_within_cut = {}
        
    def __getitem__(self, att):
        return self.att_dict[att]
        
class mTSP_Edge():
    def __init__(self, **attr):
        self.att_dict = attr
        
    def __getitem__(self, att):
        return self.att_dict[att]
class mTSP_Graph():
    def __init__(self):
        self.nodes = {}
        self.edges = {}
        
    def add_node(self, node_id, **attr):
        self.nodes[node_id] = mTSP_Node(node_id, **attr)
        
    def add_edge(self, start_node_id, end_node_id, within_cut, **attr):
        if not self.nodes.get(start_node_id):
            self.add_node(start_node_id)
        if not self.nodes.get(end_node_id):
            self.add_node(end_node_id)
        self.edges[(start_node_id, end_node_id)] = mTSP_Edge(**attr)
        self.nodes[start_node_id].successors[end_node_id] = 1
        self.nodes[end_node_id].predecessors[start_node_id] = 1
        if within_cut:
            self.nodes[start_node_id].successors_within_cut[end_node_id] = 1
            self.nodes[end_node_id].predecessors_within_cut[start_node_id] = 1
        
    def remove_node(self, node_id):
        for next_node_id in self.nodes[node_id].successors.keys():
            del self.edges[(node_id, next_node_id)]
            del self.nodes[next_node_id].predecessors[node_id]
            try:
               del self.nodes[next_node_id].predecessors_within_cut[node_id]
            except:
                pass 
        for prev_node_id in self.nodes[node_id].predecessors.keys():
            del self.edges[(prev_node_id, node_id)]
            del self.nodes[prev_node_id].successors[node_id]
            try:
               del self.nodes[prev_node_id].successors_within_cut[node_id]
            except:
                pass 
            
    def successors(self, node_id):
        return self.nodes[node_id].successors.keys()
    
    def predecessors(self, node_id):
        return self.nodes[node_id].predecessors.keys()
    
    def successors_cut(self, node_id):
        return self.nodes[node_id].successors_within_cut.keys()
    
    def predecessors_cut(self, node_id):
        return self.nodes[node_id].predecessors_within_cut.keys()
        

class GraphContractionTSP(RevelationHorizonBase):
    """ this algorithm batches reservation requests in batch of sizes specified by the input parameter "op_res_batch_size"
    within the batches all possible requests groups are calculated (V2RB without a vehicle)
    in the assignement step, these batches are connected one after another starting with the first one
    the connection is made by solving a maximum priority matching problem to schedule request groups within batches
    the batches (requests) are in the current version sorted by their earliest pick-up time
    this method is currently only stable if all reservation requests are known beforehand (no update of solution possible)"""
    def __init__(self, fleetctrl : FleetControlBase, operator_attributes : dict, dir_names : dict, solver : str="Gurobi"):
        super().__init__(fleetctrl, operator_attributes, dir_names, solver=solver)

        self.solver = solver

        self.local_graph_max_depth = operator_attributes[G_RA_RES_LG_MAX_DEPTH]
        self.local_graph_cut_time = operator_attributes[G_RA_RES_LG_MAX_CUT]
        
    def _batch_optimisation(self, sim_time):
        LOG.debug("reservation GraphContractionTSP optimization!")
        if len(self._unprocessed_rids) != 0:
            # shared rr graph
            LOG.info("search for res solution at time {}".format(sim_time))
            t = time.time()
            rr_g = self._create_shared_rr_graph()
            # copy graph (this will be contracted)
            g_contract = nx.Graph()
            for node in rr_g.nodes:
                g_contract.add_node(node, rg=rr_g.nodes[node]["rg"])
            for edge in rr_g.edges:
                g_contract.add_edge(edge[0], edge[1], rg=rr_g.edges[edge]["rg"], obj=rr_g.edges[edge]["obj"])
            LOG.info("create rr_g took {}".format(time.time() -t))
            t = time.time()
            g_mTSP = self._create_mTSP_graph()
            LOG.info("create mTSP took {}".format(time.time() -t))
            t = time.time()
            rr_g, g_mTSP = self._contract_graphs(rr_g, g_mTSP)
            LOG.info("contraction took {}".format(time.time() -t))
            t = time.time()
            overall_cfv, routes = self._solve_mTSP(g_mTSP, self.fleet_size)
            LOG.info("optimization took {}".format(time.time() -t))
            t = time.time()
            self._plan_id_to_off_plan = {}
            for plan_id, r in enumerate(routes):
                list_planstops = []
                for rg_key in r:
                    if rg_key == -1:
                        continue
                    rg: SharedRequestGroup = g_mTSP.nodes[rg_key]["rg"]
                    best_plan = rg.return_best_plan(self.fleetctrl.vr_ctrl_f, self.routing_engine, self.active_reservation_requests)
                    list_planstops += best_plan.list_plan_stops
                self._plan_id_to_off_plan[plan_id] = QuasiVehiclePlan(self.routing_engine, list_planstops, self.vehicle_capacity)
                LOG.debug("{} -> {}".format(plan_id, self._plan_id_to_off_plan[plan_id]))   
            self._unprocessed_rids = {} 
            LOG.info("route creation took {}".format(time.time() -t))
        
    def _create_shared_rr_graph(self):
        """ this function initiallize the shared rr-graph
        Define shared rr-graph:
        - Nodes correspond to lower requestgroup (initially grade 1 (only the request))
        - Edges correspond to shared requestgroup (shared between start and end node (initially grade 2: the shared route between the two requests))
        """
        # 1) fast filter of possible rr pairs for speedup
        preproc_rr = {}
        c = 0
        for rid1, prq1 in self.active_reservation_requests.items():
            c += 1
            pu_earl_1 = prq1.t_pu_earliest
            pu_late_1 = prq1.t_do_latest
            if c%100 == 0:
                LOG.debug(f"preproc {c} / {len(self.active_reservation_requests)}")
            for rid2, prq2 in self.active_reservation_requests.items():
                if rid1 == rid2:
                    continue
                key = tuple(sorted([rid1, rid2]))
                if preproc_rr.get(key):
                    continue
                pu_earl_2 = prq2.t_pu_earliest
                pu_late_2 = prq2.t_do_latest
                if (pu_earl_1 <= pu_earl_2 and pu_late_1 >= pu_earl_2) or (pu_earl_2 <= pu_earl_1 and pu_late_2 >= pu_earl_1):
                    preproc_rr[key] = 1
        # 2) search for completely shared routes and create shared rr-graph
        rr_g = nx.Graph()
        for rid, prq in self.active_reservation_requests.items():
            rg = SharedRequestGroup(prq, self.routing_engine, self.fleetctrl.const_bt, self.fleetctrl.add_bt, self.vehicle_capacity)
            rr_g.add_node((rid,), rg = rg)
        c = 0
        for rid1, rid2 in preproc_rr.keys():
            c += 1
            if c%10000 == 0:
                LOG.debug(f"shared {c} / {len(preproc_rr)}")
            prq1 = self.active_reservation_requests[rid1]
            prq2 = self.active_reservation_requests[rid2]
            node = (rid1,) #g.nodes[(rid1,)]
            node2 = (rid2,) #g.nodes[(rid2,)]
            new_rg = SharedRequestGroup(rr_g.nodes[node2]["rg"].list_rqs[0], self.routing_engine, self.fleetctrl.const_bt, self.fleetctrl.add_bt, self.vehicle_capacity, lower_request_group=rr_g.nodes[node]["rg"])
            if new_rg.is_feasible():
                if new_rg.has_completely_shared_route():
                    obj = new_rg.return_objective(self.fleetctrl.vr_ctrl_f, self.routing_engine, self.active_reservation_requests)
                    rr_g.add_edge( node, node2, rg = new_rg, obj = obj)
        return rr_g
    
    def _create_mTSP_graph(self):
        """
        Define mTSP-Graph (graph with items the can be scheduled to solve an multi vehicle travelling salesman problem)
        - nodes correspond to requests or shared request groups
        - edges correspond to possibility of scheduling
            - edges parameters:
                - obj (distance)
                - time horizon (end time to start time)
        - will also be contracted when g_contract is contract
        - used for final optimisation problem of scheduling (mTSP)
        """
        g_mTSP = mTSP_Graph()
        LOG.debug("create mTSP graph")
        n = 0
        for rid, prq in self.active_reservation_requests.items():
            rg = SharedRequestGroup(prq, self.routing_engine, self.fleetctrl.const_bt, self.fleetctrl.add_bt, self.vehicle_capacity)
            g_mTSP.add_node((rid,), rg = rg)
            n += 1
        c = 0
        for s in g_mTSP.nodes.keys():
            for e in g_mTSP.nodes.keys():
                if s==e:
                    continue
                s_rg : SharedRequestGroup = g_mTSP.nodes[s]["rg"]
                e_rg : SharedRequestGroup = g_mTSP.nodes[e]["rg"]
                s_start_pos, s_start_time, s_end_pos, s_end_time = s_rg.get_start_end_constraints(self.fleetctrl.vr_ctrl_f, self.routing_engine, self.active_reservation_requests)
                e_start_pos, e_start_time, e_end_pos, e_end_time = e_rg.get_start_end_constraints(self.fleetctrl.vr_ctrl_f, self.routing_engine, self.active_reservation_requests)
                _, tt, dis = self.routing_engine.return_travel_costs_1to1(s_end_pos, e_start_pos)
                if e_start_time - s_end_time >= tt:
                    within_cut = True
                    if self.local_graph_cut_time is not None and e_start_time - s_end_time > self.local_graph_cut_time:
                        within_cut = False
                    g_mTSP.add_edge(s, e, within_cut, obj = dis, time_horizon = e_start_time - s_end_time)
                    c += 1
        LOG.debug("mTSP graph with {} nodes and {} edges".format(n, c))
        return g_mTSP
                    
    def _merge_sharedRequestGroups(self, rg1:SharedRequestGroup, rg2:SharedRequestGroup) -> SharedRequestGroup:
        """ this function tries to merge two request groups into one (feasible routes that serve all requests within rg1 and rg2)
        if no feasible route for the merged request group is found, None is returned
        else this function returns the merged request groupe"""
        if len(rg1.list_rqs) >= len(rg2.list_rqs):
            o_rg = rg1
            d_rg = rg2
        else:
            o_rg = rg2
            d_rg = rg1
        rqs = d_rg.list_rqs
        new_rg = SharedRequestGroup(rqs[0], self.routing_engine, self.fleetctrl.const_bt, self.fleetctrl.add_bt, self.vehicle_capacity, lower_request_group=o_rg)
        if len(rqs) > 1 and new_rg.is_feasible():
            for rq in rqs[1:]:
                new_rg = SharedRequestGroup(rq, self.routing_engine, self.fleetctrl.const_bt, self.fleetctrl.add_bt, self.vehicle_capacity, lower_request_group=new_rg)
                if not new_rg.is_feasible():
                    break
        if not new_rg.is_feasible() or not new_rg.has_completely_shared_route():
            return None
        else:
            return new_rg
        
    def _check_local_graph(self, g_mTSP, rr_g, edge, max_depth = 3, cut_time = None):
        """ this function checks the local graph if an contraction would """
        #LOG.debug("check local graph of", edge)
        t = time.time()
        s_node = edge[0]
        e_node = edge[1]
        # local graph
        local_base_graph = nx.DiGraph()
        local_base_graph.add_node(s_node, rg = g_mTSP.nodes[s_node]["rg"])
        local_base_graph.add_node(e_node, rg = g_mTSP.nodes[e_node]["rg"])
        node_depth_list = [(s_node, 0), (e_node, 0)]
        for cur_node, depth in node_depth_list:
            if depth >= max_depth:
                break
            for next_node in g_mTSP.successors_cut(cur_node):
                n_edge = (cur_node, next_node)
                if local_base_graph.has_edge(n_edge[0], n_edge[1]):
                    continue
                if not local_base_graph.has_node(next_node):
                    local_base_graph.add_node(next_node, rg = g_mTSP.nodes[next_node]["rg"])
                local_base_graph.add_edge(n_edge[0], n_edge[1], time_horizon = g_mTSP.edges[n_edge]["time_horizon"], obj=g_mTSP.edges[n_edge]["time_horizon"])
                node_depth_list.append( (next_node, depth+1) )
            for next_node in g_mTSP.predecessors_cut(cur_node):
                n_edge = (next_node, cur_node)
                if local_base_graph.has_edge(n_edge[0], n_edge[1]):
                    continue
                if not local_base_graph.has_node(next_node):
                    local_base_graph.add_node(next_node, rg = g_mTSP.nodes[next_node]["rg"])
                local_base_graph.add_edge(n_edge[0], n_edge[1], time_horizon = g_mTSP.edges[n_edge]["time_horizon"], obj=g_mTSP.edges[n_edge]["time_horizon"])
                node_depth_list.append( (next_node, depth+1) )
        # nx.draw_networkx(local_base_graph)
        # plt.show()
        # local graph with edge replaced by rg
        
        local_contr_graph = nx.DiGraph()
        for n in local_base_graph.nodes():
            if n == s_node or n == e_node:
                continue
            local_contr_graph.add_node(n, rg = g_mTSP.nodes[n]["rg"])
        for o_edge in local_base_graph.edges():
            if o_edge[0] == s_node or o_edge[1] == e_node or o_edge[1] == s_node or o_edge[0] == e_node:
                continue
            local_contr_graph.add_edge(o_edge[0], o_edge[1], time_horizon = local_base_graph.edges[o_edge]["time_horizon"], obj=local_base_graph.edges[o_edge]["time_horizon"])
        edge_rg = rr_g.edges[edge]["rg"]
        new_node = tuple(sorted(list(edge[0]) + list(edge[1])))
        local_contr_graph.add_node(new_node, rg=edge_rg)
        for n in local_contr_graph.nodes():
            n_rg : SharedRequestGroup = local_contr_graph.nodes[n]["rg"]
            s_start_pos, s_start_time, s_end_pos, s_end_time = edge_rg.get_start_end_constraints(self.fleetctrl.vr_ctrl_f, self.routing_engine, self.active_reservation_requests)
            e_start_pos, e_start_time, e_end_pos, e_end_time = n_rg.get_start_end_constraints(self.fleetctrl.vr_ctrl_f, self.routing_engine, self.active_reservation_requests)
            _, tt, dis = self.routing_engine.return_travel_costs_1to1(s_end_pos, e_start_pos)
            if e_start_time - s_end_time >= tt:
                local_contr_graph.add_edge(new_node, n, obj = dis, time_horizon = e_start_time - s_end_time)
            _, tt, dis = self.routing_engine.return_travel_costs_1to1(e_end_pos, s_start_pos)
            if s_start_time - e_end_time >= tt:
                local_contr_graph.add_edge(n, new_node, obj = dis, time_horizon = s_start_time - e_end_time)
        # nx.draw_networkx(local_contr_graph)
        # plt.show()
        LOG.debug(f"local base graph: {len(local_base_graph.nodes)} nodes and {len(local_base_graph.edges)} local contr graph: {len(local_contr_graph.nodes)} nodes and {len(local_contr_graph.edges)}")
        LOG.debug(f"graph creation took {time.time() - t}")
        t = time.time()
        base_sol, _ = self._solve_mTSP(local_base_graph, new_vehicle_cost=10000, N_vehicles=10000)
        contr_sol, _ = self._solve_mTSP(local_contr_graph, new_vehicle_cost=10000, N_vehicles=10000)
        
        LOG.debug(f"opt took {time.time() - t}")
        return contr_sol + rr_g.edges[edge]["obj"] - base_sol
    
    def _contract_graphs(self, rr_g, g_mTSP):
        """ this method is used to contract the request group graph an mTSP graph iterativly
        following steps are conducted:
        1) initially start with the uncontracted graphs
            rr_g: nodes are requests groups of grade one; edges are feasible requests groups of grade 2 connecting the requests
            g_mTSP: nodes are request groups of type 1; edges (directed) relate to costs scheduling two request groups one after another
        2) iteratively:
            2.1) select from rr_g: edge (request group with best obj value)
            2.2) check if replacing this edge by a node makes sense:
                2.2.1) create neighborhood local graphs of rr_g, g_mTSP
                2.2.2) create two options: a) status quo; b) contracted version: remove connecting nodes rgs from rr_g and g_mTSP and add the edge rg into both graphs
                2.2.3) solve the local mTSP and accept the contraction if obj improves
            2.3) if replacing makes sense: contract rr_g and g_mTSP:
                 remove connecting nodes rgs from rr_g and g_mTSP and add the edge rg into both graphs
                 else: remove the edge from rr_g
            2.4) repeat until no more edges in rr_g can be contracted
        """
        def contract_rr_g(next_edge, rg):
            new_node_id = tuple(sorted(list(next_edge[0]) + list(next_edge[1])))
            nodes_to_add.append((new_node_id, rg, len(new_node_id)))
            o_adj = list(nx.neighbors(rr_g, next_edge[0]))
            d_adj = list(nx.neighbors(rr_g, next_edge[1]))

            for o_node in set(o_adj + d_adj):
                if o_node == next_edge[0] or o_node == next_edge[1]:
                    continue
                o_rg = rr_g.nodes[o_node]["rg"]
                new_rg = self._merge_sharedRequestGroups(rg, o_rg)
                if new_rg is not None:
                    prev_obj1 = rg.return_objective(self.fleetctrl.vr_ctrl_f, self.routing_engine, self.active_reservation_requests)
                    prev_obj2 = o_rg.return_objective(self.fleetctrl.vr_ctrl_f, self.routing_engine, self.active_reservation_requests)
                    obj = new_rg.return_objective(self.fleetctrl.vr_ctrl_f, self.routing_engine, self.active_reservation_requests)
                    delta_obj = obj - prev_obj1 - prev_obj2
                    edges_to_add.append((new_node_id, o_node, new_rg, delta_obj))

            nodes_to_remove.append(next_edge[0])
            nodes_to_remove.append(next_edge[1])

            for x in nodes_to_add:
                rr_g.add_node(x[0], rg=x[1])
            for x in edges_to_add:
                rr_g.add_edge(x[0], x[1], rg=x[2], obj=x[3])
            for x in nodes_to_remove:
                try:
                    rr_g.remove_node(x)
                except:
                    pass
                
        def contract_g_mTSP(edge, edge_rg):
            possible_connectors = set( list(g_mTSP.predecessors(edge[0])) + list(g_mTSP.successors(edge[0])) +list(g_mTSP.predecessors(edge[1])) + list(g_mTSP.successors(edge[1])) )
            new_node = tuple(sorted(list(edge[0]) + list(edge[1])))
            g_mTSP.add_node(new_node, rg=edge_rg)
            s_start_pos, s_start_time, s_end_pos, s_end_time = edge_rg.get_start_end_constraints(self.fleetctrl.vr_ctrl_f, self.routing_engine, self.active_reservation_requests)
            for n in possible_connectors:
                n_rg : SharedRequestGroup = g_mTSP.nodes[n]["rg"]
                e_start_pos, e_start_time, e_end_pos, e_end_time = n_rg.get_start_end_constraints(self.fleetctrl.vr_ctrl_f, self.routing_engine, self.active_reservation_requests)
                _, tt, dis = self.routing_engine.return_travel_costs_1to1(s_end_pos, e_start_pos)
                within_cut = True
                if self.local_graph_cut_time is not None and e_start_time - s_end_time > self.local_graph_cut_time:
                    within_cut = False
                if e_start_time - s_end_time >= tt:
                    within_cut = True
                    if self.local_graph_cut_time is not None and e_start_time - s_end_time > self.local_graph_cut_time:
                        within_cut = False
                    g_mTSP.add_edge(new_node, n, within_cut, obj = dis, time_horizon = e_start_time - s_end_time)
                _, tt, dis = self.routing_engine.return_travel_costs_1to1(e_end_pos, s_start_pos)
                if s_start_time - e_end_time >= tt:
                    within_cut = True
                    if self.local_graph_cut_time is not None and s_start_time - e_end_time > self.local_graph_cut_time:
                        within_cut = False
                    g_mTSP.add_edge(n, new_node, within_cut, obj = dis, time_horizon = s_start_time - e_end_time)
            g_mTSP.remove_node(edge[0])
            g_mTSP.remove_node(edge[1])

        counter = 0
        draw = False
        do_print = False
        done_edge = {}
        while len(rr_g.edges) > 0:
            nodes_to_remove = []
            nodes_to_add = []
            edges_to_add = [] 
            edges_with_obj = [(edge, rr_g.edges[edge]["obj"]) for edge in rr_g.edges if done_edge.get(edge) is None] #list(g.edges)
            LOG.debug(f"round {counter} possible edges: {len(edges_with_obj)}")
            if len(edges_with_obj) == 0:
                break
            #next_edge = edges[np.random.randint(len(edges))]
            next_edge_with_obj = min(edges_with_obj, key=lambda x:x[1])

            next_edge=next_edge_with_obj[0]
            t = time.time()
            sol_diff = self._check_local_graph(g_mTSP, rr_g, next_edge, max_depth=self.local_graph_max_depth, cut_time=self.local_graph_cut_time)
            LOG.debug(f"check for edge {next_edge} took {time.time() - t} sol diff {sol_diff}.")
            done_edge[next_edge] = 1
            
            if sol_diff > 0:
                continue
            
            rg = rr_g.edges[next_edge]["rg"]
            t = time.time()
            contract_rr_g(next_edge, rg)
            contract_g_mTSP(next_edge, rg)
            LOG.debug("contraction took: {}".format(time.time() - t))
            
            counter += 1
            if counter % 50 == 0:
                LOG.info(f"current iteration: {counter} number edges: {len(rr_g.edges)} last improvement {sol_diff}.")
                
        return rr_g, g_mTSP
    
    def _solve_mTSP(self, graph, N_vehicles, new_vehicle_cost = 0):
        """ this function is used to solve the multi vehicle travelling salesman problem
        :param graph: directed graph with nodes connected that can be scheduled, an edge needs to have the attribute "obj" reflecting the cost for scheduling the two nodes
        :param N_vehicles: maxum number of vehicles
        :param new_vehicle_cost: cost for adding a new vehicle to the solution
        :return: tuple (overall cost function value, list of routes (list of node indices (first and last entry is -1 (depot)))"""
        if self.solver == "Gurobi":
            return self._solve_mTSP_gurobi(graph, N_vehicles, new_vehicle_cost=new_vehicle_cost)
        else:
            raise EnvironmentError("solver {} not defined in this class".format(self.solver)) 
        
    def _solve_mTSP_gurobi(self, graph, N_vehicles, new_vehicle_cost = 0):     
        import gurobipy as gp 
        from gurobipy import GRB

        with gp.Env(empty=True) as env:
            env.setParam('OutputFlag', 1)
            env.setParam('LogToConsole', 1)
            env.start()

            m = gp.Model("assignment", env = env)
            vars_dict = {}
            node_to_incoming_var = {}
            node_to_outgoing_var = {}

            for edge in graph.edges.keys():
                cost = graph.edges[edge]["obj"]
                vars_dict[edge] = cost
                try:
                    node_to_incoming_var[edge[1]][edge] = 1
                except KeyError:
                    node_to_incoming_var[edge[1]] = {edge: 1}
                try:
                    node_to_outgoing_var[edge[0]][edge] = 1
                except:
                    node_to_outgoing_var[edge[0]] = {edge : 1}

            start_depot_constr = {}
            end_depot_constr = {}
            for node in graph.nodes.keys():
                #for node in list(set(list(node_to_incoming_var.keys()) + list(node_to_outgoing_var.keys()))):
                vars_dict[(-1, node)] = new_vehicle_cost
                vars_dict[(node, -1)] = new_vehicle_cost
                start_depot_constr[(-1, node)] = 1
                end_depot_constr[(node, -1)] = 1
                try:
                    node_to_incoming_var[node][(-1, node)] = 1
                except KeyError:
                    node_to_incoming_var[node] = {(-1, node) : 1}
                try:
                    node_to_outgoing_var[node][(node, -1)] = 1
                except KeyError:
                    node_to_outgoing_var[node] = {(node, -1) : 1}
            
            # unassigned    
            unassigned_constr = {}
            uc = -2
            for node in graph.nodes.keys():
                #for node in list(set(list(node_to_incoming_var.keys()) + list(node_to_outgoing_var.keys()))):
                vars_dict[(uc, node)] = 10000000 * len(node)
                vars_dict[(node, uc)] = 10000000 * len(node)
                unassigned_constr[(uc, node)] = (node, uc)
                try:
                    node_to_incoming_var[node][(uc, node)] = 1
                except KeyError:
                    node_to_incoming_var[node] = {(uc, node) : 1}
                try:
                    node_to_outgoing_var[node][(node, uc)] = 1
                except KeyError:
                    node_to_outgoing_var[node] = {(node, uc): 1}
                uc -= 1

            vars = {}
            for varname, cost in vars_dict.items():
                vars[varname] = m.addVar(obj=cost, vtype=GRB.BINARY,
                                    name=f"{varname}")

            m.addConstr(sum(vars[x] for x in start_depot_constr.keys()) <= N_vehicles, name="start depot")
            m.addConstr(sum(vars[x] for x in end_depot_constr.keys()) <= N_vehicles, name = "end dept")
            for a, b in unassigned_constr.items():
                m.addConstr(vars[a] - vars[b] == 0, name = f"unnassigned {a[1]}")
            for node, outgoing_edges in node_to_outgoing_var.items():
                m.addConstr(sum(vars[x] for x in outgoing_edges.keys()) == 1, name=f"out node {node}")
            for node, incoming_edges in node_to_incoming_var.items():
                m.addConstr(sum(vars[x] for x in incoming_edges.keys()) == 1, name=f"in node {node}")
            m.write(r'C:\Users\ge37ser\Documents\Coding\TUM_VT_FleetSimulation\tum-vt-fleet-simulation\FleetPy\studies\reservation\dev\model.lp')
            m.optimize()
            
            vals = m.getAttr('X', vars)
            sols = {x : 1 for x, v in vals.items() if int(np.round(v)) == 1}
            LOG.debug("opt sols:")
            LOG.debug(f"{list(sols.keys())}")
            sol_graph = {}
            for s, e in sols.keys():
                if type(s) == int and s < -1:
                    LOG.debug(f"{e} unassigned!")
                elif type(e) == int and e < -1:
                    LOG.debug(f"{s} unassigned!")
                else:
                    try:
                        sol_graph[s][e] = 1
                    except KeyError:
                        sol_graph[s] = {e : 1}
            #LOG.debug(sol_graph)
            routes = []
            un = 0
            for s in sol_graph[-1].keys():
                routes.append([-1, s])
                e = list(sol_graph[s].keys())[0]
                routes[-1].append(e)
                while e != -1:
                    e = list(sol_graph[e].keys())[0]
                    routes[-1].append(e)
            overall_cfv = len(routes)*new_vehicle_cost
            overall_rg_cfvs = 0
            overall_conn_cfvs = 0
            for r in routes:
                list_plan_stops = []
                single_objs_sum = 0
                for rg_key in r:
                    if rg_key == -1:
                        continue
                    rg : SharedRequestGroup = graph.nodes[rg_key]["rg"]
                    c = rg.return_objective(self.fleetctrl.vr_ctrl_f, self.routing_engine, self.active_reservation_requests)
                    overall_rg_cfvs += c
                    single_objs_sum += c
                    best_plan = rg.return_best_plan(self.fleetctrl.vr_ctrl_f, self.routing_engine, self.active_reservation_requests)
                    if len(list_plan_stops) > 0:
                        last_pos = list_plan_stops[-1].get_pos()
                        next_pos = best_plan.list_plan_stops[0].get_pos()
                        _, tt, dis = self.routing_engine.return_travel_costs_1to1(last_pos, next_pos)
                        overall_conn_cfvs += dis
                    list_plan_stops += best_plan.list_plan_stops
                if len(list_plan_stops) > 0:
                    veh_p = QuasiVehiclePlan(self.routing_engine, list_plan_stops, self.vehicle_capacity)
                    cfv = veh_p.compute_obj_function(self.fleetctrl.vr_ctrl_f, self.routing_engine, self.active_reservation_requests)
                    #LOG.debug(cfv, "<->", single_objs_sum)
                    overall_cfv += cfv
            del m
        gp.disposeDefaultEnv()
        return overall_cfv, routes