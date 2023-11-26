# -------------------------------------------------------------------------------------------------------------------- #
# standard distribution imports
# -----------------------------
import os
import logging

# additional module imports (> requirements)
# ------------------------------------------
import pandas as pd
import numpy as np

# src imports
# -----------
from src.routing.NetworkBase import NetworkBase
from src.misc.functions import PiecewiseContinuousLinearFunction, PolynomialFunction

# -------------------------------------------------------------------------------------------------------------------- #
# global variables
# ----------------
from src.misc.globals import *
LOG = logging.getLogger(__name__)
EPS = 0.001
DEFAULT_MAX_X_SEARCH = 600
LARGE_INT = 10000000


# -------------------------------------------------------------------------------------------------------------------- #
# functions
# ---------
def read_node_line(columns):
    return Node(int(columns[G_NODE_ID]), int(columns[G_NODE_STOP_ONLY]),
                float(columns[G_NODE_X]), float(columns[G_NODE_Y]))


def create_cluster_tt_func(v0, v1):
    def cttf(avg_speed):
        if avg_speed > 0:
            return v0 * (1/avg_speed + 1/v1)
        else:
            return 1.0
    return cttf


# -------------------------------------------------------------------------------------------------------------------- #
# help classes
# ------------
class Node:
    def __init__(self, node_index, is_stop_only, pos_x, pos_y):
        # node_id -> node_obj
        self.to_nodes = {}
        self.from_nodes = {}
        #
        self.surround_next = {}  # max search -> set of d_node indices [dict_keys]
        self.surround_prev = {}  # max search -> set of o_node indices [dict_keys]
        self.node_index = node_index
        self.is_stop_only = is_stop_only
        self.pos_x = pos_x
        self.pos_y = pos_y

        self.zone_id = None

    def __str__(self):
        return str(self.node_index)

    def add_next_edge_to(self, other_node):
        self.to_nodes[other_node.node_index] = other_node

    def add_prev_edge_from(self, other_node):
        self.from_nodes[other_node.node_index] = other_node

    def must_stop(self):
        return self.is_stop_only

    def set_surround_prev(self, time_range, list_surround_prev): 
        if time_range > 600:
            return
        self.surround_prev[time_range] = list_surround_prev

    def set_surround_next(self, time_range, list_surround_next):
        if time_range > 600:
            return
        self.surround_next[time_range] = list_surround_next

    def set_zone_id(self, zone_id):
        self.zone_id = zone_id

    def get_zone_id(self):
        return self.zone_id


# -------------------------------------------------------------------------------------------------------------------- #
# module class
# ------------
class DynamicNFDNetwork(NetworkBase):
    """Network and Routing with dynamically changing travel times based on number of driving vehicles and NFD"""
    def __init__(self, network_name_dir, network_dynamics_file_name=None, scenario_time=None):
        """The network will be initialized.

        :param network_name_dir: name of the network_directory to be loaded
        :type network_name_dir: str
        :param type: determining whether the base or a pre-processed network will be used
        :type type: str
        :param scenario_time: applying travel times for a certain scenario at a given time in the scenario
        :type scenario_time: str
        :param network_dynamics_file_name: file-name of the network dynamics file
        :type network_dynamics_file_name: str
        """
        # TODO # after ISTTT es werden keine netzwerkdaten rausgeschrieben, wenn startzeit der simulation > 0
        super().__init__(network_name_dir, network_dynamics_file_name=network_dynamics_file_name, scenario_time=scenario_time)
        self.zones = None
        self.network_stat_f = None
        self.cluster_tt_factors = {}  # cluster -> cluster tt factor
        self.traveling_vehicles = {}  # sim_minute -> {}: cluster -> number of traveling vehicles
        self.start_time = 0
        self.sim_time = 0
        self.time_step = 1
        self.moving_average_bin = 1
        self.moving_average_duration = 1
        self.cluster_nfds = {}  # cluster -> nfd function (density -> flow)
        self.cluster_tt_funcs = {}  # cluster -> c-tt-factor function (average speed -> c-tt-factor)
        self.cluster_bg_traffic_df = {}  # cluster -> number of vehicles driving from/to outside of the operating area
        self.cluster_nw_length = {}  # cluster -> length
        self.cluster_toll_funcs = {}  # cluster -> toll function
        self.network_name_dir = network_name_dir
        # load network structure: nodes
        nodes_f = os.path.join(network_name_dir, "base", "nodes.csv")
        print(f"\t ... loading nodes from {nodes_f} ...")
        nodes_df = pd.read_csv(nodes_f)
        self.nodes = nodes_df.apply(read_node_line, axis=1).to_dict()
        self.number_nodes = len(self.nodes)
        self.tt = np.empty((self.number_nodes,self.number_nodes))
        self.td = np.empty((self.number_nodes, self.number_nodes))
        # dynamically build cluster travel time information (only once per simulation)
        # new: change to cluster_ff_tt and delete self.ctt
        # self.ctt = np.nan * np.ones((self.number_nodes,self.number_nodes))
        self.cluster_ff_tt = {} # (o,d) -> cluster -> cluster ff tt
        # network memory in order to not rebuild it for every customer, update each time step
        self.current_veh_pos_nodes = {} # node -> (origin pos, add_tt, add_td)
        # load network structure: edges
        edges_f = os.path.join(network_name_dir, "base", "edges.csv")
        print(f"\t ... loading edges from {edges_f} ...")
        edges_df = pd.read_csv(edges_f)
        for _, edge_info_row in edges_df.iterrows():
            o_node_index = edge_info_row[G_EDGE_FROM]
            o_node_obj = self.nodes[o_node_index]
            d_node_index = edge_info_row[G_EDGE_TO]
            d_node_obj = self.nodes[d_node_index]
            if o_node_index != d_node_index:
                o_node_obj.add_next_edge_to(d_node_obj)
                d_node_obj.add_prev_edge_from(o_node_obj)
        # load travel times
        self.load_tt_file(None)

    def add_init_data(self, start_time, time_step, moving_average_temporal_bin_size, moving_average_duration,
                      zone_system_obj, network_stat_f):
        """This method adds information specific to the initialization of the NFD-based dynamic network.

        :param start_time: simulation start time
        :type start_time: float
        :param time_step: time step of simulation
        :type time_step: float
        :param moving_average_temporal_bin_size: temporal resolution of veh count
        :type moving_average_temporal_bin_size: int
        :param moving_average_duration: duration of moving average
        :type moving_average_duration: int
        :param zone_system_obj: zone system determining the NFD clusters
        :type zone_system_obj: ZoneSystem
        :param network_stat_f: stat file to record the network travel time factors
        :type network_stat_f: str
        """
        self.start_time = start_time
        self.sim_time = start_time
        self.time_step = time_step
        self.moving_average_bin = moving_average_temporal_bin_size
        self.moving_average_duration = moving_average_duration
        self.zones = zone_system_obj
        for node_obj in self.nodes.values():
            node_obj.set_zone_id(self.zones.get_zone_from_node(node_obj.node_index))
        self.network_stat_f = network_stat_f
        with open(self.network_stat_f, "w") as fhout:
            fhout.write(f"{G_SIM_TIME},{G_ZONE_ZID},{G_ZONE_MOVE_VEH},{G_ZONE_DENSITY},{G_ZONE_FLOW},{G_ZONE_CTT},"
                        f"{G_ZONE_CTC}\n")
        # load cluster NFDs
        tmp_path = os.path.join(self.zones.zone_network_dir, "nfd_coefs_poly.csv")
        if os.path.isfile(tmp_path):
            # 1) polynomial fit exists
            fhin = open(tmp_path, "r")
            header = True
            for line in fhin:
                if header:
                    header = False
                    continue
                lc = line.strip().split(",")
                self.cluster_nfds[int(lc[0])] = PolynomialFunction([float(x) for x in lc[1:]])
            for zone_id in self.zones.zones:
                if self.cluster_nfds.get(zone_id) is None:
                    raise IOError(f"Did not find a cluster NFD for zone {zone_id} in {tmp_path}!")
        else:
            # 2) check for cluster NFDs
            for zone_id, zone_name in self.zones.general_info_df[G_ZONE_NAME].items():
                tmp_path = os.path.join(self.zones.zone_network_dir, f"nfd_reg_{zone_name}.csv")
                if os.path.isfile(tmp_path):
                    dict_density_flow_points = pd.read_csv(tmp_path, index_col=0, squeeze=True)
                    # ensure that (0,0) is a point of NFD
                    dict_density_flow_points[0] = 0
                    list_density_flow_points = [(d,f) for d,f in dict_density_flow_points.items()]
                    self.cluster_nfds[zone_id] = PiecewiseContinuousLinearFunction(list_density_flow_points)
                else:
                    raise IOError(f"Did not find the cluster NFD regression file {tmp_path} for zone {zone_id}")
        # load cluster travel time function
        tmp_path = os.path.join(self.zones.zone_network_dir, "cluster_tt_factors.csv")
        if os.path.isfile(tmp_path):
            tmp_df = pd.read_csv(tmp_path, index_col=0)
            for zone_id in self.zones.zones:
                tmp = tmp_df.loc[zone_id]
                v0 = tmp["v0"]
                v1 = tmp["v1"]
                self.cluster_tt_funcs[zone_id] = create_cluster_tt_func(v0, v1)
                self.cluster_tt_factors[zone_id] = 1.0
        else:
            raise IOError(f"Did not find information about cluster travel-time function definition {tmp_path}!")
        # load background traffic and network length (to compute density)
        tmp_path = os.path.join(self.zones.zone_network_dir, "background_traffic.csv")
        if os.path.isfile(tmp_path):
            self.cluster_bg_traffic_df = pd.read_csv(tmp_path, index_col=0)
            self.cluster_nw_length = self.cluster_bg_traffic_df["cluster_network_length"]
        else:
            raise IOError(f"Did not find background traffic file {tmp_path}!")
        for zone_id, zone_name in self.zones.general_info_df[G_ZONE_NAME].items():
            tmp_path = os.path.join(self.zones.zone_network_dir, f"toll_model_{zone_name}.csv")
            if os.path.isfile(tmp_path):
                dict_density_toll_points = pd.read_csv(tmp_path, index_col=0, squeeze=True)
                list_density_toll_points = [(d,t) for d,t in dict_density_toll_points.items()]
                self.cluster_toll_funcs[zone_id] = PiecewiseContinuousLinearFunction(list_density_toll_points)
            else:
                # TODO # after ISTTT: think about logger in network functions
                print(f"No Cluster toll function for {zone_name} found under {tmp_path}")

    def load_tt_file(self, scenario_time):
        """Loads new edge travel times from a file > since dynamism is controlled via network factor, only
        free flow travel times have to be read.

        :param scenario_time: applying travel times for a certain scenario at a given time in the scenario
        :type scenario_time: str
        """
        tt_table_f = os.path.join(self.network_name_dir, "ff", "tables", "nn_fastest_tt.npy")
        self.tt = np.load(tt_table_f)
        distance_table_f = os.path.join(self.network_name_dir, "ff", "tables", "nn_fastest_distance.npy")
        self.td = np.load(distance_table_f)

    def update_network(self, simulation_time, update_state=True):
        """This method can be called during simulations to update travel times (dynamic networks).

        :param simulation_time: time of simulation
        :type simulation_time: float
        :param update_state: if true, the cluster tt factors are reset
        :type update_state: bool
        :return: new_tt_flag True, if new travel times found; False if not
        :rtype: bool
        """
        self.sim_time = simulation_time
        # new: reset vehicle position nodes
        self.current_veh_pos_nodes = {}  # node -> (origin pos, add_tt, add_td)
        # new: surroundings, cluster_tt defined on free-flow velocity and therefore constant
        # # empty old node surroundings
        # for node_obj in self.nodes.values():
        #     node_obj.surround_next = {}
        #     node_obj.surround_prev = {}
        # # set self.ctt to nan
        # self.ctt = np.nan * np.ones((self.number_nodes, self.number_nodes))
        if update_state and simulation_time > self.start_time:
            # check moving vehicle counts per zone (offset by background traffic) -> density, flow (from NFD)
            cluster_sum_densities = {}
            cluster_list_velocity = {}
            cluster_record_info = {}
            counter = 0
            start_t = int(max(simulation_time-self.moving_average_duration, self.start_time) / self.moving_average_bin)
            end_t = max(start_t + 1, simulation_time // self.moving_average_bin, 0)
            # TODO # after ISTTT: hard code hour as time // 3600 (seconds always internal time?)
            sim_hour = simulation_time // 3600
            for t in range(start_t, end_t):
                counter += 1
                for cluster_id in self.zones.zones:
                    nr_vehicles = self.traveling_vehicles.get(t,{}).get(cluster_id, 0)
                    density = nr_vehicles / self.cluster_nw_length[cluster_id] +\
                                  self.cluster_bg_traffic_df.get(f"bg_density {sim_hour}", {}).get(cluster_id, 0)
                    flow = self.cluster_nfds[cluster_id].get_y(density)
                    if density > 0:
                        velocity = flow / density
                    else:
                        velocity = np.nan
                    try:
                        cluster_sum_densities[cluster_id] += density
                        cluster_list_velocity[cluster_id].append(velocity)
                    except:
                        cluster_sum_densities[cluster_id] = density
                        cluster_list_velocity[cluster_id] = [velocity]
                    if t == end_t - 1:
                        cluster_record_info[cluster_id] = [simulation_time, cluster_id, nr_vehicles, density, flow]
            if self.traveling_vehicles.get(start_t):
                del self.traveling_vehicles[start_t]
            # derive moving average of inverse network velocity and cluster_tt_factors from v1,v2 fit
            for cluster_id, list_velocity in cluster_list_velocity.items():
                moving_average_velocity = np.nanmean(list_velocity)
                if not np.isnan(moving_average_velocity):
                    cluster_tt_factor = self.cluster_tt_funcs[cluster_id](moving_average_velocity)
                else:
                    cluster_tt_factor = 1.0
                self.cluster_tt_factors[cluster_id] = cluster_tt_factor
                cluster_record_info[cluster_id].append(cluster_tt_factor)
            # determine cluster tolls
            cluster_toll_coefficients = {}
            for cluster_id, sum_densities in cluster_sum_densities.items():
                tmp_f = self.cluster_toll_funcs.get(cluster_id)
                if tmp_f is not None:
                    cluster_toll_coefficients[cluster_id] = tmp_f.get_y(sum_densities / counter)
                else:
                    cluster_toll_coefficients[cluster_id] = 0
                cluster_record_info[cluster_id].append(cluster_toll_coefficients[cluster_id])
            self.zones.set_current_toll_costs(rel_toll_cost_dict=cluster_toll_coefficients)
            # record cluster network information
            with open(self.network_stat_f, "a") as fhout:
                for cluster_id in self.zones.zones:
                    out_str = ",".join([str(x) for x in cluster_record_info.get(cluster_id, [])])
                    out_str += "\n"
                    fhout.write(out_str)
            return True
        else:
            # nothing happens in first time step
            return False

    def get_number_network_nodes(self):
        """This method returns a list of all street network node indices.

        :return: number of network nodes
        :rtype: int
        """
        return self.number_nodes

    def get_must_stop_nodes(self):
        """ returns a list of node-indices with all nodes with a stop_only attribute """
        return [n.node_index for n in self.nodes if n.must_stop()]

    def return_node_coordinates(self, node_index):
        """Returns the spatial coordinates of a node.

        :param node_index: id of node
        :type node_index: int
        :return: (x,y) for metric systems
        :rtype: list
        """
        node_obj = self.nodes[node_index]
        return node_obj.pos_x, node_obj.pos_y

    def return_position_coordinates(self, position_tuple):
        """Returns the spatial coordinates of a position.

        :param position_tuple: (o_node, d_node, rel_pos) | (o_node, None, None)
        :return: (x,y) for metric systems
        """
        if position_tuple[1] is None:
            return self.return_node_coordinates(position_tuple[0])
        else:
            c0 = np.array(self.return_node_coordinates(position_tuple[0]))
            c1 = np.array(self.return_node_coordinates(position_tuple[1]))
            c_rel = position_tuple[2] * c1 + (1 - position_tuple[2]) * c0
            return c_rel[0], c_rel[1]


    def get_section_infos(self, start_node_index, end_node_index):
        """Returns travel time and distance information of a section.

        :param start_node_index: index of start_node of section
        :param end_node_index: index of end_node of section
        :return: (travel time, distance); if no section between nodes (None, None)
        :rtype: list
        """
        tt = self.tt[start_node_index, end_node_index]
        c_cluster = self.nodes[start_node_index].get_zone_id()
        scaled_tt = tt * self.cluster_tt_factors.get(c_cluster, 1.0)
        if tt not in [None, np.nan, np.inf]:
            return scaled_tt, self.td[start_node_index, end_node_index]
        else:
            return None, None

    def return_route_infos(self, route, rel_start_edge_position, start_time=0):
        """This method returns the information travel information along a route. The start position is given by a
        relative value on the first edge [0,1], where 0 means that the vehicle is at the first node.

        :param route: list of node ids
        :type route: list
        :param rel_start_edge_position: float [0,1] determining the start position
        :type rel_start_edge_position: float
        :param start_time: can be used as an offset in case the route is planned for a future time
        :type start_time: float
        :return: (arrival time, distance to travel)
        :rtype: list
        """
        if not route or len(route) < 2:
            return 0,0
        rel_pos = (route[0], route[1], rel_start_edge_position)
        ff_time, sum_distance = self._get_section_overhead(rel_pos, False)
        c_cluster = self.nodes[route[0]].get_zone_id()
        sum_time = ff_time * self.cluster_tt_factors.get(c_cluster, 1.0)
        i = 2
        while i < len(route):
            scaled_route_infos = self.get_section_infos(route[i-1], route[i])
            sum_time += scaled_route_infos[0]
            sum_distance += scaled_route_infos[1]
            i += 1
        sum_time += start_time
        dest_pos = (route[-1], None, None)
        _, tt, td = self.return_travel_costs_1to1(rel_pos, dest_pos)
        return sum_time, sum_distance

    def assign_route_to_network(self, route, start_time, end_time=None, number_vehicles=1):
        """This method can be used for dynamic network models in which the travel times will be derived from the
        number of vehicles/routes assigned to the network.

        :param route: list of nodes
        :type route: list
        :param start_time: start of travel, can be used as an offset in case the route is planned for a future time
        :type start_time: float
        :param end_time: optional parameter; can be used to assign a vehicle to the cluster of the first node of the
                    route for a certain time
        :type end_time: float
        :param number_vehicles: optional parameter; can be used to assign multiple vehicles at once
        :type number_vehicles: int
        """
        if not route:
            return
        c_sim_time = start_time
        c_sim_min = int(start_time / self.moving_average_bin)
        c_route_index = 0
        # at least assign the vehicle for the current minute
        c_cluster = self.nodes[route[c_route_index]].get_zone_id()# self.zones.get_zone_from_node(route[c_route_index])
        self._assign_moving_vehicles_to_cluster(c_sim_min, c_cluster, number_vehicles)
        # time controlled assignment
        if end_time is not None:
            if end_time > start_time:
                for minute in range(c_sim_min+1, int(end_time / self.moving_average_bin)):
                    self._assign_moving_vehicles_to_cluster(minute, c_cluster, number_vehicles)
        # route assignment
        else:
            last_c_sim_min = c_sim_min
            for c_route_index in range(len(route) - 1):
                c_cluster = self.nodes[route[c_route_index]].get_zone_id()
                c_tt_factor = self.cluster_tt_factors.get(c_cluster, 1.0)
                next_time = c_sim_time + self.tt[route[c_route_index], route[c_route_index + 1]] * c_tt_factor
                next_c_sim_min = int(next_time / self.moving_average_bin)
                if next_c_sim_min != last_c_sim_min:
                    for minute in range(last_c_sim_min + 1, next_c_sim_min + 1):
                        self._assign_moving_vehicles_to_cluster(minute, c_cluster, number_vehicles)
                    last_c_sim_min = next_c_sim_min
                c_sim_time = next_time

    def return_travel_costs_1to1(self, origin_position, destination_position, customized_section_cost_function=None):
        """This method will return the travel costs of the fastest route between two nodes.

        :param origin_position: (origin_node_index, destination_node_index, relative_position) of current_edge
        :type origin_position: list
        :param destination_position: (origin_node_index, destination_node_index, relative_position) of destination_edge
        :type destination_position: list
        :param customized_section_cost_function: function to compute the travel cost of an section:
                which takes the args: (travel_time, travel_distance, current_dijkstra_node_index) -> cost_value
                if None: travel_time is considered as the cost_function of a section
        :type customized_section_cost_function: func
        :return: (cost_function_value, travel time, travel_distance) between the two nodes
        :rtype: list
        """
        origin_node, destination_node, add_tt, add_dist = self._get_od_nodes_and_section_overheads(origin_position,
                                                                                                  destination_position)
        c_cluster = origin_node.get_zone_id()# self.zones.get_zone_from_node(origin_node.node_index)
        scaled_add_tt = add_tt * self.cluster_tt_factors.get(c_cluster, 1.0)
        if origin_node == destination_node:
            if destination_position[1] is None:
                return 0, 0, 0
            else:
                return scaled_add_tt, scaled_add_tt, add_dist
        # new: use linear combination of (cluster free flow travel times * cluster travel time factors)
        # if not np.isnan(self.ctt[origin_node.node_index, destination_node.node_index]):
        #     scaled_route_tt = self.ctt[origin_node.node_index, destination_node.node_index]
        cluster_ff_tt = self.cluster_ff_tt.get((origin_node, destination_node))
        if cluster_ff_tt:
            scaled_route_tt = 0
            for cluster_id, cluster_tt_fac in self.cluster_tt_factors.items():
                scaled_route_tt += (cluster_tt_fac * cluster_ff_tt.get(cluster_id, 0.0))
        else:
            _, scaled_route_tt = self._lookup_dijkstra_1to1(origin_node, destination_node)
        cfv = scaled_route_tt + scaled_add_tt
        dist = self.td[origin_node.node_index, destination_node.node_index]
        return cfv, cfv, dist + add_dist

    def return_best_route_1to1(self, origin_position, destination_position, customized_section_cost_function=None):
        """This method will return the best route [list of node indices] between two nodes, where origin_position[0] and
        destination_position[1] or (destination_position[0] if destination_postion[1]==None) are included

        :param origin_position: (origin_node_index, destination_node_index, relative_position) of current_edge
        :type origin_position: list
        :param destination_position: (origin_node_index, destination_node_index, relative_position) of destination_edge
        :type destination_position: list
        :param customized_section_cost_function: function to compute the travel cost of an section
                which takes the args: (travel_time, travel_distance, current_dijkstra_node_index) -> cost_value
                if None: travel_time is considered as the cost_function of a section
        :type customized_section_cost_function: func
        :return: list of node-indices of the fastest route
        """
        origin_node, destination_node, add_tt, add_dist = self._get_od_nodes_and_section_overheads(origin_position,
                                                                                                  destination_position)
        c_cluster = origin_node.get_zone_id()# self.zones.get_zone_from_node(origin_node.node_index)
        scaled_add_tt = add_tt * self.cluster_tt_factors.get(c_cluster, 1.0)
        if origin_node == destination_node:
            if destination_position[1] is None:
                return [origin_node.node_index]
            else:
                return scaled_add_tt, scaled_add_tt, add_dist
        node_list, _ = self._lookup_dijkstra_1to1(origin_node, destination_node)
        if origin_node.node_index != origin_position[0]:
            node_list = [origin_position[0]] + node_list
        if destination_position[1] is not None:
            node_list.append(destination_position[1])
        return node_list

    def return_travel_costs_Xto1(self, list_origin_positions, destination_position, max_routes=None,
                                 max_cost_value=None, customized_section_cost_function=None):
        """This method will return a list of tuples of origin positions and cost values of the X fastest routes between
        a list of possible origin nodes and a certain destination node. Combinations that do not fulfill all constraints
        will not be returned.
        Specific to this framework: max_cost_value = None translates to max_cost_value = DEFAULT_MAX_X_SEARCH

        :param list_origin_positions: list of origin_positions
                (current_edge_origin_node_index, current_edge_destination_node_index, relative_position)
        :type list_origin_positions: list
        :param destination_position: (origin_node_index, destination_node_index, relative_position) of destination_edge
        :type destination_position: list
        :param max_routes: maximal number of fastest route triples that should be returned
        :type max_routes: int/None
        :param max_cost_value: latest cost function value of a route at destination to be considered as solution
                (max time if customized_section_cost_function == None)
        :type max_cost_value: float/None
        :param customized_section_cost_function: function to compute the travel cost of an section
                which takes the args: (travel_time, travel_distance, current_dijkstra_node_index) -> cost_value
                if None: travel_time is considered as the cost_function of a section
        :type customized_section_cost_function: func
        :return: list of (origin_position, cost_function_value, travel time, travel_distance) tuples
        :rtype: list
        """
        # LOG.debug("return_tc_Xto1 inputs {} {} {} {}".format(destination_position, max_cost_value, max_routes,
        #                                                      customized_section_cost_function))
        # get list of (origin nodes, add_tt, add_td) -> assign positions to origin_node
        if len(list_origin_positions) == 0:
            return []
        # assumption: d-pos is on node
        d_node = self.nodes[destination_position[0]]
        #  in general, handled by network attribute self.current_veh_pos_nodes -> only run this part if it is reset
        if not self.current_veh_pos_nodes:
            for o_pos in list_origin_positions:
                o_node, d_node, add_tt, add_dist = self._get_od_nodes_and_section_overheads(o_pos, destination_position)
                try:
                    self.current_veh_pos_nodes[o_node.node_index].add((o_pos, add_tt, add_dist))
                except:
                    self.current_veh_pos_nodes[o_node.node_index] = {(o_pos, add_tt, add_dist)}
        # set default max_cost_value if not available
        if max_cost_value is None:
            max_time = DEFAULT_MAX_X_SEARCH
        else:
            max_time = max_cost_value
        # check if prev valid for destination node
        set_d_surround_prev = d_node.surround_prev.get(max_time)
        # LOG.debug("return_tc_Xto1 max_time? {} set_d_surround_prev? {}".format(max_time, set_d_surround_prev))
        if set_d_surround_prev is not None:
            list_solutions = []
            # for o_node in o_node_pos_dict.keys():
            #     if o_node.node_index in set_d_surround_prev:
            intersect = set_d_surround_prev & self.current_veh_pos_nodes.keys()
            for o_node_index in intersect:
                # new: surroundings and travel times based on free-flow travel times
                # list_solutions.append((o_node.node_index, self.ctt[o_node.node_index, d_node.node_index]))

                cluster_ff_tt = self.cluster_ff_tt.get((o_node_index, d_node.node_index))
                if cluster_ff_tt:
                    scaled_route_tt = 0
                    for cluster_id, cluster_tt_fac in self.cluster_tt_factors.items():
                        scaled_route_tt += (cluster_tt_fac * cluster_ff_tt.get(cluster_id, 0.0))
                else:
                    _, scaled_route_tt = self._lookup_dijkstra_1to1(self.nodes[o_node_index], d_node)
                if not max_cost_value or scaled_route_tt <= max_cost_value:
                    list_solutions.append((o_node_index, scaled_route_tt))
        else:
            list_o_nodes = [self.nodes[x] for x in self.current_veh_pos_nodes.keys()]
            if max_routes == 1:
                list_solutions, _ = self._lookup_dijkstra_Xto1(list_o_nodes, d_node, max_time, best_only=True)
            else:
                if max_routes is not None and max_cost_value > 1000.0:
                    list_solutions, _ = self._lookup_dijkstra_Xto1_max_routes(list_o_nodes, d_node, max_time, max_routes)
                else:
                    list_solutions, _ = self._lookup_dijkstra_Xto1(list_o_nodes, d_node, max_time)
        # use max_cost_value with costs for positions on links
        return_list = []
        for o_node_id, ctt in list_solutions:
            dist = self.td[o_node_id, d_node.node_index]
            for o_pos, add_tt, add_dist in self.current_veh_pos_nodes[o_node_id]:
                if max_cost_value is None or ctt + add_tt <= max_cost_value:
                    return_list.append((o_pos, ctt + add_tt, ctt + add_tt, dist + add_dist))
        # sort if only limited amount of routes should be returned
        # if len(return_list) == 0:
        #     LOG.warning("no route found for return_travel_costs_Xto1 to target {} number origins {} time range {}".format(destination_position, len(list_origin_positions), max_cost_value))
        if max_routes:
            return_list.sort(key=lambda x: x[1])
            return_list = return_list[:max_routes]
        return return_list

    def return_best_route_Xto1(self, list_origin_positions, destination_position, max_cost_value=None,
                               customized_section_cost_function = None):
        """This method will return the best route between a list of possible origin nodes and a certain destination
        node. A best route is defined by [list of node_indices] between two nodes,
        while origin_position[0] and destination_position[1](or destination_position[0]
        if destination_position[1]==None) is included. Combinations that do not fulfill all constraints
        will not be returned.
        Specific to this framework: max_cost_value = None translates to max_cost_value = DEFAULT_MAX_X_SEARCH

        :param list_origin_positions: list of origin_positions
                (current_edge_origin_node_index, current_edge_destination_node_index, relative_position)
        :type list_origin_positions: list
        :param destination_position: (origin_node_index, destination_node_index, relative_position) of destination_edge
        :type destination_position: list
        :param max_cost_value: latest cost function value of a route at destination to be considered as solution
                (max time if customized_section_cost_function == None)
        :type max_cost_value: float/None
        :param customized_section_cost_function: function to compute the travel cost of an section
                which takes the args: (travel_time, travel_distance, current_dijkstra_node_index) -> cost_value
                if None: travel_time is considered as the cost_function of a section
        :type customized_section_cost_function: func
        :return: list of node-indices of the fastest routes
        :rtype: list
        """
        # call return_travel_costs_Xto1() to determine best origin position (this method checks whether data is
        # available or has to be computed) and then build 1-to-1 route from prev dict)
        list_route_infos = self.return_travel_costs_Xto1(list_origin_positions, destination_position, max_routes=1,
                                                         max_cost_value=max_cost_value)
        if not list_route_infos:
            return []
        origin_position, _, _, _ = list_route_infos[0]
        return self.return_best_route_1to1(origin_position, destination_position)

    def return_travel_costs_1toX(self, origin_position, list_destination_positions, max_routes=None,
                                 max_cost_value=None, customized_section_cost_function = None):
        """This method will return a list of tuples of destination node and travel time of the X fastest routes between
        a list of possible destination nodes and a certain origin node. Combinations that do not fulfill all constraints
        will not be returned.

        :param origin_position: (origin_node_index, destination_node_index, relative_position) of origin edge
        :type origin_position: list
        :param list_destination_positions: list of destination positions
                (origin_node_index, destination_node_index, relative_position) of destination_edge
        :type list_destination_positions: list
        :param max_routes: maximal number of fastest route triples that should be returned
        :type max_routes: int/None
        :param max_cost_value: latest cost function value of a route at destination to be considered as solution
                (max time if customized_section_cost_function == None)
        :param customized_section_cost_function: function to compute the travel cost of an section
                which takes the args: (travel_time, travel_distance, current_dijkstra_node_index) -> cost_value
                if None: travel_time is considered as the cost_function of a section
        :type customized_section_cost_function: func
        :return: list of (destination_position, cost_function_value, travel time, travel_distance) tuples
        :rtype: list
        """
        # get list of (destination nodes, add_tt, add_td) -> assign positions to origin_node
        d_node_pos_dict = {}
        for d_pos in list_destination_positions:
            o_node, d_node, add_tt, add_dist = self._get_od_nodes_and_section_overheads(origin_position, d_pos)
            try:
                d_node_pos_dict[d_node].add((d_pos, add_tt, add_dist))
            except:
                d_node_pos_dict[d_node] = {(d_pos, add_tt, add_dist)}
        list_d_nodes = list(d_node_pos_dict.keys())
        # set default max_cost_value if not available
        if max_cost_value is None:
            max_time = DEFAULT_MAX_X_SEARCH
        else:
            max_time = max_cost_value
        # check if prev valid for destination node
        set_o_surround_next = o_node.surround_next.get(max_time)
        if set_o_surround_next is not None:
            list_solutions = []
            for d_node in d_node_pos_dict.keys():
                if d_node.node_index in set_o_surround_next:
                    # new: surroundings and travel times based on free-flow travel times
                    # list_solutions.append((d_node.node_index, self.ctt[o_node.node_index, d_node.node_index]))
                    cluster_ff_tt = self.cluster_ff_tt[(o_node.node_index, d_node.node_index)]
                    scaled_route_tt = 0
                    for cluster_id, cluster_tt_fac in self.cluster_tt_factors.items():
                        scaled_route_tt += (cluster_tt_fac * cluster_ff_tt.get(cluster_id, 0.0))
                    if max_cost_value is None or scaled_route_tt <= max_cost_value:
                        list_solutions.append((d_node.node_index, scaled_route_tt))
        else:
            if max_routes == 1:
                list_solutions, _ = self._lookup_dijkstra_1toX(o_node, list_d_nodes, max_time, best_only=True)
            else:
                list_solutions, _ = self._lookup_dijkstra_1toX(o_node, list_d_nodes, max_time)
        # use max_cost_value with costs for positions on links
        return_list = []
        for d_node_id, ctt in list_solutions:
            dist = self.td[o_node.node_index, d_node_id]
            for d_pos, add_tt, add_dist in d_node_pos_dict[self.nodes[d_node_id]]:
                if not max_cost_value or ctt + add_tt <= max_cost_value:
                    return_list.append((d_pos, ctt + add_tt, ctt + add_tt, dist + add_dist))
        # sort if only limited amount of routes should be returned
        if max_routes:
            return_list.sort(key=lambda x: x[1])
            return_list = return_list[:max_routes]
        return return_list

    def return_best_route_1toX(self, origin_position, list_destination_positions, max_cost_value=None,
                               customized_section_cost_function = None):
        """This method will return the best route between a list of possible destination nodes and a certain origin
        node. A best route is defined by [list of node_indices] between two nodes,
        while origin_position[0] and destination_position[1](or destination_position[0]
        if destination_position[1]==None) is included. Combinations that do not fulfill all constraints
        will not be returned.
        Specific to this framework: max_cost_value = None translates to max_cost_value = DEFAULT_MAX_X_SEARCH

        :param origin_position: (origin_node_index, destination_node_index, relative_position) of origin edge
        :type origin_position: list
        :param list_destination_positions: list of destination positions
                (origin_node_index, destination_node_index, relative_position) of destination_edge
        :type list_destination_positions: list
        :param max_cost_value: latest cost function value of a route at destination to be considered as solution
                (max time if customized_section_cost_function == None)
        :type max_cost_value: float/None
        :param customized_section_cost_function: function to compute the travel cost of an section
                which takes the args: (travel_time, travel_distance, current_dijkstra_node_index) -> cost_value
                if None: travel_time is considered as the cost_function of a section
        :type customized_section_cost_function: func
        :return: list of node-indices of the fastest routes
        :rtype: list
        """
        # call return_travel_costs_Xto1() to determine best origin position (this method checks whether data is
        # available or has to be computed) and then build 1-to-1 route from prev dict)
        list_route_infos = self.return_travel_costs_1toX(origin_position, list_destination_positions, max_routes=1,
                                                         max_cost_value=max_cost_value)
        if not list_route_infos:
            return []
        destination_position, _, _, _ = list_route_infos[0]
        return self.return_best_route_1to1(origin_position, destination_position)

    def return_travel_cost_matrix(self, list_positions, customized_section_cost_function = None):
        """This method will return the cost_function_value between all positions specified in list_positions

        :param list_positions: list of positions to be computed
        :type list_positions: list
        :param customized_section_cost_function: function to compute the travel cost of an section
                which takes the args: (travel_time, travel_distance, current_dijkstra_node_index) -> cost_value
                if None: travel_time is considered as the cost_function of a section
        :type customized_section_cost_function: func
        :return: dictionary: (o_pos,d_pos) -> (cfv, tt, dist)
        :rtype: dict
        """
        # TODO # after ISTTT: test return_travel_cost_matrix()
        number_positions = len(list_positions)
        result_dict = {}
        for o_pos in list_positions:
            not_found = {} # d_node -> [(d_pos, add_tt, add_dist)]
            for d_pos in list_positions:
                o_node, d_node, add_tt, add_dist = self._get_od_nodes_and_section_overheads(o_pos, d_pos)
                tmp = self.cluster_ff_tt.get((o_node.node_index, d_node.node_index))
                if tmp:
                    route_tt = 0
                    for cluster_id, in_cluster_tt in tmp.items():
                        route_tt += self.cluster_tt_factors(cluster_id, 1.0) * in_cluster_tt
                    tt = route_tt + add_tt
                    result_dict[(o_pos, d_pos)] = (tt, tt, self.td[o_node.node_index, d_node.node_index] + add_dist)
                else:
                    try:
                        not_found[d_node].append((d_pos, add_tt, add_dist))
                    except KeyError:
                        not_found[d_node] = [(d_pos, add_tt, add_dist)]
            number_not_found = len(not_found)
            # run dijkstra 1toX if more than half of the entries are missing
            # run dijkstra Xto1 if less than half of the entries are missing
            # -> search from d_node of origin position (o_node, d_node, rel)
            if d_node is None:
                o_node = o_pos[0]
            else:
                o_node = o_pos[1]
            if number_not_found > number_positions / 2:
                list_destination_nodes = list(not_found.keys())
                self._lookup_dijkstra_1toX(o_node, list_destination_nodes, LARGE_INT)
            elif number_not_found > 0:
                list_origins = list(not_found.keys()) + [o_node]
                for d_node in not_found.keys():
                    self._lookup_dijkstra_1toX(list_origins, d_node, LARGE_INT)
            # collect data from now filled ctt-matrix
            for d_node, d_node_infos in not_found.items():
                tmp = self.cluster_ff_tt.get((o_node.node_index, d_node.node_index))
                for d_pos, add_tt, add_dist in d_node_infos:
                    route_tt = 0
                    for cluster_id, in_cluster_tt in tmp.items():
                        route_tt += self.cluster_tt_factors(cluster_id, 1.0) * in_cluster_tt
                    tt = route_tt + add_tt
                    result_dict[(o_pos, d_pos)] = (tt, tt, self.td[o_node.node_index, d_node.node_index] + add_dist)
            return result_dict

    def move_along_route(self, route, last_position, time_step, sim_vid_id=None, new_sim_time=None,
                         record_node_times=False):
        """This method computes the new position of a (vehicle) on a given route (node_index_list) from it's
        last_position (position_tuple). The first entry of route has to be the same as the first entry of last_position!
        Specific to this framework: count moving vehicles to street network density! make sure to do this before
        updating the network!

        :param route: list of node_indices of the current route
        :type route: list
        :param last_position: position_tuple of starting point
        :type last_position: list
        :param time_step: time [s] passed since last observed at last_position
        :type time_step: float
        :param sim_vid_id: id of simulation vehicle; required for simulation environments with external traffic simulator
        :type sim_vid_id: int
        :param new_sim_time: new time to coordinate simulation times
        :type new_sim_time: float
        :param record_node_times: if this flag is set False, the output list_passed_node_times will always return []
        :type record_node_times: bool
        :return: returns a tuple with
                i) new_position_tuple
                ii) driven distance
                iii) arrival_in_time_step [s]: -1 if vehicle did not reach end of route | time since beginning of time
                        step after which the vehicle reached the end of the route
                iv) list_passed_nodes: if during the time step a number of nodes were passed, these are
                v) list_passed_node_times: list of checkpoint times at the respective passed nodes
        """
        end_time = self.sim_time + time_step
        last_time = self.sim_time
        c_pos = last_position
        if c_pos[2] is None:
            c_pos = (c_pos[0], route[0], 0.0)
        list_passed_nodes = []
        list_passed_node_times = []
        arrival_in_time_step = -1
        driven_distance = 0
        #
        c_cluster = None
        last_dyn_step = None
        for i in range(len(route)):
            # if last_time is a new minute, assign a moving vehicle to the respective cluster
            tmp = int(last_time / self.moving_average_bin)
            if tmp != last_dyn_step:
                c_cluster = self.nodes[c_pos[0]].get_zone_id()# self.zones.get_zone_from_node(c_pos[0])
                if last_dyn_step is None:
                    self._assign_moving_vehicles_to_cluster(tmp, c_cluster)
                else:
                    for dyn_step in range(last_dyn_step + 1, tmp + 1):
                        self._assign_moving_vehicles_to_cluster(last_dyn_step, c_cluster)
                last_dyn_step = tmp
            # check remaining time on current edge
            if c_pos[2] is None:
                c_pos = (c_pos[0], route[i], 0)
            rel_factor = (1 - c_pos[2])
            c_edge_tt = rel_factor * self.cluster_tt_factors.get(c_cluster,1.0) * self.tt[c_pos[0], c_pos[1]]
            next_node_time = last_time + c_edge_tt
            if next_node_time > end_time:
                # move vehicle to final position of current edge
                end_rel_factor = c_pos[2] + (end_time - last_time) / c_edge_tt
                driven_distance += (end_rel_factor - c_pos[2]) * self.td[c_pos[0], c_pos[1]]
                c_pos = (c_pos[0], c_pos[1], end_rel_factor)
                arrival_in_time_step = -1
                break
            else:
                # move vehicle to next node/edge and record data
                driven_distance += rel_factor * self.td[c_pos[0], c_pos[1]]
                next_node = route[i]
                list_passed_nodes.append(next_node)
                if record_node_times:
                    list_passed_node_times.append(next_node_time)
                last_time = next_node_time
                c_pos = (next_node, None, None)
                arrival_in_time_step = last_time
        return c_pos, driven_distance, arrival_in_time_step, list_passed_nodes, list_passed_node_times

    # internal methods
    # ----------------
    def _get_section_overhead(self, position, traveled_from_start = True):
        """
        :param position: (current_edge_origin_node_index, current_edge_destination_node_index, relative_position)
        :param traveled_from_start: computes already traveled travel_time and distance,
                if False: computes rest travel time (relative_position -> 1.0-relative_position)
        :return: (rest travel time, rest travel distance)
        """
        if position[1] is None:
            return (0.0, 0.0)
        o_node_index = position[0]
        d_node_index = position[1]
        all_travel_time = self.tt[o_node_index, d_node_index]
        all_travel_distance = self.td[o_node_index, d_node_index]
        overhead_fraction = position[2]
        if not traveled_from_start:
            overhead_fraction = 1.0 - overhead_fraction
        # adapt to dynamic travel times!
        o_cluster = self.nodes[o_node_index].get_zone_id()# self.zones.get_zone_from_node(o_node_index)
        c_tt_factor = self.cluster_tt_factors.get(o_cluster,1.0)
        return all_travel_time * overhead_fraction * c_tt_factor, all_travel_distance * overhead_fraction

    def _get_od_nodes_and_section_overheads(self, origin_position, destination_position):
        """
        :param origin_position: (current_edge_origin_node_index, current_edge_destination_node_index, relative_position)
        :param destination_position: (destination_edge_origin_node_index, destination_edge_destination_node_index, relative_position)
        :return: (origin_node, destination_node, add_tt, add_dist)
        """
        trivial = False
        if origin_position[0] == destination_position[0] and origin_position[1] == destination_position[1]:
            if origin_position[2] is not None and destination_position[2] is not None:
                if origin_position[2] < destination_position[2]:
                    overhead = destination_position[2] - origin_position[2]
                    ov_tt, ov_dist = self._get_section_overhead((origin_position[0], origin_position[1], overhead))
                    return self.nodes[origin_position[0]], self.nodes[origin_position[0]], ov_tt, ov_dist
            elif origin_position[2] is None and destination_position[2] is None:
                return (self.nodes[origin_position[0]], self.nodes[origin_position[0]], 0.0, 0.0)
            elif origin_position[2] is None:
                ov_tt, ov_dist = self._get_section_overhead(destination_position)
                return self.nodes[origin_position[0]], self.nodes[origin_position[0]], ov_tt, ov_dist
            # last case is not trivial
        if not trivial:
            add_0_tt, add_0_dist = self._get_section_overhead(origin_position, False)
            add_1_tt, add_1_dist = self._get_section_overhead(destination_position)
            if origin_position[1] is not None:
                o_node = self.nodes[origin_position[1]]
            else:
                o_node = self.nodes[origin_position[0]]
            return o_node, self.nodes[destination_position[0]], add_0_tt + add_1_tt, add_0_dist + add_1_dist

    def _lookup_dijkstra_1to1(self, origin_node, destination_node):
        """This internal method computes the lookup Dijkstra between two nodes.

        :param origin_node: node object of origin
        :type origin_node: Node
        :param destination_node: node object of destination
        :type destination_node: Node
        :return: node_index_list, scaled_route_tt, distance
        :rtype: list
        """
        cluster_ff_tt = {} # cluster_id -> tt in this cluster
        current_node = origin_node
        node_list = [current_node.node_index]
        route_tt = 0.0
        scaled_route_tt = 0.0
        total_tt = self.tt[origin_node.node_index, destination_node.node_index]
        if total_tt in [None, np.nan, np.inf]:
            prt_str = f"There is no route from {origin_node} to {destination_node}"
            raise AssertionError(prt_str)
        while current_node != destination_node:
            c_cluster = current_node.get_zone_id() # self.zones.get_zone_from_node(current_node)
            c_tt_factor = self.cluster_tt_factors.get(c_cluster, 1.0)
            found_next_node = False
            for next_node_id, next_node_obj in current_node.to_nodes.items():
                # since complete travel time matrix is known, nodes on the route have to satisfy
                # A->B + B->C = A->C
                if next_node_obj.is_stop_only and next_node_obj != destination_node:
                    continue
                next_tt = self.tt[current_node.node_index, next_node_id]
                from_next_tt = self.tt[next_node_id, destination_node.node_index]
                if route_tt + next_tt + from_next_tt - total_tt < EPS:
                    found_next_node = True
                    node_list.append(next_node_id)
                    route_tt += next_tt
                    scaled_route_tt += (next_tt * c_tt_factor)
                    current_node = next_node_obj
                    # # add data to self.ctt
                    # self.ctt[origin_node.node_index, next_node_id] = scaled_route_tt
                    # add data to self.cluster_ff_tt
                    try:
                        cluster_ff_tt[c_cluster] += next_tt
                    except:
                        cluster_ff_tt[c_cluster] = next_tt
                    break
            if not found_next_node:
                prt_str = f"Could not find next node after current node {current_node} in search" \
                          f" of route from {origin_node} to {destination_node}; current node list: {node_list}"
                raise AssertionError(prt_str)
        self.cluster_ff_tt[(origin_node.node_index, destination_node.node_index)] = cluster_ff_tt.copy()
        return node_list, scaled_route_tt

    def _lookup_dijkstra_Xto1(self, list_origin_nodes, destination_node, max_time_value, best_only=False):
        """This internal method computes a backward lookup Dijkstra between a node and a list of origin nodes.

        :param list_origin_nodes: node objects of possible origins
        :type list_origin_nodes: list of Nodes
        :param destination_node: node object of destination
        :type destination_node: Node
        :param max_time_value: maximal search radius
        :type max_time_value: float
        :param best_only: if this option is selected, max_time_value can be adjusted according to the currently
                            best solution and used as stop criterion
        :type best_only: bool
        :return: tuple (list of (origin_node_index, scaled_route_tt), next_dict) where next_dict is a
                    dictionary that can be used to build routes
        :rtype: list
        """
        # LOG.debug("dijkstra xto1 check input and globals: max time {} Large int {} eps {}".format(max_time_value,
        #                                                                                           LARGE_INT, EPS))
        cluster_ff_tt = {} # cluster_id -> tt in this cluster
        number_origins = len(list_origin_nodes)
        visited = {}
        next_dict = {}
        solutions = []
        frontier = [destination_node]
        scaled_route_tt = {destination_node.node_index: 0.0}
        # break conditions: 1) no more frontier, 2) all solutions found
        while frontier:
            if len(solutions) == number_origins:
                break
            # either use deque.popleft() to start from left or list.pop() to start from right!
            current_node = frontier.pop()
            # it is possible that two paths have the exact same travel time and a node would be twice in the frontier
            if visited.get(current_node.node_index):
                continue
            c_node_scaled_tt = scaled_route_tt[current_node.node_index]
            if current_node in list_origin_nodes:
                # new: check if current travel time of solution is satisfying condition
                if scaled_route_tt[current_node.node_index] < max_time_value:
                    if best_only:
                        solutions = [(current_node.node_index, scaled_route_tt[current_node.node_index])]
                        max_time_value = scaled_route_tt[current_node.node_index]
                    else:
                        solutions.append((current_node.node_index, scaled_route_tt[current_node.node_index]))
            for prev_node_id, prev_node_obj in current_node.from_nodes.items():
                if prev_node_obj.is_stop_only and prev_node_obj not in list_origin_nodes:
                    continue
                c_node_tt = self.tt[current_node.node_index, destination_node.node_index]
                prev_tt = self.tt[prev_node_id, current_node.node_index]
                total_tt = self.tt[prev_node_id, destination_node.node_index]
                # check if the node is on a fastest free-flow travel-time path
                if c_node_tt + prev_tt - total_tt <= EPS:
                    p_cluster = prev_node_obj.get_zone_id()# self.zones.get_zone_from_node(prev_node_id)
                    p_tt_factor = self.cluster_tt_factors.get(p_cluster, 1.0)
                    scaled_tt = c_node_scaled_tt + prev_tt * p_tt_factor
                    # check if time constraint is kept; only add if smaller than max time value -> frontier breaks loop
                    # if scaled_tt <= max_time_value:
                    # new: make check dependent on free-flow travel time -> surrounding defined by ff travel time
                    if total_tt <= max_time_value:
                        frontier.append(prev_node_obj)
                        next_dict[prev_node_id] = current_node.node_index
                        scaled_route_tt[prev_node_id] = scaled_tt
            visited[current_node.node_index] = True
        # set list of found nodes only if len(solutions) != number_origins and node is stop_node!
        if len(solutions) != number_origins and destination_node.must_stop():
            destination_node.set_surround_prev(max_time_value, next_dict.keys())
            #exit()
        return solutions, next_dict

    def _lookup_dijkstra_1toX(self, origin_node, list_destination_nodes, max_time_value, best_only=False):
        """This internal method computes a forward lookup Dijkstra between a node and a list of destination nodes.

        :param origin_node: node object of origin
        :type origin_node: Nodes
        :param list_destination_nodes: node objects of possible destinations
        :type list_destination_nodes: list of Nodes
        :param max_time_value: maximal search radius
        :type max_time_value: float
        :param best_only: if this option is selected, max_time_value can be adjusted according to the currently
                            best solution and used as stop criterion
        :type best_only: bool
        :return: tuple (list of (destination_node_index, scaled_route_tt), next_dict) where next_dict is a
                    dictionary that can be used to build routes
        :rtype: list
        """
        cluster_ff_tt = {}  # cluster_id -> tt in this cluster
        number_destinations = len(list_destination_nodes)
        prev_dict = {}
        visited = {}
        solutions = []
        frontier = [origin_node]
        scaled_route_tt = {origin_node.node_index: 0.0}
        # break conditions: 1) no more fontier / all solutions found
        while frontier:
            if len(solutions) == number_destinations:
                break
            # either use deque.popleft() to start from left or list.pop() to start from right!
            current_node = frontier.pop()
            n_cluster = current_node.get_zone_id()# self.zones.get_zone_from_node(current_node.node_index)
            n_tt_factor = self.cluster_tt_factors.get(n_cluster, 1.0)
            # it is possible that two paths have the exact same travel time and a node would be twice in the frontier
            if visited.get(current_node.node_index):
                continue
            c_node_scaled_tt = scaled_route_tt[current_node.node_index]
            if current_node in list_destination_nodes:
                if scaled_route_tt[current_node.node_index] < max_time_value:
                    if best_only:
                        solutions = [(current_node.node_index, scaled_route_tt[current_node.node_index])]
                        max_time_value = scaled_route_tt[current_node.node_index]
                    else:
                        solutions.append((current_node.node_index, scaled_route_tt[current_node.node_index]))
            for next_node_id, next_node_obj in current_node.to_nodes.items():
                if next_node_obj.is_stop_only and next_node_obj not in list_destination_nodes:
                    continue
                c_node_tt = self.tt[origin_node.node_index, current_node.node_index]
                next_tt = self.tt[current_node.node_index, next_node_id]
                total_tt = self.tt[origin_node.node_index, next_node_id]
                # check if the node is on a fastest free-flow travel-time path
                if c_node_tt + next_tt - total_tt <= EPS:
                    scaled_tt = c_node_scaled_tt + next_tt * n_tt_factor
                    # check if time constraint is kept; only add if smaller than max time value -> frontier breaks loop
                    # if scaled_tt <= max_time_value:
                    # new: make check dependent on free-flow travel time -> surrounding defined by ff travel time
                    if total_tt <= max_time_value:
                        frontier.append(next_node_obj)
                        prev_dict[next_node_id] = current_node.node_index
                        scaled_route_tt[next_node_id] = scaled_tt
            visited[current_node.node_index] = True
        # set list of found nodes only if len(solutions) != number_destinations and node is stop_node!
        if len(solutions) != number_destinations and origin_node.must_stop():
            origin_node.set_surround_next(max_time_value, prev_dict.keys())
        return solutions, prev_dict

    def _assign_moving_vehicles_to_cluster(self, sim_min, cluster_id, number_vehicles=1):
        try:
            self.traveling_vehicles[sim_min][cluster_id] += number_vehicles
        except KeyError:
            try:
                self.traveling_vehicles[sim_min][cluster_id] = number_vehicles
            except KeyError:
                self.traveling_vehicles[sim_min] = {cluster_id: number_vehicles}

    def _lookup_dijkstra_Xto1_max_routes(self, list_origin_nodes, destination_node, max_time_value, max_routes, best_only=False):
        """This internal method computes a backward lookup Dijkstra between a node and a list of origin nodes. but breaks if max routes is reached

        :param list_origin_nodes: node objects of possible origins
        :type list_origin_nodes: list of Nodes
        :param destination_node: node object of destination
        :type destination_node: Node
        :param max_time_value: maximal search radius
        :type max_time_value: float
        :param max_routes: max number of routes to be computed
        :type max_routes: int
        :param best_only: if this option is selected, max_time_value can be adjusted according to the currently
                            best solution and used as stop criterion
        :type best_only: bool
        :return: tuple (list of (origin_node_index, scaled_route_tt), next_dict) where next_dict is a
                    dictionary that can be used to build routes
        :rtype: list
        """
        # LOG.debug("dijkstra xto1 check input and globals: max time {} Large int {} eps {}".format(max_time_value,
        #                                                                                           LARGE_INT, EPS))
        cluster_ff_tt = {} # cluster_id -> tt in this cluster
        number_origins = len(list_origin_nodes)
        visited = {}
        next_dict = {}
        solutions = []
        frontier = [destination_node]
        scaled_route_tt = {destination_node.node_index: 0.0}
        # break conditions: 1) no more frontier, 2) all solutions found
        while frontier:
            if len(solutions) == number_origins:
                break
            # either use deque.popleft() to start from left or list.pop() to start from right!
            current_node = frontier.pop()
            # it is possible that two paths have the exact same travel time and a node would be twice in the frontier
            if visited.get(current_node.node_index):
                continue
            c_node_scaled_tt = scaled_route_tt[current_node.node_index]
            if current_node in list_origin_nodes:
                # new: check if current travel time of solution is satisfying condition
                if scaled_route_tt[current_node.node_index] < max_time_value:
                    if best_only:
                        solutions = [(current_node.node_index, scaled_route_tt[current_node.node_index])]
                        max_time_value = scaled_route_tt[current_node.node_index]
                    else:
                        solutions.append((current_node.node_index, scaled_route_tt[current_node.node_index]))
                if len(solutions) >= max_routes:
                    break
            for prev_node_id, prev_node_obj in current_node.from_nodes.items():
                if prev_node_obj.is_stop_only and prev_node_obj not in list_origin_nodes:
                    continue
                c_node_tt = self.tt[current_node.node_index, destination_node.node_index]
                prev_tt = self.tt[prev_node_id, current_node.node_index]
                total_tt = self.tt[prev_node_id, destination_node.node_index]
                # check if the node is on a fastest free-flow travel-time path
                if c_node_tt + prev_tt - total_tt <= EPS:
                    p_cluster = prev_node_obj.get_zone_id()# self.zones.get_zone_from_node(prev_node_id)
                    p_tt_factor = self.cluster_tt_factors.get(p_cluster, 1.0)
                    scaled_tt = c_node_scaled_tt + prev_tt * p_tt_factor
                    # # add data to self.ctt
                    # self.ctt[prev_node_id, destination_node.node_index] = scaled_tt
                    # add data to self.cluster_ff_tt
                    # check if time constraint is kept; only add if smaller than max time value -> frontier breaks loop
                    # if scaled_tt <= max_time_value:
                    # new: make check dependent on free-flow travel time -> surrounding defined by ff travel time
                    if total_tt <= max_time_value:
                        frontier.append(prev_node_obj)
                        next_dict[prev_node_id] = current_node.node_index
                        scaled_route_tt[prev_node_id] = scaled_tt
            visited[current_node.node_index] = True
        return solutions, next_dict

    # def checkNetwork(self, considered_stops, target_pos): # TODO # wieder rausschmeißen!
    #     x = self.return_best_route_Xto1(considered_stops, target_pos,
    #                                                                               max_cost_value=LARGE_INT)
    #     print(x)
    #     y = self.return_travel_costs_Xto1(considered_stops, target_pos, max_routes=1, max_cost_value=LARGE_INT)
    #     print(y)
    #     exit()
    #     import sys
    #     sys.path.append(r'C:\Users\ge37ser\Documents\Projekte\EasyRide\AP2300\AP2320\scripts\routing')
    #     from NetworkBasic import Network
    #     nw = Network(self.network_name_dir)
    #     #nw2 = Network(r'C:\Users\ge37ser\Documents\Projekte\EasyRide\AP2300\AP2320\networks\MUNbene_withBPs_300_1_LHMArea_OVstations_reduced_myscript')
    #     nw2 = Network(r'C:\Users\ge37ser\Documents\Projekte\EasyRide\AP2300\AP2320\networks\MUNbene_withBPs_300_1_LHMArea_OVstations_reduced_myscript')
    #     bn = [n for n in nw.nodes if n.must_stop()]
    #     bn2 = [n for n in nw2.nodes if n.must_stop()]
    #     bn0 = [n for n in self.nodes.values() if n.must_stop()]
    #     all_node_pos = [nw.return_node_position(n.node_index) for n in nw.nodes]
    #     all_node_pos2 = [nw.return_node_position(n.node_index) for n in nw2.nodes]
    #     all_node_pos0 = [self.return_node_position(n.node_index) for n in self.nodes.values()]
    #
    #     start1 = target_pos# nw.return_node_position(12961)
    #     #start2 = nw.return_node_position(b.node_index)
    #     start0 = target_pos#nw.return_node_position(12961)
    #     s1 = nw.return_travel_costs_Xto1(considered_stops, start1)
    #     s1 = [x for x in s1 if x[1] < 99999999999.9]
    #     # s2 = nw2.return_travel_costs_1toX(start2, all_node_pos2)
    #     # s2 = [x for x in s2 if x[1] < 99999999999.9]
    #     s3 = self.return_travel_costs_Xto1(considered_stops, start0, max_cost_value=LARGE_INT)
    #     s3 = [x for x in s3 if x[1] < 99999999999.9]
    #     print("n {} || self: {}/{} | nw1: {}/{} || check ".format(12961, len(s3), len(self.nodes), len(s1), len(nw.nodes)))
    #     sol_dict_1 = {s[0] : s[1:] for s in s1}
    #     sol_dict_3 = {s[0] : s[1:] for s in s3}
    #     for start in sol_dict_3.keys():
    #         print(start, sol_dict_1.get(start, "No"), sol_dict_3.get(start, "No"))
    #     print("")
    #     for start in sol_dict_1.keys():
    #         print(start, sol_dict_1[start], sol_dict_3.get(start, "No"))
    #     print(self.return_best_route_Xto1(considered_stops, start0, max_cost_value=LARGE_INT))
    #     exit()
    #     for a, b, c in zip(bn, bn2, bn0):
    #         start1 = nw.return_node_position(a.node_index)
    #         start2 = nw.return_node_position(b.node_index)
    #         start0 = nw.return_node_position(c.node_index)
    #         s1 = nw.return_travel_costs_1toX(start1, all_node_pos)
    #         s1 = [x for x in s1 if x[1] < 99999999999.9]
    #         s2 = nw2.return_travel_costs_1toX(start2, all_node_pos2)
    #         s2 = [x for x in s2 if x[1] < 99999999999.9]
    #         s3 = self.return_travel_costs_1toX(start0, all_node_pos0)
    #         s3 = [x for x in s3 if x[1] < 99999999999.9]
    #         print("n {} || self: {}/{} | nw1: {}/{} | nw2: {}/{} || check {} {} {}".format(c.node_index, len(s3), len(self.nodes), len(s1), len(nw.nodes), len(s2), len(nw2.nodes), a.pos_x, b.pos_x, c.pos_x))
    #         sol_dict_1 = {s[0] : s[1:] for s in s1}
    #         sol_dict_3 = {s[0] : s[1:] for s in s3}
    #         for start in sol_dict_3.keys():
    #             if not sol_dict_1.get(start):
    #                 print(start, sol_dict_3[start])
    #     print(len(bn), len(bn2), len(bn0))
        
        # print("loaded!")
        # org = self.return_node_position(12961)
        # sol = self.return_travel_costs_Xto1([nw.return_node_position(n.node_index) for n in self.nodes.values()], org)
        # print(len(sol))
        # print(len(nw.nodes))
        # sol = nw.return_travel_costs_Xto1([nw.return_node_position(n.node_index) for n in nw.nodes], org)
        # sol = [s for s in sol if s[1] < 999999999.9]
        # print(len(sol))
        # s2 = nw.return_best_route_Xto1([nw.return_node_position(n.node_index) for n in nw.nodes], org)
        # s3 = self.return_best_route_Xto1([nw.return_node_position(n.node_index) for n in nw.nodes], org)
        # # print(s2)
        # # print(s3)
        # s2 = [x for x in s2 if len(x) > 2]
        # print(len(s2), len(s3))
        # exit()
        # for n1, n2 in zip(self.nodes.values(), nw.nodes):
        #     #print(n1.pos_x, n2.pos_x)
        #     try:
        #         b = nw.return_travel_costs_1to1(org, self.return_node_position(n1.node_index))
        #     except:
        #         b = (float("inf"), float("inf"), float("inf"))
        #     #print(b)
        #     try:
        #         a = self.return_travel_costs_1to1(org, self.return_node_position(n1.node_index))
        #     except:
        #         a = (float("inf"), float("inf"), float("inf"))
        #     if abs(a[0] - b[0]) > 0.0001:
        #         print(a, b)
        # exit()