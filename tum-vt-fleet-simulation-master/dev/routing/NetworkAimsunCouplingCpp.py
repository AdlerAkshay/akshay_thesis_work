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
from src.routing.NetworkBasicWithStoreCpp import NetworkBasicWithStoreCpp
from src.routing.cpp_router.PyNetwork import PyNetwork

# -------------------------------------------------------------------------------------------------------------------- #
# global variables
# ----------------
from src.misc.globals import *
LOG = logging.getLogger(__name__)

class NetworkAimsunCouplingCpp(NetworkBasicWithStoreCpp):
    def __init__(self, network_name_dir, network_dynamics_file_name=None, scenario_time=None):
        """ this network is used to couple with aimsun.
        depending on performance the parent network might be changed since routing functions are inherited completely
        edge travel times are changed dynamically within the aimsun simulation, the updated travel time files are therefore not stored
        in the network-folder but in the scenario results folder. This class reads the file from there
        :param network_dynamics_file_name: file-name of the network dynamics file
        :type network_dynamics_file_name: str
        """
        self.aimsun_edge_travel_times_base_path = None
        self.last_travel_time_update = -1
        super().__init__(network_name_dir, scenario_time=scenario_time)

        self.must_stop_nodes = [n.node_index for n in self.nodes if n.must_stop()]

    def add_init_data(self, aimsun_edge_travel_times_base_path):
        self.aimsun_edge_travel_times_base_path = aimsun_edge_travel_times_base_path

    def update_network(self, simulation_time, update_state = False):
        """This method can be called during simulations to update travel times (dynamic networks).

        update_network is called every simulation step, but only when a new travel_time file is created the update_state flag is set to True

        :param simulation_time: time of simulation
        :type simulation_time: float
        :param update_state: False by default: Tells if a new tt file should be loaded (raises error if this file is not created)
        :rtype update_state: bool
        :return: new_tt_flag True, if new travel times found; False if not
        :rtype: bool
        """
        LOG.info(f"update network {simulation_time}")
        self.sim_time = simulation_time
        if update_state and self.last_travel_time_update != simulation_time:
            self.load_tt_file(simulation_time)
            self._reset_internal_attributes_after_travel_time_update()
            self.last_travel_time_update = simulation_time
            return True
        return False

    def load_tt_file(self, scenario_time):
        """
        loads new travel time files for scenario_time
        """
        if self.aimsun_edge_travel_times_base_path is None:
            return
        tt_file = os.path.join(self.aimsun_edge_travel_times_base_path, str(scenario_time), "edges_td_att.csv")
        tmp_df = pd.read_csv(tt_file, index_col=[0,1])
        for edge_index_tuple, new_tt in tmp_df["edge_tt"].iteritems():
            o_node = self.nodes[edge_index_tuple[0]]
            d_node = self.nodes[edge_index_tuple[1]]
            edge_obj = o_node.edges_to[d_node]
            edge_obj.set_tt(new_tt)
        self.cpp_router.updateEdgeTravelTimes(tt_file.encode())

    def return_travel_costs_1to1(self, origin_position, destination_position, customized_section_cost_function = None):
        """
        This method will return the travel costs of the fastest route between two nodes.
        :param origin_position: (current_edge_origin_node_index, current_edge_destination_node_index, relative_position)
        :param destination_position: (destination_edge_origin_node_index, destination_edge_destination_node_index, relative_position)
        :param customized_section_cost_function: function to compute the travel cost of an section: args: (travel_time, travel_distance, current_dijkstra_node) -> cost_value
                if None: travel_time is considered as the cost_function of a section
        :return: (cost_function_value, travel time, travel_distance) between the two nodes
        """
        if customized_section_cost_function is not None:
            return super().return_travel_costs_1to1(origin_position, destination_position, customized_section_cost_function=customized_section_cost_function)
        trivial_test = self.test_and_get_trivial_route_tt_and_dis(origin_position, destination_position)
        if trivial_test is not None:
            return trivial_test[1]
        origin_node = origin_position[0]
        origin_overhead = (0.0, 0.0, 0.0)
        if origin_position[1] is not None:
            origin_node = origin_position[1]
            origin_overhead = self.get_section_overhead(origin_position, from_start=False)
        destination_node = destination_position[0]
        destination_overhead = (0.0, 0.0, 0.0)
        if destination_position[1] is not None:
            destination_overhead = self.get_section_overhead(destination_position, from_start=True)
        s = None
        if customized_section_cost_function is None:
            s = self.travel_time_infos.get( (origin_node, destination_node) , None)
        if s is None:
            if self.nodes[origin_node].must_stop() and self.nodes[destination_node].must_stop():
                s = self.cpp_router.computeTravelCosts1toX(origin_node, self.must_stop_nodes)
                for dest_node, tt, dis in s:
                    if tt < -0.0001:
                        continue
                    self.travel_time_infos[(origin_node, dest_node)] = (tt, tt, dis)
                s = self.travel_time_infos[(origin_node, destination_node)]
            else:
                r = self.cpp_router.computeTravelCosts1To1(origin_node, destination_node)
                if r[0] < -0.0001:
                    r = (float("inf"), float("inf"))
                s = (r[0], r[0], r[1])
                self.travel_time_infos[(origin_node, destination_node)] = s
        return (s[0] + origin_overhead[0] + destination_overhead[0], s[1] + origin_overhead[1] + destination_overhead[1], s[2] + origin_overhead[2] + destination_overhead[2])

