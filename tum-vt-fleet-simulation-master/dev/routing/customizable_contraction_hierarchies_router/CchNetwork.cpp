#include <vector>
#include <cfloat>
#include <iostream>
#include <stdexcept>

#include "CchNetwork.h"

CchNetwork::CchNetwork(std::string filepath) : cchGraph(CCHGraph(filepath, true, true)) {}

void CchNetwork::updateEdgeTravelTimes(std::string file_path) {
    cchGraph.load_new_edge_travel_times(file_path);
}

unsigned int CchNetwork::getNumberNodes() {
    return this->cchGraph.get_number_of_nodes();
}

int CchNetwork::computeTravelCosts1ToXpy (
    int start_node_index,
    int number_targets,
    int* targets,
    int* reached_targets,
    double* reached_target_tts,
    double* reached_target_dis,
    double time_range,
    int max_targets 
) {
    std::vector<int> targets_vector = std::vector<int>(targets, targets + number_targets);

    std::vector<NetworkPath> target_costs = this->cchGraph.shortest_path_1_to_N(start_node_index, targets_vector);

    // Target is reached if cost is not infinity
    int cur_index = 0;
    for (int i = 0; i < number_targets; i++) {
        if (target_costs[i].cost != DBL_MAX) {
            reached_targets[cur_index] = targets[i];
            reached_target_dis[cur_index] = target_costs[i].distance;
            reached_target_tts[cur_index] = target_costs[i].travel_time;

            cur_index ++;
        }
    }

    return cur_index; // number of reached targets
}

int CchNetwork::computeTravelCostsXTo1py (
    int start_node_index,
    int number_targets,
    int* targets,
    int* reached_targets,
    double* reached_target_tts,
    double* reached_target_dis,
    double time_range,
    int max_targets
) {
    std::vector<int> starts_vector = std::vector<int>(targets, targets + number_targets);

    std::vector<NetworkPath> start_costs = this->cchGraph.shortest_path_N_to_1(starts_vector, start_node_index);

    // Start node is reached if cost is not infinity
    int cur_index = 0;
    for (int i = 0; i < number_targets; i++) {
        if (start_costs[i].cost != DBL_MAX) {
            reached_targets[cur_index] = targets[i];
            reached_target_dis[cur_index] = start_costs[i].distance;
            reached_target_tts[cur_index] = start_costs[i].travel_time;

            cur_index ++;
        }
    }

    return cur_index;
}

void CchNetwork::computeTravelCosts1To1py (int start_node_index, int end_node_index, double* tt, double* dis) {
    NetworkPath path = this->cchGraph.shortest_path_1_to_1(start_node_index, end_node_index);

    *tt = path.travel_time;
    *dis = path.distance;
}

int CchNetwork::computeRouteSize1to1(int start_node_index, int end_node_index) {
    NetworkPath path = this->cchGraph.shortest_path_1_to_1(start_node_index, end_node_index, true);

    this->computed_path = std::move(path.nodes);

    return (int)this->computed_path.size();
}

void CchNetwork::writeRoute(int* output_array) {
    for (int i = 0; i < this->computed_path.size(); i++)
        output_array[i] = computed_path[i];
}

