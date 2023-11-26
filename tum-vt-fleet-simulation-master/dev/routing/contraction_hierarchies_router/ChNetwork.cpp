#include <vector>
#include <cfloat>
#include <iostream>
#include <stdexcept>

#include "ChNetwork.h"

ChNetwork::ChNetwork(std::string filepath) : chGraph(ChGraph(filepath)) {}

void ChNetwork::updateEdgeTravelTimes(std::string file_path) {
    std::cout << "Updating travel times is not supported in the Contraction Hierarchy router.\n";
    throw;
}

unsigned int ChNetwork::getNumberNodes() {
    return this->chGraph.get_number_of_nodes();
}

int ChNetwork::computeTravelCosts1ToXpy (
    int start_node_index,
    int number_targets,
    int* targets,
    int* reached_targets,
    double* reached_target_tts,
    double* reached_target_dis,
    double time_range,
    int max_targets 
) {
    // Create a vector with the target node indices
    std::vector<int> targets_vector = std::vector<int>(targets, targets + number_targets);
    
    // 1-to-N query call to the chGraph
    std::vector<NetworkPath> target_costs = this->chGraph.shortest_path_1_to_N(start_node_index, targets_vector);

    // Fill the reached targets arrays where the cost is < Infinity
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

int ChNetwork::computeTravelCostsXTo1py (
    int start_node_index,
    int number_targets,
    int* targets,
    int* reached_targets,
    double* reached_target_tts,
    double* reached_target_dis,
    double time_range,
    int max_targets
) {
    // Create a vector with the source node indices
    std::vector<int> starts_vector = std::vector<int>(targets, targets + number_targets);

    // N-to-1 query call to the chGraph
    std::vector<NetworkPath> start_costs = this->chGraph.shortest_path_N_to_1(starts_vector, start_node_index);

    // Fill the reached targets arrays where the cost is < infinity
    int cur_index = 0;
    for (int i = 0; i < number_targets; i++) {
        if (start_costs[i].cost != DBL_MAX) {
            reached_targets[cur_index] = targets[i];
            reached_target_dis[cur_index] = start_costs[i].distance;
            reached_target_tts[cur_index] = start_costs[i].travel_time;

            cur_index ++;
        }
    }

    return cur_index; // number of reached targets
}

void ChNetwork::computeTravelCosts1To1py (int start_node_index, int end_node_index, double* tt, double* dis) {
    // Compute the shortest 1-to-1 path costs without the actual node list
    NetworkPath path = this->chGraph.shortest_path_1_to_1(start_node_index, end_node_index);

    // save tt and dis
    *tt = path.travel_time;
    *dis = path.distance;
}

int ChNetwork::computeRouteSize1to1(int start_node_index, int end_node_index) {
    // Compute the shortest 1-to-1 path costs with the actual node list
    NetworkPath path = this->chGraph.shortest_path_1_to_1(start_node_index, end_node_index, true);

    // Save the computed path nodes
    this->computed_path = std::move(path.nodes);

    // Return the number of nodes in the path
    return (int)this->computed_path.size();
}

void ChNetwork::writeRoute(int* output_array) {
    for (int i = 0; i < this->computed_path.size(); i++)
        output_array[i] = computed_path[i];
}

