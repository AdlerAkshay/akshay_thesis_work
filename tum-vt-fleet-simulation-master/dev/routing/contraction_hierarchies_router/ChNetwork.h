#pragma once
#include <vector>

#include "ChGraph.h"

struct Resultstruct {
    int target;
    double traveltime;
    double traveldistance;
};

class ChNetwork {
private:
    // Most of the logic is packed here.
    ChGraph chGraph;

    std::vector<int> computed_path;
public:
	/*
		Constructor. Creates the ChGraph for the given network path

		@param network_path: path to the network directory (where the base folder is located).
	*/
	ChNetwork(std::string network_path);

	/*
		Updates the travel times of the network edges
		Is not supported for the CH router.

		@param file_path: path to the file containing the new travel times
	*/
	void updateEdgeTravelTimes(std::string file_path);

	/*
		@return the number of nodes in the network
	*/
	unsigned int getNumberNodes();

	/*
		Computes the shortest path costs between the source node and all target nodes.

		@param start_node_index: the index of the start node
		@param number_targets: the number of target nodes
		@param targets: pointer to the array containing number_targets target node indices
		@param reached_targets: pointer to an array to be filled with the reached target node indices
		@param reached_target_tts: pointer to an array to be filled with the reached target node travel times
		@param reached_target_dis: pointer to an array to be filled with the reached target node distances
		@param time_range: not used
		@param max_targets: not used

		@return the number of reached targets and thus the size of reached_targets, reached_targets_tts and reached_targets_dis
	*/
	int computeTravelCosts1ToXpy(int start_node_index, int number_targets, int* targets, int* reached_targets, double* reached_target_tts, double* reached_target_dis, double time_range = -1, int max_targets = -1);

	/*
		Computes the shortest path costs between the source node and all target nodes in a backward manner.
		This is equivalent to 1ToX on the reversed graph. So the params and return value definitions will be the same here.

		@param start_node_index: the index of the start node
		@param number_targets: the number of target nodes
		@param targets: pointer to the array containing number_targets target node indices
		@param reached_targets: pointer to an array to be filled with the reached target node indices
		@param reached_target_tts: pointer to an array to be filled with the reached target node travel times
		@param reached_target_dis: pointer to an array to be filled with the reached target node distances
		@param time_range: not used
		@param max_targets: not used

		@return the number of reached targets and thus the size of reached_targets, reached_targets_tts and reached_targets_dis
	*/
	int computeTravelCostsXTo1py(int start_node_index, int number_targets, int* targets, int* reached_targets, double* reached_target_tts, double* reached_target_dis, double time_range = -1, int max_targets = -1);
	
	/*
		Computes the shortest path cost between the source and target nodes.

		@param start_node_index: index of the starting node
		@param target_node_index: index of the target node
		@param tt: pointer to a double where the shortest path travel time is to be saved
		@pararm dis: pointer to a double where the shortest path distance is to be saved
	*/
	void computeTravelCosts1To1py(int start_node_index, int end_node_index, double* tt, double* dis);

	/*
		Computes the actual shortest path between the source and target nodes. 
		The path will be saved in this->computed_graph

		@param start_node_index: index of the starting node
		@param target_node_index: index of the target node
		
		@return the number of nodes in the shortest path
	*/
	int computeRouteSize1to1(int start_node_index, int end_node_index);

	/*
		Saves the computed shortest path from the above method in the given array pointer.

		@param output_array: a pointer to an array where the computed shortest path is to be saved
	*/
	void writeRoute(int* output_array);
};