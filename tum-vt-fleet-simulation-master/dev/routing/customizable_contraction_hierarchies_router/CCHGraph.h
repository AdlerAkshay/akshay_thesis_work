#pragma once
#include <vector>
#include <cassert>
#include <cfloat>
#include <algorithm>
#include <math.h>

#include "../contraction_hierarchies_router/ChGraph.h"

class CCHGraph: private ChGraph {
private:
    // Flag-variables
    bool ndComputed = false;
    bool phase1Complete = false;
    bool phase2Complete = false;

    /*
        Sorts the given vector of edge pairs according to the edge endpoints levels. Lower endpoints first.
        The given vector of pairs contains a pair (e1, e1_rev) where (*e1_rev) is the reversed edge of (*e1)
        So for sorting it suffices to look at the first element of the pair.
        Before sorting we determine the node_order of each node on the edge.

        @param arcs: vector of pairs (pointer to an edge and a pointer to the reversed edge)

        @return sorted arcs by edge level (node order of edge endpoints). Lower first.
    */
    std::vector<std::pair<Edge*, Edge*>> sort_arcs(std::vector<std::pair<Edge*, Edge*>>& arcs);
public:
    /*
        Constructor to create a CchGraph of the given network path

        @param graph_path: path to the network (where the base folder is located)
        @param load_phase_one_graph: true if the phase one graph should be loaded if it exists, false otherwise
        @param load_phase_two_graph: true if the phase two graph should be loaded if it exists, false otherwise
    */
    CCHGraph(std::string graph_path, bool load_phase_one_graph = true, bool load_phase_two_graph = true);

    using ChGraph::add_edge;
    using ChGraph::get_number_of_nodes;
    using ChGraph::get_number_of_edges;

    /*
        Clears all graph edges and adjacency lists
    */
    void clear_edges_and_adjacency_lists();
    
    /*
        Load new edge travel times from the given file path

        @param file_path: path to the file from which the new travel times should be loaded
    */
    void load_new_edge_travel_times(std::string file_path);

    // PHASE-1: Metric-independent order (Nested Dissection)
    void load_nested_dissection_order();  // Loads the nested dissection order from <network_path>/base/cch_nd_order
    void contract_nodes_in_order();       // Contracts the network nodes in the order defined by this->node_orders (should be the nested dissection order)
    void save_phase_one_graph();          // Save the graph edges after running the two methods above under <network_path>/cch_graph/phase_one_edges.csv
    void load_phase_one_graph();          // Load the graph edges from <network_path>/cch_graph/phase_one_edges.csv

    // PHASE-2: Triangle enumeration and edge weights computation.
    void run_triangle_enumeration_and_update_weights();   // Runs the triangle enumeration step and updates shortcut weights on the post-phase-one graph
    void save_phase_two_graph();                          // Save the graph edges after running the method above under <network_path>/cch_graph/phase_two_edges.csv
    void load_phase_two_graph();                          // Load the graph edges from <network_path>/cch_graph/phase_two_edges.csv

    // PHASE-3: Querying
    // We use the implementation of ChGraph. Documentation is there.
    using ChGraph::shortest_path_1_to_1;
    using ChGraph::shortest_path_1_to_1_dijkstra;
    using ChGraph::shortest_path_1_to_N;
    using ChGraph::shortest_path_N_to_1;
    using ChGraph::shortest_path_N_to_N;
    using ChGraph::unpack_path;
};