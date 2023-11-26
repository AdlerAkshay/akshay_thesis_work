#pragma once
#include <vector>
#include <algorithm>
#include <unordered_map>
#include <math.h>

#include "utils.h"

struct pair_hash;

/*
    A structure that represents a network path.
*/
struct NetworkPath {
    // cost of the path. (Can be distance or travel time or a combination of both)
    double cost;
    // distance of the whole path
    double distance;
    // travel_time of the whole path
    double travel_time;
    // The list of nodes the path goes through. Can be empty if not needed.
    std::vector<int> nodes;

    // Creates an empty path with infinity as distance and travel time.
    NetworkPath() {
        cost = distance = travel_time = DBL_MAX;
    }

    // Creates a NetworkPath without the list of node stops.
    NetworkPath(double cost, double distance, double travel_time): cost(cost), distance(distance), travel_time(travel_time) {}
    // Creates a NetworkPath with the list of node stops.
    NetworkPath(double cost, double distance, double travel_time, std::vector<int>&& nodes): cost(cost), distance(distance), travel_time(travel_time), nodes(std::move(nodes)) {}
};

class ChGraph {
protected:
    /* ----------------------------- General information ----------------------------- */
    int number_of_nodes;  // The number of nodes in the network
    int number_of_edges;  // The number of edges in the network (shortcuts not included)
    std::string network_path;  // Path to the network (where the base/ folder is located)
    /* ------------------------------------------------------------------------------- */

    /* ------------------------------- Node information ------------------------------ */
    std::vector<double> node_x;  // i-th element is the x-coordinate of the i-th node
    std::vector<double> node_y;  // i-th element is the y-coordinate of the i-th node
    std::vector<bool> node_is_stop;  // i-th element is true iff the i-th node is a stop-only node

    std::vector<int> node_in_degree;  // i-th element is the number of incoming edges to the i-th node
    std::vector<int> node_out_degree; // i-th element is the number of outgoing edges to the i-th node
    /* ------------------------------------------------------------------------------- */

    /* ------------------------------ Edge information ------------------------------- */

    // The list of edges in the network including shortcuts and their reverse edges
    // Index 2*i contains an edge (can be a shortcut) and 2*i + 1 contains its reverse edge.
    // Reverse edge is the same edge in the reversed direction.
    std::vector<Edge> edges;  

    /*
        cost function for edges. Cost can be distance or travel time or a combination of both.
        At the moment, travel times are used as edge costs.

        @param edge: an edge or shortcut (can be a reversed edge)

        @return the cost of an edge
    */

    double get_edge_cost(Edge edge);

    /*
        cost function for edges. Cost can be distance or travel time or a combination of both.
        At the moment, travel times are used as edge costs.

        @param edge: the index of an edge or shortcut (can be a reversed edge)

        @return the cost of this->edges[edge_id]
    */
    double get_edge_cost(int edge_id);
    /* ------------------------------------------------------------------------------- */

    /* -------------------- Adjacency lists of the network graph --------------------- */

    // Adjacency list of the graph with shortcuts.
    // network_adjacency_with_shortcuts[i] is a list of edge indices j (all even) with this->edges[j].from == i
    // i.e. The neighbourhood of node i as a list of edge indices.
    std::vector<std::vector<int>> network_adjacency_with_shortcuts;

    // Adjacency list of the reversed graph with shortcuts.
    // network_adjacency_reversed_with_shortcuts[i] is a list of edge indices j (all odd) with this->edge[j].from == i
    // Notice here the edges are all reversed edges from the edges vector.
    std::vector<std::vector<int>> network_adjacency_reversed_with_shortcuts;
    /* ------------------------------------------------------------------------------- */

    /* ----------------------- Contraction hierarchy specific ------------------------ */
    bool ch_built = false;  // Flag variable. True iff the network is contracted.
    int shortcuts_count = 0; // The number of shortcuts added to the network
    std::vector<int> node_orders;  // i-th element is the order/rank of node i. Ranks are in [0 ... number_of_nodes - 1]
    
    /* The following functions and vectors are needed in build_contraction_hierarchies */
    std::vector<bool> is_contracted; // i-th element is true iff i-th node is contracted
    std::vector<double> priorities;  // i-th element is the priority of node i

    /*
        Compute the edge_differences heuristic for the given node
        edge_differences(v) := #number-of-shortcuts-to-add-when-v-is-contracted - #incoming-edges-to-v

        @param node: index of the node for which the edge differences is to be computed

        @return the edge differences of the given node
    */
    double compute_edge_difference(int node);

    /*
        Compute the deleted_neighbours heuristic for the given node
        deleted_neighbours(v) := #number-of-contracted-neighbours-of-v

        @param node: index of the node for which the deleted neighbours is to be computed

        @return the deleted_neighbours of the given node
    */
    double compute_deleted_neighbours(int node);

    /*
        Compute priority value of the given node.
        priority(v) := 1/2 * edge_differences(v) + 1/2 * deleted_neighbours(v)

        @param node: index of the node for which the priority is to be computed

        @return the priority of the given node
    */
    double compute_node_priority(int node);

    /*
        update the priority of the given node

        @param node: index of the node for which the priority is to be updated

        @return true iff the node priority has changes. false otherwise
    */
    bool update_priority(int node);

    /*
        Check whether there is a path between from and to 
        that does not use the node with index ignore_node 
        and has a cost < ref_distance with at most hop_limit stops

        @param from: the index of the starting node
        @param to: the index of the target node
        @param ignore_node: the index of a node that should be ignored in the search. Set to -1 if no ignore_node is needed
        @param ref_dist: an upper bound on the path cost
        @param hop_limit: an upper bound on the path length

        @return true iff such a path is found. false otherwise.
    */
    bool there_is_a_shorter_path(int from, int to, int ignore_node, double ref_dist, int hop_limit);

    /*
        Contract the given node, add shortcuts where needed and update the priorities of all neighbours

        @param node: index of the node to contract

        @return a list of pairs (prio, v) with prio being the new priority of the neighbour with the index v, if its priority has actually changed.
    */
    std::vector<std::pair<double, int>> contract_node_and_update_neighbours_priorities(int node);
    /* ------------------------------------------------------------------------------- */
    
    /* --------------------------- Shortest path related ----------------------------- */

    // Theses vectors are used to avoid a new memory allocation each time the dijkstra function is to be called.
    std::vector<double> costs1, costs2, times1, times2, dists1, dists2;
    std::vector<Edge*> parents1, parents2;
    std::vector<double> backward_costs;

    /*
        A special Dijkstra implementation that can be used for all variations for finding shortest paths (i.e the path with the lowest cost).
        Note: the cost is defined by the implementation of get_edge_cost() function.

        @param graph: the adjacency list to be used in the dijkstra search
        @param start_node: the node index to start the search from
        @param target_node: then node index of the target node to find the shortest path to. Can be set to -1 if we want to find SPs to all nodes
        @param costs: to be filled with the optimal path's costs from start_node to each node with index i (DBL_MAX if not explored)
        @param dists: to be filled with the optimal path's distances from start_node to each node with index i (DBL_MAX if not explored)
        @param times: to be filled with the optimal path's times from start_node to each node with index i (DBL_MAX if not explored)
        @param parents: to be filled with the optimal path's parents from start node to each node with index i. (-1 if no parent, i.e. not in the path)
        @param is_backward: true if the search is a backward search. false otherwise 
                            NOTE: if this is set to true, the graph passed as a parameter is the reversed graph
                            NOTE: if this is set to true, then forward_costs is expected to be given. Which would be the result of the forward pass done previously. (e.g. from 1-to-1 routing with CH)
        @param forward_costs: pointer to a vector with i-th element being the cost of reaching the i-th node from a previous call with some start node.
        @param go_upwards: true if the search is done upwards. I.e. from node u only explore nodes v with this->node_orders[v] > this->nodes_orders[u]. False otherwise
        @param ignore_node: -1 if no node is to be ignored. Otherwise, the node index of a node to not be used in the search.
        @param ignore_contracted: true if nodes that are contracted (i.e. this->is_contracted[node] == true) should be ignored/not used in the search. False otherwise
        @param hop_limit: -1 if no limit is used. Otherwise, an upperbound on the depth of the path search
        @param cost_limit: An upperbound on the path cost (i.e. do not keep exploring if cost already exceeds cost_limit). Default is infinity.

        @return pair (cost, v) with v being the target for a normal dijkstra call and cost being the optimal path cost to v.
                When is_backward == true && go_upwards == true && forward_costs != null, this means that the call is the backward upward search of the 1-to-1 routing with CH:
                in this case, v is the ideal meeting point between the forward and backward searches and cost is the optimal path cost.
    */
    std::pair<double, int> dijkstra (                   
        std::vector<std::vector<int>>& graph,
        int start_node,
        int target_node,
        std::vector<double>& costs,
        std::vector<double>& dists,
        std::vector<double>& times,
        std::vector<Edge*>& parents,
        bool is_backward = false,
        std::vector<double>* forward_costs = nullptr,
        bool go_upwards = false,
        int ignore_node = -1,
        bool ignore_contracted = false,
        int hop_limit = -1,
        double cost_limit = DBL_MAX
    );

    /*
        Function used to solve the Many-to-Many query. Perform an upward seach on the graph from each node in the nodes list. 
        Keep track of costs to reach every node v in the graph from every node u in nodes in buckets[v]
        Better to understand by reading the implementation of shortest_path_N_to_N()

        @param graph: the adjacency list to use (either network_adjacency_with_shortcuts or network_adjacency_reversed_with_shortcuts)
        @param nodes: list of nodes to perform upward passes from
        @param buckets: list of size number_of_nodes. i-th bucket maps every node from nodes to the optimal network path cost.
        @param cost_limit: limit on the radius of the search cost. 
    */
    void ch_NtoN_pass (
        std::vector<std::vector<int>>& graph,
        std::vector<int>& nodes,
        std::vector<std::unordered_map<int, NetworkPath>>& buckets,
        double cost_limit = DBL_MAX
    );

    /*
        combine two calls of ch_NtoN_pass() to find all pairs optimal paths between start_nodes and target_nodes
        Better to understand by reading the implementation of shortest_path_N_to_N()

        @param start_nodes: list of start nodes
        @param target_nodes: list of target_nodes
        @param first_pass_buckets: buckets to use for ch_NtoN_pass() call with the start nodes and forward graph
        @param second_pass_buckets: buckets to use for ch_NtoN_pass() call with the target nodes and reversed graph
        @param cost_limit: limit on the radius of the search cost

        @return map each (u, v) to the optimal path from u to v with u in start_nodes and v in target_nodes
    */
    std::unordered_map<std::pair<int, int>, NetworkPath, pair_hash> combine_NtoN_passes (
        std::vector<int>& start_nodes,
        std::vector<int>& target_nodes,
        std::vector<std::unordered_map<int, NetworkPath>>& first_pass_buckets,
        std::vector<std::unordered_map<int, NetworkPath>>& second_pass_buckets,
        double cost_limit = DBL_MAX
    );

    /*
        Utility function used to unpack an edge to a list of node stops.
        If shortcut: recursively unpack edgeA and edgeB of the shortcut, then merge.

        @param edge: pointer to the edge to unpack (can be a shortcut)

        @return a list of node stops
    */
    std::vector<int> unpack_edge(Edge* edge);
    /* ------------------------------------------------------------------------------- */

public:
    /*
        Constructor for the contraction hierarchies graph.

        @param network_path: path to the network (where the base folder is located)
        @param load_contracted_if_present: if set to true, then the contracted network is loaded from the disk. False if no loading is wished.
    */
    ChGraph(std::string network_path, bool load_contracted_if_present = true);

    /*
        Add edge to the network.
        The edge is added to this->edges in an even index 2*i, its reverse edge is computed and saved at 2*i+1.
        The adjacency lists are updated accordingly.

        @param edge: the edge to be added to the network

        @return edge index in the edges vector
    */
    int add_edge(Edge edge); // returns edge-id (used in the adjacency lists)

    // Getter function to return the number of nodes in the graph
    int get_number_of_nodes();
    // Getter function to return the number of edges in the graph
    int get_number_of_edges();

    /*
        build the contraction hierarchies graph.
        Shortcuts are added to the graph accordingly.
        Node orders are determined when the CH is built and the ch_built flag is set.
    */
    void build_contraction_hierarchy();
    /*
        Save the contraction Hierarchies graph under <network_path>/ch_graph
    */
    void save_contraction_hierarchy();

    /*
        Compute the shortest path between node with index from and node with index to.
        1. Forward upward seach from start node
        2. Backward upward seach from target node
        -> Shortest path between all search intersections.
    
        @param from: the start node index
        @param to: the target node index
        @param with_path: true if the actual path (node stops) are to be included in the NetworkPath result. False otherwise
        @param cost_limit: upperbound on the path cost

        @return NetworkPath object with optimal path cost, distance and travel time and node stops if with_path==true
    */
    NetworkPath shortest_path_1_to_1(int from, int to, bool with_path = false, double cost_limit = DBL_MAX);

    /*
        This function is used for comparison/sanity checks only. This is uses normal dijkstra for solving the one-to-one query.
    */
    NetworkPath shortest_path_1_to_1_dijkstra(int from, int to);

    /*
        Compute the shortest paths from the start node to all target nodes.

        @param from: index of the start node
        @param target_nodes: list of indices target nodes
        @param cost_limit: upperbound on the path cost

        @return list of optimal network paths (without the actual node stops). i-th element is the optimal path from start node to target_nodes[i]
    */
    std::vector<NetworkPath> shortest_path_1_to_N (int from, std::vector<int>& target_nodes, double cost_limit = DBL_MAX);

    /*
        Compute the shortest paths from all start nodes to the target node

        @param start_nodes: list of indices of start nodes
        @param target_nodes: target node
        @param cost_limit: upperbound on the path cost

        @return list of optimal network paths (without the actual node stops). i-th element is the optimal path from start_nodes[i] to target_node
    */
    std::vector<NetworkPath> shortest_path_N_to_1 (std::vector<int>& start_nodes, int target_node, double cost_limit = DBL_MAX);

    /*
        Compute the shortest paths from each start node to each target node

        @param start_nodes: list of indices of start nodes
        @param target_nodes: list of indices of target nodes
        @param cost_limit: upperbound on the path costs

        @return a map from each pair of (u,v) to the optimal network path (without the actual node stops) for u in start nodes and v in target nodes
    */
    std::unordered_map<std::pair<int, int>, NetworkPath, pair_hash> shortest_path_N_to_N (
        std::vector<int>& start_nodes,
        std::vector<int>& target_nodes,
        double cost_limit = DBL_MAX
    );

    /*
        Compute the path nodes

        @param from: index of start node
        @param to: index of target node
        @param parents: array of node parents (as edges) in the optimal path from start to target nodes

        @return list of node stops of the optimal path from start to target nodes
    */
    std::vector<int> unpack_path(int from, int to, std::vector<Edge*>& parents);

};