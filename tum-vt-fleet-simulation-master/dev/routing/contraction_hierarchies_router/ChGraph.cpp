#include <vector>
#include <algorithm>
#include <unordered_map>
#include <math.h>
#include <iostream>
#include <queue>
#include <cassert>
#include <cfloat>

#include "ChGraph.h"

// When node w is contracted and u -> w -> v:
// Add shortcut from u to v if there is no path of length <= HOP_LIMIT_FOR_SHORTCUTS 
// from u to v with cost lower than the cost of u -> w -> v
const int HOP_LIMIT_FOR_SHORTCUTS = 10;

ChGraph::ChGraph(std::string filepath, bool load_contracted_if_possible) {
    this->network_path = filepath;
    // read nodes
    this->number_of_nodes = read_nodes(filepath + "/base/nodes.csv", this->node_x, this->node_y, this->node_is_stop);
    std::cout << "Number of nodes is " << this->number_of_nodes << std::endl;
    // initialization
    this->shortcuts_count = 0;
    this->edges.clear();

    this->network_adjacency_with_shortcuts.resize(this->number_of_nodes);
    this->network_adjacency_reversed_with_shortcuts.resize(this->number_of_nodes);
    this->node_in_degree = std::vector<int>(this->number_of_nodes, 0);
    this->node_out_degree = std::vector<int>(this->number_of_nodes, 0);

    this->costs1 = std::vector<double>(this->number_of_nodes, DBL_MAX);
    this->costs2 = std::vector<double>(this->number_of_nodes, DBL_MAX);
    this->dists1 = std::vector<double>(this->number_of_nodes, DBL_MAX);
    this->dists2 = std::vector<double>(this->number_of_nodes, DBL_MAX);
    this->times1 = std::vector<double>(this->number_of_nodes, DBL_MAX);
    this->times2 = std::vector<double>(this->number_of_nodes, DBL_MAX);

    this->parents1 = std::vector<Edge*>(this->number_of_nodes, nullptr);
    this->parents2 = std::vector<Edge*>(this->number_of_nodes, nullptr);

    // read edges
    if (load_contracted_if_possible && contracted_graph_is_present(filepath)) {
        std::cout << "Contracted graph already exists. Will load from disk\n";
        this->ch_built = true;
        std::cout << "Loading node order ... ";
        read_node_order(filepath + "/ch_graph/node_order.csv", this->node_orders);
        std::cout << "Done.\nLoading contracted graph ... ";
        read_contracted_graph(filepath + "/ch_graph/chgraph.csv", *this);
        std::cout << "Done.\n\n";
    } else {
        std::cout << "Contracted graph does not exist. Nothing to load from disk.\n";
        this->number_of_edges = read_edges(filepath + "/base/edges.csv", *this);
    }
}

int ChGraph::add_edge(Edge edge) {
    // The index at which the edge will be added and thus the return value of the function
    int ret = this->edges.size();

    // Add the edge at the end of the edges vector. This will by implementation an even index
    this->edges.push_back(edge);
    // Add the reversed edge at the next index.
    this->edges.push_back(reverse_edge(&edge));

    int from = edge.from;
    int to = edge.to;

    // Update the adjacency lists accordingly.
    this->network_adjacency_with_shortcuts[from].push_back(ret);
    this->network_adjacency_reversed_with_shortcuts[to].push_back(ret + 1);

    // Update the node degrees accordingly.
    this->node_in_degree[to]++;
    this->node_out_degree[from]++;

    return ret;
}

double ChGraph::get_edge_cost(Edge edge) {
    return edge.t;  // At the moment the edge cost is just the travel time
}

double ChGraph::get_edge_cost(int edge_id) {
    return get_edge_cost(this->edges[edge_id]);
}

double ChGraph::compute_edge_difference(int node) {
    int n_shortcuts_to_add = 0;
    int n_incident_edges = this->node_in_degree[node];

    // For every u with u->node
    for (auto &e1_id: this->network_adjacency_reversed_with_shortcuts[node]) {
        Edge *e1 = &(this->edges[e1_id]);
        int u = e1->to;

        // if u is contracted, then skip
        if (is_contracted[u])
            continue;

        // For every v with node->v
        for (auto &e2_id: this->network_adjacency_with_shortcuts[node]) {
            Edge *e2 = &(this->edges[e2_id]);
            int v = e2->to;

            // if v is contracted then skip
            if (is_contracted[v])
                continue;

            // Check if contracting node would add a shortcut from u to v or not. If so increase the number of shortcuts to add.
            if (!this->there_is_a_shorter_path(u, v, node, get_edge_cost(e1_id) + get_edge_cost(e2_id), HOP_LIMIT_FOR_SHORTCUTS))
                n_shortcuts_to_add++;
        }
    }

    // #shortcuts-to-add-if-node-is-contracted - #number-of-incoming-edges-to-node
    return n_shortcuts_to_add - n_incident_edges;
}

double ChGraph::compute_deleted_neighbours(int node) {
    int n_contracted_neighbours = 0;

    // for every u with u -> node
    for (auto &edge_id: this->network_adjacency_with_shortcuts[node])
        n_contracted_neighbours += is_contracted[this->edges[edge_id].to];

    // for every v with node -> v
    for (auto &edge_id: this->network_adjacency_reversed_with_shortcuts[node])
        n_contracted_neighbours += is_contracted[this->edges[edge_id].to];

    // number of contracted neighbours
    return (double) n_contracted_neighbours;
}

double ChGraph::compute_node_priority(int node) {
    double ed = this->compute_edge_difference(node);
    double nd = this->compute_deleted_neighbours(node);

    // a 50-50 combination of edge differences and deleted neighbours heuristics.
    double p = 0.5;
    return p * ed + (1 - p) * nd;
}

bool ChGraph::update_priority(int node) {
    if (!this->is_contracted[node]) {
        // re-compute the node priority
        double node_priority = this->compute_node_priority(node);

        // if the node priority decreases
        if (node_priority < this->priorities[node]) {
            this->priorities[node] = node_priority;
            return true;
        }
    }
    return false;
}

bool ChGraph::there_is_a_shorter_path(int from, int to, int ignore_node, double ref_cost, int hop_limit) {
    std::vector<double> costs = std::vector<double>(this->number_of_nodes);
    std::vector<double> dists = std::vector<double>(this->number_of_nodes);
    std::vector<double> times = std::vector<double>(this->number_of_nodes);
    std::vector < Edge * > parents = std::vector<Edge *>(this->number_of_nodes);

    // Determine the shortest path from start to target node with radius at most hop_limit.
    double cost = this->dijkstra(this->network_adjacency_with_shortcuts, from, to, costs, dists, times, parents, false, nullptr, false,
                              ignore_node, true, hop_limit).first;
    //std:: cout << "Dijkstra returned " << d << " compared to ref_dist " << ref_dist << "\n"; 

    return cost < ref_cost;
}

std::vector <std::pair<double, int>> ChGraph::contract_node_and_update_neighbours_priorities(int node) {
    // A vector storing the neighbours with their updated priority.
    std::vector <std::pair<double, int>> to_update;

    // We do not contract nodes that are stop only.
    // Otherwise, we can use the shortcut in some path, even though the path would go through this node when it shouldn't.
    if(!(node_is_stop[node])) {
        // Adding shortcuts when needed
        for (auto &e1_id: this->network_adjacency_reversed_with_shortcuts[node]) {
            // u -> node
            Edge *e1 = &(this->edges[e1_id]);
            int u = e1->to;
            if (this->is_contracted[u])
                continue;

            for (auto &e2_id: this->network_adjacency_with_shortcuts[node]) {
                // node -> v
                Edge *e2 = &(this->edges[e2_id]);
                int v = e2->to;
                if (this->is_contracted[v] || u == v)
                    continue;

                // Now we need to figure out whether we should add a shortcut u -> v.
                if (!there_is_a_shorter_path(u, v, node, get_edge_cost(e1_id) + get_edge_cost(e2_id), HOP_LIMIT_FOR_SHORTCUTS)) {
                    // We need to add the shortcut to preserve the shortest paths.
                    int e1_rev_id = e1_id % 2 ? e1_id - 1 : e1_id + 1;
                    Edge *e1_rev = &(this->edges[e1_rev_id]);

                    this->add_edge(create_shortcut_from_edges(e1_rev, e2, e1_rev_id, e2_id));
                }
            }
        }
    }
    

    // updates priorities of neighbours.
    for (auto &edge_id: this->network_adjacency_with_shortcuts[node]) {
        Edge *edge = &(this->edges[edge_id]);
        int neighbour = edge->to;
        if (this->update_priority(neighbour))
            to_update.push_back(std::make_pair(this->priorities[neighbour], neighbour));
    }

    for (auto &edge_id: this->network_adjacency_reversed_with_shortcuts[node]) {
        Edge *edge = &(this->edges[edge_id]);
        int neighbour = edge->to;
        if (this->update_priority(neighbour))
            to_update.push_back(std::make_pair(this->priorities[neighbour], neighbour));
    }

    return to_update;
}

void ChGraph::build_contraction_hierarchy() {
    // store (priority, node_index) so that elements are automatically sorted by priority.
    std::priority_queue < std::pair < double, int >, std::vector < std::pair < double, int >>, std::greater <
                                                                                               std::pair <
                                                                                               double, int>>> pq;

    // Keep track of what nodes are contracted.
    this->is_contracted = std::vector<bool>(this->number_of_nodes, false);
    this->priorities = std::vector<double>(this->number_of_nodes);
    this->node_orders = std::vector<int>(this->number_of_nodes);

    // To set the node order.
    int rank = 0;

    // Add all nodes with their priority to the priority queue.
    for (int i = 0; i < this->number_of_nodes; i++) {
        double prio = this->compute_node_priority(i);
        priorities[i] = prio;
        pq.push(std::make_pair(prio, i));
    }

    while (!pq.empty()) {
        if (rank % 2000 == 0) {
            std::cout << "Contracted " << rank << " out of " << this->number_of_nodes << " nodes.\n";
        }

        // The node not yet contracted with smallest priority.
        std::pair<double, int> cur_pair = pq.top();
        int cur_node = cur_pair.second;
        pq.pop();

        // If already contracted continue
        if (is_contracted[cur_node])
            continue;

        // Lazy-update: Recompute priority and check if still smallest.
        double cur_node_prio = this->compute_node_priority(cur_node);
        priorities[cur_node] = cur_node_prio;

        if (!pq.empty() && cur_node_prio > pq.top().first) {
            // re-insert to queue with new priority and do not contract.
            pq.push(std::make_pair(cur_node_prio, cur_node));
            continue;
        }

        // Contract node and update neighbours prios.
        std::vector <std::pair<double, int>> to_update = this->contract_node_and_update_neighbours_priorities(cur_node);

        for (auto &p: to_update) {
            // sadly pq has no decrease_key we insert again.
            pq.push(p);
        }

        is_contracted[cur_node] = true;

        // Set the node order
        this->node_orders[cur_node] = rank++;
    }

    // Assert that all nodes were contracted.
    assert(rank == this->number_of_nodes);
    this->ch_built = true;
}

void ChGraph::save_contraction_hierarchy() {
    // Save the node order
    save_node_order(this->network_path + "/ch_graph", "node_order.csv", this->node_orders);
    // Save the contracted graph edges.
    save_contracted_graph(this->network_path + "/ch_graph", "chgraph.csv", this->edges);
}

std::pair<double, int> ChGraph::dijkstra(
        std::vector <std::vector<int>> &graph, // with shortcuts
        int start_node,
        int target_node,
        std::vector<double> &costs,
        std::vector<double> &dists,
        std::vector<double> &times,
        std::vector<Edge *> &parents,
        bool is_backward,
        std::vector<double> *forward_costs,
        bool go_upwards,
        int ignore_node,
        bool ignore_contracted,
        int hop_limit,
        double cost_limit
) {
    /* ------- Assertions to make sure this is called correctly. ------ */
    assert((int) graph.size() == this->number_of_nodes);
    assert((int) dists.size() == this->number_of_nodes);
    assert((int) parents.size() == this->number_of_nodes);
    if (is_backward)
        assert((int) forward_costs->size() == this->number_of_nodes);
    if (ignore_contracted)
        assert(!this->ch_built);
    if (go_upwards)
        assert(this->ch_built);
    /* ---------------------------------------------------------------- */

    // Initialize costs with Infinity and parents with null
    std::fill(costs.begin(), costs.end(), DBL_MAX);
    std::fill(times.begin(), times.end(), DBL_MAX);
    std::fill(dists.begin(), dists.end(), DBL_MAX);
    std::fill(parents.begin(), parents.end(), nullptr);

    // A vector to keep track of nodes that are settled in the Dijkstra.
    std::vector<bool> visited = std::vector<bool>(this->number_of_nodes, false);

    // A min-heap storing (cost, node, depth)
    std::priority_queue < std::tuple < double, int, int >, std::vector < std::tuple < double, int, int >>,
            std::greater < std::tuple < double, int, int>>> pq;

    // initialize start
    costs[start_node] = 0;
    times[start_node] = 0;
    dists[start_node] = 0;
    pq.push(std::make_tuple(0, start_node, 0));

    // This is used for the backward cost of the 1-to-1 query.
    double backward_best_cost = DBL_MAX;
    int meeting_node = -1;

    while (!pq.empty()) {
        int cur_node = std::get<1>(pq.top());

        double cur_cost = costs[cur_node];
        double cur_dist = dists[cur_node];
        double cur_time = times[cur_node];

        int cur_depth = std::get<2>(pq.top());

        pq.pop();

        // We do not explore further if the node is stop_only or already settled.
        if (visited[cur_node] || (node_is_stop[cur_node] && cur_node != start_node))
            continue;

        // End search if target node is reacheed
        if (cur_node == target_node) // never happens if target_node is set to -1
            return std::make_pair(cur_cost, target_node);

        // Update best cost path when the call is for the backward upward seach of 1-to-1 query
        if (is_backward && (*forward_costs)[cur_node] < DBL_MAX) {
            if (cur_cost + (*forward_costs)[cur_node] < backward_best_cost) {
                backward_best_cost = cur_cost + (*forward_costs)[cur_node];
                meeting_node = cur_node;
            }
        }

        // Don't explore further if max hop_limit is reached.
        if (hop_limit != -1 && cur_depth >= hop_limit)
            continue;

        // Settle current node.
        visited[cur_node] = true;

        // Iterate over neighbours.
        for (auto &edge_id: graph[cur_node]) {
            Edge *edge = &(this->edges[edge_id]);
            int next_node = edge->to;

            double edge_cost = this->get_edge_cost(edge_id);
            double edge_d = edge->d;
            double edge_t = edge->t;

            // Ignore the neighbour if already settled, should be ignored, cost is already lower, has lower rank in an upward search or is stop-only non target node.
            if (visited[next_node] ||
                next_node == ignore_node ||
                costs[next_node] <= cur_cost + edge_cost ||
                (ignore_contracted && this->is_contracted[next_node]) ||
                (go_upwards && this->node_orders[next_node] < this->node_orders[cur_node]) ||
                (node_is_stop[next_node] && next_node != start_node && next_node != target_node)
            )
                continue;

            // If neighbour node cost is lower than the previous cost, update and add to the queue.
            if (cost_limit == -1 || cur_cost + edge_cost < cost_limit) {
                dists[next_node] = cur_dist + edge_d;
                times[next_node] = cur_time + edge_t;
                costs[next_node] = cur_cost + edge_cost;
                
                // parent of neighbour is current node. (Here we specify what edge is used)
                parents[next_node] = edge;

                // Add neighbour with new cost and depth to the queue.
                pq.push(std::make_tuple(costs[next_node], next_node, cur_depth + 1));
            }
        }
    }
    
    // What to return, depends on the type of the call. 
    double ret_cost = (is_backward ? backward_best_cost : (target_node != -1 ? costs[target_node] : DBL_MAX));
    int ret_target = (is_backward ? meeting_node : target_node);
    return std::make_pair(ret_cost, ret_target);
}

// This method performs a forward upward search in the contracted graph from all source nodes.
void ChGraph::ch_NtoN_pass (
    std::vector<std::vector<int>>& graph,
    std::vector<int>& nodes,
    std::vector<std::unordered_map<int, NetworkPath>>& buckets,
    double cost_limit
) {
    std::vector<double>& costs = this->costs1;
    std::vector<double>& dists = this->dists1;
    std::vector<double>& times = this->times1;
    std::vector<Edge*>& parents = this->parents1;

    // For each node in the node list perform a dijkstra upward search search on the given graph (can be called with a reversed graph).
    for(auto node: nodes) {
        this->dijkstra(graph, node, -1, costs, dists, times, parents, false, nullptr, true, -1, false, -1, cost_limit);
        // Keep track of costs of each node from each node.
        for (int i = 0; i < this->number_of_nodes; i++) {
            if (dists[i] != DBL_MAX) {
                // In-buckets[i] we store the cost, dist and time for reaching node
                buckets[i][node] = NetworkPath(costs[i], dists[i], times[i], std::vector<int>());
            }
        }
    }
}

std::unordered_map<std::pair<int, int>, NetworkPath, pair_hash> ChGraph::combine_NtoN_passes (
    std::vector<int>& start_nodes,
    std::vector<int>& target_nodes,
    std::vector<std::unordered_map<int, NetworkPath>>& first_pass_buckets,
    std::vector<std::unordered_map<int, NetworkPath>>& second_pass_buckets,
    double cost_limit
) {
    // Object to store all pairs shortest paths between start_nodes and target_nodes
    std::unordered_map<std::pair<int, int>, NetworkPath, pair_hash> res;

    // Initialize all pairs with infinity path costs
    for (int start: start_nodes)
        for (int end: target_nodes)
            res[std::make_pair(start, end)] = NetworkPath(DBL_MAX, DBL_MAX, DBL_MAX, std::vector<int>()); // empty node list as a path.

    // Iterate over all nodes
    for (int i = 0; i < this->number_of_nodes; i++) {
        // Iterate over all sources that have reached node i
        for (auto& first_pass_entry: first_pass_buckets[i]) {
            int first_node = first_pass_entry.first;

            double first_cost = first_pass_entry.second.cost;
            double first_dist = first_pass_entry.second.distance;
            double first_time = first_pass_entry.second.travel_time;

            // Iterate over all targets that node i can reach
            for (auto& second_pass_entry: second_pass_buckets[i]) {
                int second_node = second_pass_entry.first;

                double second_cost = second_pass_entry.second.cost;
                double second_dist = second_pass_entry.second.distance;
                double second_time = second_pass_entry.second.travel_time;

                // Update shortest path between the source (first_node) and the target (second_node).
                if (first_cost != DBL_MAX && second_cost != DBL_MAX && (cost_limit == -1 || first_cost + second_cost < cost_limit)) {
                    auto cur = std::make_pair(first_node, second_node);
                    if (first_cost + second_cost < res[cur].cost)
                        res[cur] = NetworkPath(first_cost + second_cost,
                                               first_dist + second_dist,
                                               first_time + second_time,
                                               std::vector<int>()); // empty node list as a path
                }
            }
        }
    }

    // Return map for shortest paths between all sources and targets.
    return res;
}

// This method is not really important. Basic dijkstra call for 1-to-1 shortest path.
NetworkPath ChGraph::shortest_path_1_to_1_dijkstra(int from, int to) {
    std::vector<double>& costs = this->costs1;
    std::vector<double>& dists = this->dists1;
    std::vector<double>& times = this->times1;
    std::vector < Edge * >& parents = this->parents1;
    this->dijkstra(this->network_adjacency_with_shortcuts, from, to, costs, dists, times, parents, false, nullptr, false, -1,
                                     false, -1).first;
    std::vector<int> hops = this->unpack_path(from, to, parents);

    return NetworkPath(costs[to], dists[to], times[to], std::move(hops));
}

NetworkPath ChGraph::shortest_path_1_to_1(int from, int to, bool with_path, double cost_limit) {
    // Initialize vectors to be filled in the forward pass from 'from'
    std::vector<double>& forward_costs = this->costs1;
    std::vector<double>& forward_dists = this->dists1;
    std::vector<double>& forward_times = this->times1;
    std::vector < Edge * >& forward_parents = this->parents1;

    // Forward-upward-search from start node
    this->dijkstra(this->network_adjacency_with_shortcuts, from, -1, forward_costs, forward_dists, forward_times, forward_parents, false, nullptr,
                   true, -1, false, -1, cost_limit);

    // Initialize vectors to be filled in the backward pass from 'to'
    std::vector<double>& backward_costs = this->costs2;
    std::vector<double>& backward_dists = this->dists2;
    std::vector<double>& backward_times = this->times2;
    std::vector < Edge * >& backward_parents = this->parents2;
    
    // Backward-upward-search from target node
    std::pair<double, int> backward_result =  this->dijkstra(this->network_adjacency_reversed_with_shortcuts, to, -1, backward_costs, backward_dists, backward_times, backward_parents,
                          true, &forward_costs, true, -1, false, -1, cost_limit);

    // The backward upward search returns the optimal meeting point of the two searches
    int meeting_point = backward_result.second;

    // If no meeting point was found, that means there is no path between from and to. Return infinity-path
    if (meeting_point == -1)
        return NetworkPath(DBL_MAX, DBL_MAX, DBL_MAX, std::vector<int>()); // No path

    // Optimal path cost, distance and time
    double path_cost = forward_costs[meeting_point] + backward_costs[meeting_point];
    double path_dist = forward_dists[meeting_point] + backward_dists[meeting_point];
    double path_time = forward_times[meeting_point] + backward_times[meeting_point];

    // If the only path found surpasses the cost limit. return infinity.
    if (path_cost > cost_limit)
        return NetworkPath(DBL_MAX, DBL_MAX, DBL_MAX, std::vector<int>());
    
    // If the actual path nodes are needed
    if (with_path) {
        // Determine the path nodes of the path from 'from' to 'meeting_point'
        std::vector<int> forward_hops = unpack_path(from, meeting_point, forward_parents);
        // Determine the path nodes of the path from 'meeting_point' to 'to'
        std::vector<int> backward_hops = unpack_path(to, meeting_point, backward_parents);

        // Join to determine the actual path from 'from' to 'to'
        std::vector<int> hops = std::move(forward_hops);
        for(int i = (int)backward_hops.size() - 1; i >= 0; i--)
            hops.push_back(backward_hops[i]);

        // Remove duplicates
        hops.erase( std::unique( hops.begin(), hops.end() ), hops.end() );

        // Return optimal path with node list
        return NetworkPath(path_cost, path_dist, path_time, std::move(hops));
    }
    // Return optimal path without node list
    return NetworkPath(path_cost, path_dist, path_time, std::vector<int>());
}

std::unordered_map<std::pair<int, int>, NetworkPath, pair_hash> ChGraph::shortest_path_N_to_N (
    std::vector<int>& start_nodes,
    std::vector<int>& target_nodes,
    double cost_limit
) {
    // Buckets to store for each node the shortest path costs from each source node
    std::vector<std::unordered_map<int, NetworkPath>> first_pass_buckets = std::vector<std::unordered_map<int, NetworkPath>>(this->number_of_nodes);
    // Buckets to store for each node the shortest path costs to each target_node
    std::vector<std::unordered_map<int, NetworkPath>> second_pass_buckets = std::vector<std::unordered_map<int, NetworkPath>>(this->number_of_nodes);

    // Forward-upward-search from each source to fill first bucket
    this->ch_NtoN_pass(this->network_adjacency_with_shortcuts, start_nodes, first_pass_buckets, cost_limit);
    // Backward-upwarch-search from each target to fill second bucket
    this->ch_NtoN_pass(this->network_adjacency_reversed_with_shortcuts, target_nodes, second_pass_buckets, cost_limit);

    // Join and return shortest paths
    return combine_NtoN_passes(start_nodes, target_nodes, first_pass_buckets, second_pass_buckets, cost_limit);
}


std::vector<NetworkPath> ChGraph::shortest_path_1_to_N (int from, std::vector<int>& target_nodes, double cost_limit) {
    std::vector<double>& costs = this->costs1;
    std::vector<double>& dists = this->dists1;
    std::vector<double>& times = this->times1;
    std::vector<Edge*>& parents = this->parents1;

    // Perform a normal dijkstra search from start node
    this->dijkstra(this->network_adjacency_with_shortcuts, from, -1, costs, dists, times, parents, false, nullptr, false, -1, false, -1, cost_limit);

    std::vector<NetworkPath> res;

    // For each result, determine the shortest path (with empty node list)
    for (int target: target_nodes)
        res.push_back(NetworkPath(costs[target], dists[target], times[target], std::vector<int>()));

    return res;
}

std::vector<NetworkPath> ChGraph::shortest_path_N_to_1 (std::vector<int>& start_nodes, int to, double cost_limit) {
    std::vector<double>& costs = this->costs1;
    std::vector<double>& dists = this->dists1;
    std::vector<double>& times = this->times1;
    std::vector<Edge*>& parents = this->parents1;

    // Perform a backward dijkstra search from the target node
    this->dijkstra(this->network_adjacency_reversed_with_shortcuts, to, -1, costs, dists, times, parents, false, nullptr, false, -1, false, -1, cost_limit);

    std::vector<NetworkPath> res;

    // For each source node, determine the shortest path (with empty node list)
    for (int start: start_nodes)
        res.push_back(NetworkPath(costs[start], dists[start], times[start], std::vector<int>()));

    return res;
}

/*
// ---------------- This is an alternative implementation of the 1-to-N / N-to-1 queries ----------------------------

std::vector<NetworkPath> ChGraph::shortest_path_1_to_N (int from, std::vector<int>& target_nodes, double cost_limit) {
    std::vector<int> froms; froms.push_back(from);

    auto res_map = this->shortest_path_N_to_N(froms, target_nodes, cost_limit);

    std::vector<NetworkPath> res;
    for (auto to: target_nodes) {
        res.push_back(std::move(res_map[std::make_pair(from, to)]));
    }

    return res;
}

std::vector<NetworkPath> ChGraph::shortest_path_N_to_1 (std::vector<int>& start_nodes, int to, double cost_limit) {
    std::vector<int> tos; tos.push_back(to);

    auto res_map = this->shortest_path_N_to_N(start_nodes, tos, cost_limit);

    std::vector<NetworkPath> res;
    for (auto from: start_nodes) {
        res.push_back(std::move(res_map[std::make_pair(from, to)]));
    }

    return res;
}
*/

std::vector<int> ChGraph::unpack_edge(Edge *edge) {
    std::vector<int> res;
    if (edge != nullptr) {
        // If a normal edge
        if (edge->is_shortcut == false) {
            // in reverse order
            res.push_back(edge->from);
            res.push_back(edge->to);
        // Else if it's a shortcut
        } else if (edge->edgeA != -1 && edge->edgeB != -1) {
            // unpage first edge -> res in right order
            res = unpack_edge(&(this->edges[edge->edgeA]));
            std::vector<int> second_unpacked = unpack_edge(&(this->edges[edge->edgeB]));
            for (auto x: second_unpacked)
                res.push_back(x);
        }
    }
    return res;
}

std::vector<int> ChGraph::unpack_path(int from, int to, std::vector<Edge *> &parents) {
    std::vector<int> path;

    int cur = to;
    // Unpack all parents until from is reached (from has no parent, thus -1)
    while (cur != -1) {
        std::vector<int> nodes = unpack_edge(parents[cur]);
        for (int i = (int)nodes.size() - 1; i >= 0; i--)
            path.push_back(nodes[i]);

        if (parents[cur] == nullptr)
            break;

        assert(parents[cur]->to == cur);
        cur = parents[cur]->from;
    }

    std::reverse(path.begin(), path.end());

    // Remove duplicates 
    path.erase( std::unique( path.begin(), path.end() ), path.end() );
    return path;
}

int ChGraph::get_number_of_nodes() { return this->number_of_nodes; }

int ChGraph::get_number_of_edges() { return this->number_of_edges; }