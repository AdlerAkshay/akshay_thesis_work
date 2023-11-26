#include <vector>
#include <cassert>
#include <cfloat>
#include <algorithm>
#include <math.h>
#include <iostream>
#include <unordered_set>
#include <map>
#include <numeric>

#include "CCHGraph.h"

CCHGraph::CCHGraph(std::string graph_path, bool load_phase_one_graph, bool load_phase_two_graph): ChGraph(graph_path, false) {
    std::cout << "Loading the nested dissection order ... ";
    this->load_nested_dissection_order();
    std::cout << "Done." << std::endl;

    if (load_phase_two_graph) {
        std::cout << "Loading phase two graph ... ";
        this->load_phase_two_graph();
        std::cout << "Done." << std::endl;
        this->phase1Complete = true;
        this->phase2Complete = true;
        this->ch_built = true;
    } 
    else if (load_phase_one_graph) {
        std::cout << "Loading phase one graph ... ";
        this->load_phase_one_graph();
        std::cout << "Done." << std::endl;
        this->phase1Complete = true;
        std::cout << "Number of edges after loading is : " << this->edges.size() << std::endl;
        std::cout << "Running triangle enumeration and updating shortcut weights ... ";
        this->run_triangle_enumeration_and_update_weights();
        std::cout << "Done." << std::endl;
    } else {
        // If load parameters are set to false. The graph will not be contracted. Uncomment if a contraction is wanted in this case.
        
        /*
        std::cout << "Nothing to load" << std::endl;
        std::cout << "Contracting nodes in order ... ";
        this->contract_nodes_in_order();
        std::cout << "Done." << std::endl;
        std::cout << "Running triangle enumeration and updating weights ... ";
        this->run_triangle_enumeration_and_update_weights();
        std::cout << "Done." << std::endl;
        */
    }
}

void CCHGraph::load_nested_dissection_order() {
    assert(this->ndComputed == false);
    // Load order from <network_path>/base/cch_nd_order
    this->node_orders = load_nd_order(this->network_path);
    // Set flag
    this->ndComputed = true;
}

void CCHGraph::contract_nodes_in_order() {
    // must load nested dissection order first
    assert(this->ndComputed);
    assert(!this->phase1Complete);

    // mark all nodes as non-contracted
    this->is_contracted = std::vector<bool>(this->number_of_nodes, false);

    // Compute list of nodes to be contracted in order of this->node_orders
    std::vector<int> nodes_in_order(this->number_of_nodes);
    for(int i = 0; i < this->number_of_nodes; i++) {
        int rank = this->node_orders[i];
        nodes_in_order[rank-1] = i;
    }

    // For each node in order
    for (auto node: nodes_in_order) {
        // Do not contract stop_only nodes, to avoid using shortcuts going through a stop only node.
        if (!node_is_stop[node]) {
            for (auto e1_id: this->network_adjacency_reversed_with_shortcuts[node]) {
                // u -> node
                Edge *e1 = &(this->edges[e1_id]);
                int u = e1->to;
                if (this->is_contracted[u])
                    continue;


                for (auto e2_id: this->network_adjacency_with_shortcuts[node]) {
                    // node -> v
                    Edge *e2 = &(this->edges[e2_id]);
                    int v = e2->to;
                    if (this->is_contracted[v] || u == v)
                        continue;
                    
                    // Get the reverse edge id.
                    int e1_rev_id = e1_id % 2 ? e1_id - 1 : e1_id + 1;
                    Edge *e1_rev = &(this->edges[e1_rev_id]);

                    // Check if there already exists an edge between u and v
                    bool should_add_edge = true;
                    for (auto e_tmp: this->network_adjacency_reversed_with_shortcuts[v])
                        if (this->edges[e_tmp].to == u) {
                            should_add_edge = false;
                            break;
                        }

                    // Add shortcut if none exist
                    if (should_add_edge)
                        this->add_edge(create_shortcut_from_edges(e1_rev, e2, e1_rev_id, e2_id, true));
                }
            }
        }
        
        // Mark node as contracted
        this->is_contracted[node] = true;
    }
    // Set flag
    this->phase1Complete = true;
}

// A workaround sorting method
std::vector<std::pair<Edge*, Edge*>> CCHGraph::sort_arcs(std::vector<std::pair<Edge*, Edge*>>& arcs) {
    std::vector<std::pair<Edge*, Edge*>> res(arcs.size());

    // Create vector with i-th element is the (sorted) pair of orders of endpoints of *(arcs[i].first)
    std::vector<std::pair<int, int>> edge_levels;
    for (auto& arc: arcs) {
        int u_level = this->node_orders[arc.first->from];
        int v_level = this->node_orders[arc.first->to];

        edge_levels.push_back(std::make_pair(std::min(u_level, v_level), std::max(u_level, v_level)));
    }

    // indices is a vector {0, 1, 2, ...}
    std::vector<int> indices(edge_levels.size());
    std::iota(indices.begin(), indices.end(), 0);

    // Sort edge_levels but keep track of sorted positions in indices
    std::stable_sort(indices.begin(), indices.end(),
       [&edge_levels](std::size_t i1, std::size_t i2) {return edge_levels[i1] < edge_levels[i2];});

    // Sort arcs the same way edge_levels was sorted
    for (std::size_t i = 0; i < indices.size(); i++) {
        res[i] = arcs[indices[i]];
    }

    return res;
}

void CCHGraph::run_triangle_enumeration_and_update_weights() {
    assert(this->phase1Complete);

    // The arcs. To be sorted by level. lowest first
    std::vector<std::pair<Edge*, Edge*>> arcs;
    for (int i = 0; i < this->edges.size(); i += 2) {
        //if (this->edges[i].is_shortcut)
            arcs.emplace_back(&(this->edges[i]), &(this->edges[i+1]));
    }
    
    /*
        This is undefined behaviour in C++ :( a more complicated method is possible though. See sort_arcs() :)

    auto compare_edges_by_level = [&] (const std::pair<Edge*, Edge*>& e1, const std::pair<Edge*, Edge*>& e2) {
        int e1_u = e1.first->from, e1_v = e1.first->to;
        int e2_u = e2.first->from, e2_v = e2.first->to;

        return std::min(this->node_orders[e1_u], this->node_orders[e1_v]) < std::min(this->node_orders[e2_u], this->node_orders[e2_v]) ||
        std::max(this->node_orders[e1_u], this->node_orders[e1_v]) < std::max(this->node_orders[e2_u], this->node_orders[e2_v]);
    };
    std::sort(arcs.begin(), arcs.end(), compare_edges_by_level);
    */

    arcs = this->sort_arcs(arcs);

    // Asserting that the arcs are correctly sorted.
    for (std::size_t i = 0; i < arcs.size() - 1; i++) {
        auto& arc1 = arcs[i];
        auto& arc2 = arcs[i + 1];

        int u1_level = this->node_orders[arc1.first->from];
        int v1_level = this->node_orders[arc1.first->to];

        int u2_level = this->node_orders[arc2.first->from];
        int v2_level = this->node_orders[arc2.first->to];
        
        int m1 = std::min(u1_level, v1_level);
        int M1 = std::max(u1_level, v1_level);

        int m2 = std::min(u2_level, v2_level);
        int M2 = std::max(u2_level, v2_level);

        assert(m1 < m2 || (m1 == m2 && M1 <= M2));
    }

    // enumerate lower triangles and update weight
    for (auto& arc: arcs) {
        Edge* e = arc.first;
        Edge* e_rev = arc.second;

        int from = e->from, to = e->to; // edge endpoints

        // Find all neighbours of from
        std::unordered_set<int> from_neighbours;
        for (auto next_edge: this->network_adjacency_with_shortcuts[from]) {
            int next = this->edges[next_edge].to;
            if (this->node_orders[next] < this->node_orders[from])
                from_neighbours.insert(next_edge);
        }

        // Find neighbours v of to that are also neighbours of from with from -> v -> to and update from -> if needed.
        for (auto prev_edge: this->network_adjacency_reversed_with_shortcuts[to]) {
            int next = this->edges[prev_edge].to;
            if (this->node_orders[next] < this->node_orders[to]) {
                for (auto neib_edge: from_neighbours) {
                    if (this->edges[neib_edge].to == next) {
                        double new_cost = get_edge_cost(neib_edge) + get_edge_cost(prev_edge);
                        double new_d = this->edges[neib_edge].d + this->edges[prev_edge].d;
                        double new_t = this->edges[neib_edge].t + this->edges[prev_edge].t;
                        
                        if (new_cost < get_edge_cost(*e)) {
                            e->d = new_d;
                            e->t = new_t;
                            // We have to update the reversed edge too. 
                            e_rev->d = new_d;
                            e_rev->t = new_t;
                        }
                    }
                }
            }
        }

    }
    this->phase2Complete = true;
    this->ch_built = true;
}

void CCHGraph::clear_edges_and_adjacency_lists() {
    this->edges.clear();
    // Clear adjacency of each node
    for(auto& adj : this->network_adjacency_with_shortcuts)
        adj.clear();
    // Clear reversed adjacency of each node
    for(auto& adj : this->network_adjacency_reversed_with_shortcuts)
        adj.clear();
}

void CCHGraph::save_phase_one_graph() {
    assert(this->phase1Complete);
    std::cout << "Saving the phase one cch graph ... "; 
    save_phase_one_cch_graph(this->network_path, this->edges);
    std::cout << "Done." << std::endl;
}

void CCHGraph::save_phase_two_graph() {
    assert(this->phase2Complete);
    std::cout << "Saving the phase two cch graph ... "; 
    save_phase_two_cch_graph(this->network_path, this->edges);
    std::cout << "Done." << std::endl;
}

void CCHGraph::load_phase_one_graph() {
    this->clear_edges_and_adjacency_lists();
    load_phase_one_cch_graph(this->network_path, *this);
}

void CCHGraph::load_phase_two_graph() {
    this->clear_edges_and_adjacency_lists();
    load_phase_two_cch_graph(this->network_path, *this);
}

void CCHGraph::load_new_edge_travel_times(std::string file_path) {
    // Set all shortcut weights to DBL_MAX
    for (auto& edge: this->edges) {
        if (edge.is_shortcut) {
            edge.d = DBL_MAX;
            edge.t = DBL_MAX;
        }
    }

    // Read new edge weights in a map
    std::map<std::pair<int, int>, double> edge_to_tt = load_edge_travel_times(file_path);

    // Update travel times of existing edges
    for(int i = 0; i < (int)this->edges.size(); i += 2) {
        int u = this->edges[i].from;
        int v = this->edges[i].to;

        assert(edge_to_tt.find(std::make_pair(u,v)) != edge_to_tt.end());
        this->edges[i].t = edge_to_tt[std::make_pair(u, v)];
        this->edges[i+1].t = edge_to_tt[std::make_pair(u, v)];
    }

    // Run triangle enumeration again
    this->run_triangle_enumeration_and_update_weights();
}

