#pragma once

struct Edge {
    int from;
    int to;
    double d; // distance
    double t; // travel time

    bool is_shortcut; // true iff the edge is a shortcut
    int edgeA; // -1 if is_shortcut == false else the first edge that the shortcut replaces
    int edgeB; // -1 if is_shortcut == false else the second edge that the shortcut replaces

    // Constructor to create a non-shortcut edge 
    Edge(int from, int to, double d, double t): from(from), to(to), d(d), t(t), is_shortcut(false), edgeA(-1), edgeB(-1) {}

    // Copy-constructor
    Edge(const Edge& edge): from(edge.from), to(edge.to), d(edge.d), t(edge.t), is_shortcut(edge.is_shortcut), edgeA(edge.edgeA), edgeB(edge.edgeB) {}

    // Function to change the edge distance and travel time
    void set_costs(double d, double t) {
        this->d = d; this->t = t;
    }
};

/*
    Creates a shortcut from edgeA and edgeB. It is assumed that edgeA.to == edgeB.from

    @param edgeA: a pointer to the first edge that the shortcut replaces
    @param edgeB: a pointer to the second edge that the shortcut replaces
    @param edgeA_id: the index of edgeA in the edges vector of the graph
    @param edgeB_id: the index of edgeB in the edges vector of the graph
    @param inf_cost: true if the distance and travel time are to be set to infinity, false if the sum of edgeA and edgeB d and tt

    @return a shortcut edge from edgeA.from to edgeB.to
*/
Edge create_shortcut_from_edges(Edge* edgeA, Edge* edgeB, int edgeA_id, int edgeB_id, bool inf_cost = false);

/*
    Creates an edge that is the reverse direction of the given edge

    @param edge: the edge to reverse

    @return a new edge from edge.to to edge.from. edgeA and edgeB are swapped and reversed in the new edge. 
*/
Edge reverse_edge(Edge* edge);