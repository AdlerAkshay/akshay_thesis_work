#include <cfloat>
#include <cassert>

#include "ChEdge.h"


/*
    Creates a shortcut from edgeA and edgeB. It is assumed that edgeA.to == edgeB.from

    @param edgeA: a pointer to the first edge that the shortcut replaces
    @param edgeB: a pointer to the second edge that the shortcut replaces
    @param edgeA_id: the index of edgeA in the edges vector of the graph
    @param edgeB_id: the index of edgeB in the edges vector of the graph
    @param inf_cost: true if the distance and travel time are to be set to infinity, false if the sum of edgeA and edgeB d and tt

    @return a shortcut edge from edgeA.from to edgeB.to
*/
Edge create_shortcut_from_edges(Edge* edgeA, Edge* edgeB, int edgeA_id, int edgeB_id, bool inf_cost) {
    // shortcut has to be created from two non null edges.
    assert(edgeA != nullptr && edgeB != nullptr);
    // edgeA and edgeB must meet in the middle
    assert(edgeA->to == edgeB->from);

    int from = edgeA->from;
    int to = edgeB->to;

    // Set distance and time to infinity if inf_cost is set to true.
    // Otherwise, sum up edgeA and edgeB distances and times.
    double d = inf_cost ? DBL_MAX : edgeA->d + edgeB->d;
    double t = inf_cost ? DBL_MAX : edgeA->t + edgeB->t;

    // Create a new edge to be returned
    Edge res = Edge(from, to, d, t);

    // Add shortcut information to the new edge.
    res.is_shortcut = true;
    res.edgeA = edgeA_id;
    res.edgeB = edgeB_id;

    return res;
}

/*
    Creates an edge that is the reverse direction of the given edge

    @param edge: the edge to reverse

    @return a new edge from edge.to to edge.from. edgeA and edgeB are swapped and reversed in the new edge. 
*/
Edge reverse_edge(Edge* edge) {
    // Create a copy of the given edge
    Edge res = Edge(*edge);

    // Swap the endpoints of the edge
    int tmp = res.to;
    res.to = res.from;
    res.from = tmp;

    // Reverse and swap edgeA and edgeB if not null
    // Assumption here that for edge with index 2i has its reverse at 2i+1.
    if (edge->edgeA != -1 && edge->edgeB != -1) {
        res.edgeA = edge->edgeB % 2 ? edge->edgeB - 1 : edge->edgeB + 1;
        res.edgeB = edge->edgeA % 2 ? edge->edgeA - 1 : edge->edgeA + 1;
    }

    return res;
}

