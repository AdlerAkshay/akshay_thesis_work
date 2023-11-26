#ifndef UTILS_HG
#define UTILS_HG

#include <map>

#include "ChEdge.h"

// Used for creating a hash-map with pairs as keys
struct pair_hash
{
    template <class T1, class T2>
    std::size_t operator() (const std::pair<T1, T2> &pair) const {
        return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
    }
};

class ChGraph;
class CCHGraph;

/*
    Read the content of a the file in the given path

    @param file_path: the path to the file to read

    @return a vector containing the lines in the file as strings
*/
std::vector<std::string> get_rows_from_file(std::string file_path);

/*
    Splits the given string by the given delimiter character

    @param s: string to split
    @param delimiter: the character to split by

    @return a vector with the tokens after splitting the string
*/
std::vector<std::string> split_string(const std::string& s, const char delimiter=',');

/*
    Read the nodes from the given file_path and fill the given node information vectors

    @param nodes_file_path: path to the file containing the node informations
    @param node_x: vector to be filled with the nodes'x-coordinates
    @param node_y: vector to be filled with the nodes'y-coordinates
    @param node_is_stop: vector to be filled. True if node is stop_only, otherwise false

    @return the number of nodes read
*/
int read_nodes(
    std::string nodes_file_path, 
    std::vector<double>& node_x,
    std::vector<double>& node_y,
    std::vector<bool>& node_is_stop
);

/*
    Read the graph edges from the given file path

    @param edges_file_path: path to the file containing the edge informations
    @param graph: the graph to which the edges are to be added
    
    @return the number of edges read from the file
*/
int read_edges (
    std::string edges_file_path,
    ChGraph& graph
);

/*
    Read the CH node order from the given file path

    @param node_order_file_path: path to the file containing the saved node order
    @param node_order: vector where to save the read node orders
*/
void read_node_order(std::string node_order_file_path, std::vector<int>& node_order);

/*
    Read the contracted graph edges from the given file path

    @param contracted_graph_file_path: path the file containing the saved contracted graph edges
    @param graph: the graph for which the contracted graph is to be loaded
*/
void read_contracted_graph (
    std::string contracted_graph_file_path,
    ChGraph& graph
);

/*
    Save the node order to the given path

    @param directory: directory path where the node order is to be saved
    @param filename: the filename under which the node order file is to be saved
    @param node_order: the node order
*/
void save_node_order(std::string directory, std::string filename, std::vector<int>& node_order);

/*
    Save the contracted graph edges to the given path

    @param directory: directory path where the contracted graph edges are to be saved
    @param filename: the filename under which the contracted graph edges file is to be saved
    @param edges: the contracted graph edges
*/
void save_contracted_graph(std::string directory, std::string filename, std::vector<Edge>& edges);

/*
    Checks if the contracted graph is saved under the given path

    @param path: path where the contracted graph is to be looked up

    @return true if one exists, false otherwise
*/
bool contracted_graph_is_present(std::string path);

/*
    Load the nested dissection order from the given path. (for CCH)
    This method expects a file called cch_nd_order to be present in the base folder of the given network directory

    @param network_path: path to the network (where the base folder is located)

    @return vector with i-th element being the order of the i-th node
*/
std::vector<int> load_nd_order(std::string network_path);

/*
    Saves the graph edges after the first phase of the network preprocessing of the CCH graph.
    (Saved under cch_graph/phase_one_edges.csv)

    @param network_path: path to the network (where the base folder is located)
    @param edges: vector with the edges after runnning phase 1
*/
void save_phase_one_cch_graph(std::string network_path, std::vector<Edge>& edges);

/*
    Saves the graph edges after the second phase of the network preprocessing of the CCH graph.
    (Saved under cch_graph/phase_two_edges.csv)

    @param network_path: path to the network (where the base folder is located)
    @param edges: vector with the edges after runnning phase 2
*/
void save_phase_two_cch_graph(std::string network_path, std::vector<Edge>& edges);

/*
    Loads the graph edges after the first phase of the network preprocessing of the CCH graph.

    @param network_path: path to the network (where the base folder is located)
    @param graph: graph to load the edges to
*/
void load_phase_one_cch_graph(std::string network_path, CCHGraph& graph);

/*
    Loads the graph edges after the second phase of the network preprocessing of the CCH graph.

    @param network_path: path to the network (where the base folder is located)
    @param graph: graph to load the edges to
*/
void load_phase_two_cch_graph(std::string network_path, CCHGraph& graph);

/*
    Load the new travel times from the given file path

    @param filepath: path to file contining the new travel times

    @return map (u, v) to the travel time of edge from u to v
*/
std::map<std::pair<int, int>, double> load_edge_travel_times(std::string filepath);

#endif