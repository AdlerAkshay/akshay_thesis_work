#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <assert.h>
#include <tuple>
#include <cfloat>
#include <filesystem>
#include <fstream>
#include <map>

#include "utils.h"
#include "ChGraph.h"
#include "../customizable_contraction_hierarchies_router/CCHGraph.h"

// Reads a file and returns its lines in a vector.
std::vector<std::string> get_rows_from_file(std::string file_path) {
    std::fstream fin;
    // vector to store the file lines to
    std::vector<std::string> rows;
    std::string s, row;

    // open file
    fin.open(file_path);

    // while still lines left
    while(fin >> s) {
        getline(fin, row, '\n'); // read line
        rows.push_back(s);
    }

    // close file
    fin.close();

    // return file lines.
    return rows;
}

// Takes a string and splits it by the given delimiter.
std::vector<std::string> split_string(const std::string& s, const char delimiter/*=','*/) {
    // vector to save the string tokens after splitting
    std::vector<std::string> tokens;

    std::string word;
    // stream to read from the string to split
    std::stringstream line(s);

    // read until the next delimiter
    while(std::getline(line, word, delimiter))
        tokens.emplace_back(word);
    
    // Special case if the string ends with the delimiter
    if (s.size() && s[s.size() - 1] == delimiter) {
        //std::cout << "CAREFUL: line ended with " << delimiter << " read as empty string.\n";
        tokens.push_back("-1");
    }

    return tokens;
}

// Read the nodes from the file_path given. Assumes the nodes are sorted by index in the file.
int read_nodes(
    std::string nodes_file_path, 
    std::vector<double>& node_x,
    std::vector<double>& node_y,
    std::vector<bool>& node_is_stop
) {
    std::vector<std::string> lines = get_rows_from_file(nodes_file_path);

    std::cout << "nodes.csv lines: " << lines.size() << std::endl;
    std::cout << "First line: " << lines[0] << "#" << std::endl;
    std::cout << "Last line: " << lines[lines.size() - 1] << "#" << std::endl;
    std::cout << std::endl;

    int count = 0;

    // Skip first line (it contains column names)
    for(std::size_t i = 1; i < lines.size(); ++i) {
        auto& line = lines[i];

        std::vector<std::string> words = split_string(line);

        assert(words.size() >= 4);

        //each line is of the following format: node_index,is_stop_only,x,y

        int node_index = stoi(words[0]);
        assert(node_index == count); // Assert that the nodes are in order.
        count += 1;
        bool is_stop_only = words[1] == "True";
        double x = std::stod(words[2]);
        double y = std::stod(words[3]);

        node_x.push_back(x);
        node_y.push_back(y);
        node_is_stop.push_back(is_stop_only);
    }

    return count;
}

int read_edges(
    std::string edges_file_path,
    ChGraph& chGraph
) {
    std::vector<std::string> lines = get_rows_from_file(edges_file_path);
    
    std::cout << "edges.csv lines: " << lines.size() << std::endl;
    std::cout << "First line: " << lines[0] << "#" << std::endl;
    std::cout << "Last line: " << lines[lines.size() - 1] << "#" << std::endl;
    std::cout << std::endl;

    int count = 0;
    // Skip first line (it contains column names)
    for(std::size_t i = 1; i < lines.size(); ++i) {
        auto& line = lines[i];
        std::vector<std::string> words = split_string(line);

        // Each line is of the following format: from,to,d,t,something-else :)

        assert(words.size() == 5);

        int from = stoi(words[0]);
        int to = stoi(words[1]);
        double d = std::stod(words[2]);
        double t = std::stod(words[3]);

        Edge edge = Edge(from, to, d, t);
        chGraph.add_edge(std::move(edge));

        count ++;
    }

    return count;
}

void read_node_order(std::string node_order_file_path, std::vector<int>& node_order) {
    std::vector<std::string> lines = get_rows_from_file(node_order_file_path);

    node_order.clear();
    // Each line is a single integer. i-th line is order of i-th node
    for (auto& order_string: lines)
        node_order.push_back(stoi(order_string));
}

void read_contracted_graph(std::string edges_file_path, ChGraph& graph) {
    std::vector<std::string> lines = get_rows_from_file(edges_file_path);
    // Skip first line (it contains column names)
    for (std::size_t i = 1; i < lines.size(); i++) {
        std::vector<std::string> words = split_string(lines[i], ',');
        
        assert(words.size() == 7);

        // parse edge information
        int from = stoi(words[0]);
        int to = stoi(words[1]);
        double d = std::stod(words[2]);
        double t = std::stod(words[3]);
        bool is_shortcut = words[4] == "1";
        int edgeA = stoi(words[5]);
        int edgeB = stoi(words[6]);

        if (is_shortcut) assert(edgeA != -1 && edgeB != -1);

        Edge edge = Edge(from, to, d, t);
        edge.is_shortcut = is_shortcut;
        edge.edgeA = edgeA;
        edge.edgeB = edgeB;

        // add edge to the graph
        graph.add_edge(std::move(edge));
    }
}

void save_node_order(std::string directory, std::string filename, std::vector<int>& node_order) {
    //std::filesystem::create_directories(directory);

    std::ofstream out(directory + "/" + filename, std::ios::trunc);

    // i-th line is order of i-th node
    for (auto& order: node_order)
    out << order << "\n";

    out.close();
}

void save_contracted_graph(std::string directory, std::string filename, std::vector<Edge>& edges) {
    //std::filesystem::create_directories(directory);

    std::ofstream out(directory + "/" + filename, std::ios::trunc);

    assert(edges.size() % 2 == 0);
    out << "from,to,d,t,is_shortcut,edgeA,edgeB\n";
    for (std::size_t i = 0; i < edges.size(); i += 2) { // Here we skip odd indices (These contain the reverse edge of the previous index)
        Edge* edge = &(edges[i]);
        out << edge->from << "," << edge->to << "," << edge->d << "," << edge->t << "," << edge->is_shortcut << "," << edge->edgeA << "," << edge->edgeB << "\n";
    }

    out.close();
}

// Checks if a file is present at the given path
bool file_is_present(std::string filepath) {
    std::ifstream ifile;
    ifile.open(filepath);
    bool ret = ifile.good();
    ifile.close();
    return ret;
}

bool contracted_graph_is_present(std::string path) {
    return file_is_present(path + "/ch_graph/chgraph.csv") && file_is_present(path + "/ch_graph/node_order.csv");
}

std::vector<int> load_nd_order(std::string network_path) {
    assert(file_is_present(network_path + "/base/cch_nd_order"));
    std::vector<std::string> lines = get_rows_from_file(network_path + "/base/cch_nd_order");

    // One line less (first line is ignored)
    std::vector<int> order(lines.size() - 1);
    for (int i = 1; i < lines.size(); i++) {
        std::string line = lines[i];
        std::vector<std::string> entries = split_string(line, ',');

        // node_index,node_order. For i-th line node_index == i so we can ignore it
        assert(entries.size() == 2);

        order[i - 1] = stoi(entries[1]);
    }

    return order;
}

void save_phase_one_cch_graph(std::string network_path, std::vector<Edge>& edges) {
    std::ofstream out(network_path + "/cch_graph/phase_one_edges.csv", std::ios::trunc);

    assert(edges.size() % 2 == 0);
    // First line is column names
    out << "from,to,d,t,is_shortcut,edgeA,edgeB\n";
    for (std::size_t i = 0; i < edges.size(); i += 2) {
        Edge* edge = &(edges[i]);
        // For shortcuts: set distance and travel time to -1 (placeholder for infinity)
        if (edge->is_shortcut)
            out << edge->from << "," << edge->to << "," << -1 << "," << -1 << "," << edge->is_shortcut << "," << edge->edgeA << "," << edge->edgeB << "\n";
        else
            out << edge->from << "," << edge->to << "," << edge->d << "," << edge->t << "," << edge->is_shortcut << "," << edge->edgeA << "," << edge->edgeB << "\n";
        
    }

    out.close();
}

void save_phase_two_cch_graph(std::string network_path, std::vector<Edge>& edges) {
    std::ofstream out(network_path + "/cch_graph/phase_two_edges.csv", std::ios::trunc);

    assert(edges.size() % 2 == 0);
    out << "from,to,d,t,is_shortcut,edgeA,edgeB\n";
    for (std::size_t i = 0; i < edges.size(); i += 2) {
        Edge* edge = &(edges[i]);
        out << edge->from << "," << edge->to << "," << edge->d << "," << edge->t << "," << edge->is_shortcut << "," << edge->edgeA << "," << edge->edgeB << "\n";  
    }

    out.close();
}

bool phase_one_cch_graph_is_present(std::string network_path) {
    return file_is_present(network_path + "/cch_graph/phase_one_edges.csv");
}

bool phase_two_cch_graph_is_present(std::string network_path) {
    return file_is_present(network_path + "/cch_graph/phase_two_edges.csv");
}

// Assumes that graph.clear_edges_and_adjaceny_lists() is called already.
void load_phase_one_cch_graph(std::string network_path, CCHGraph& graph) {
    assert(phase_one_cch_graph_is_present(network_path));

    std::vector<std::string> lines = get_rows_from_file(network_path + "/cch_graph/phase_one_edges.csv");

    // Skip first line containing column names
    for (std::size_t i = 1; i < lines.size(); i++) {
        std::vector<std::string> words = split_string(lines[i], ',');
        

         // Line format: from,to,d,t,is_shortcut,edgeA,edgeB 
        assert(words.size() == 7);

        int from = stoi(words[0]);
        int to = stoi(words[1]);
        double d = std::stod(words[2]);
        double t = std::stod(words[3]);
        if (d == -1) d = DBL_MAX;
        if (t == -1) t = DBL_MAX;
        bool is_shortcut = words[4] == "1";
        int edgeA = stoi(words[5]);
        int edgeB = stoi(words[6]);
        
        if (is_shortcut) assert(edgeA != -1 && edgeB != -1);

        Edge edge = Edge(from, to, d, t);
        edge.is_shortcut = is_shortcut;
        edge.edgeA = edgeA;
        edge.edgeB = edgeB;

        graph.add_edge(std::move(edge));
    }
}

void load_phase_two_cch_graph(std::string network_path, CCHGraph& graph) {
    assert(phase_two_cch_graph_is_present(network_path));

    std::vector<std::string> lines = get_rows_from_file(network_path + "/cch_graph/phase_two_edges.csv");
    
    // Skip first line containing column names
    for (std::size_t i = 1; i < lines.size(); i++) {
        std::vector<std::string> words = split_string(lines[i], ',');
        
         // Line format: from,to,d,t,is_shortcut,edgeA,edgeB 
        assert(words.size() == 7);

        int from = stoi(words[0]);
        int to = stoi(words[1]);
        double d = std::stod(words[2]);
        double t = std::stod(words[3]);
        bool is_shortcut = words[4] == "1";
        int edgeA = stoi(words[5]);
        int edgeB = stoi(words[6]);

        if (is_shortcut) assert(edgeA != -1 && edgeB != -1); // Shortcuts must have edgeA and edgeB pointing to edge indices and not -1

        Edge edge = Edge(from, to, d, t);
        edge.is_shortcut = is_shortcut;
        edge.edgeA = edgeA;
        edge.edgeB = edgeB;

        graph.add_edge(std::move(edge));
    }
}

std::map<std::pair<int, int>, double> load_edge_travel_times(std::string filepath) {
    std::vector<std::string> lines = get_rows_from_file(filepath);


    std::map<std::pair<int, int>, double> res;
    for(int i = 1; i < (int)lines.size(); i++) {
        std::vector<std::string> words = split_string(lines[i], ',');
        int u = stoi(words[0]);
        int v = stoi(words[1]);
        double tt = stod(words[2]);

        res[std::make_pair(u, v)] = tt;
    }

    return std::move(res);
}
