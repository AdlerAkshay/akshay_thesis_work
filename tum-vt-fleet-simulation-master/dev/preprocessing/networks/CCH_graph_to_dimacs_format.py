import sys, os
from pathlib import Path
import pandas as pd

# DIMACS Format:

"""
 n m
 v11 v12 ... v1k1
 ...
 vn1 vn2 ... vnkn
"""

# n: number of nodes
# m: number of undirected edges (each counted once)
# n following lines. i-th line:
#   vij: neighbours of node i

def usage():
    print("Usage: python CCH_graph_to_dimacs_format.py [path_to_network]   --  path to network is where the base folder is located")

if __name__ == "__main__":

    if len(sys.argv) != 2:
        usage()
        sys.exit()

    network_path = Path(sys.argv[1])
    
    base_path = network_path / 'base'
    dimacs_save_path = base_path / 'graph_dimacs.graph'

    nodes_file = base_path / 'nodes.csv'
    edges_file = base_path / 'edges.csv'

    nodes = pd.read_csv(str(nodes_file))
    edges = pd.read_csv(str(edges_file))

    number_of_nodes = len(nodes)

    graph = [set() for _ in range(number_of_nodes)]

    froms = edges.from_node
    tos = edges.to_node

    for u, v in zip(froms, tos):
        graph[u].add(v)
        graph[v].add(u)

    number_of_edges = sum([len(x) for x in graph]) // 2

    with open(dimacs_save_path, 'w+') as fp:
        fp.write(str(number_of_nodes) + " " + str(number_of_edges) + '\n')
        for adj in graph:
            fp.write(" ".join([str(x+1) for x in adj]) + '\n')