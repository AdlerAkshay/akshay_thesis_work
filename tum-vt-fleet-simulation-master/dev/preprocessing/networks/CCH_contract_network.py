'''
NOTE It is expected that the network_path contains a file with the nested dissection order already. Check ../README.md for more details.
NOTE If the nested dissection file contains '\t' instead of ' ' as a seperator, then replace with ' ' (use change all occurences option for example)
'''

import sys, os
from pathlib import Path


def usage():
    print("Usage: python contract_network.py [path_to_network]   --  path to network is where the base folder is located")

def execute_command(command):
    os.system(command)

if __name__ == "__main__":

    if len(sys.argv) != 2:
        usage()
        sys.exit()
    
    network_path = Path(sys.argv[1])
    
    base_path = network_path / 'base'
    cch_graph_path = network_path / 'cch_graph'

    nd_order_path = network_path /'base' / 'cch_nd_order'

    if not(nd_order_path.exists()):
        print("Before contracting the network, compute nested dissection order. See ../README.md")
        sys.exit()
    
    if not(network_path.exists() and base_path.exists()):
        usage()
        sys.exit()

    if not(cch_graph_path.exists()):
        cch_graph_path.mkdir(parents=True, exist_ok=True)

    os.chdir(Path(__file__).parent.parent / 'build')
    execute_command('g++ -std=c++17 -Wall ../../contraction_hierarchies_router/ChEdge.cpp ../../contraction_hierarchies_router/utils.cpp ../../contraction_hierarchies_router/ChGraph.cpp ../CCHGraph.cpp ../main.cpp -o router.exe')
    execute_command('router.exe ' + str(network_path.resolve()))
