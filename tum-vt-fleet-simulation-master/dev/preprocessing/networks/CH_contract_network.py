import sys, os
from pathlib import Path
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))
from dev.routing.contraction_hierarchies_router.PyNetwork import PyGraph


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
    ch_graph_path = network_path / 'ch_graph'
    
    if not(network_path.exists() and base_path.exists()):
        usage()
        sys.exit()

    if not(ch_graph_path.exists()):
        ch_graph_path.mkdir(parents=True, exist_ok=True)

    print(" ... load graph")
    g = PyGraph(str(network_path.resolve()).encode())
    print(" ... start contracting")
    g.build_contraction_hierarchy()
    print(" ... save files")
    g.save_contraction_hierarchy()


