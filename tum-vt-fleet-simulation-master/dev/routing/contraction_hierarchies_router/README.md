# Using the contraction hierarchies router
### 0. Installing requirements:
First, we need to have g++ compiler for c++ installed. Details on how to install it are illustrated here https://www3.cs.stonybrook.edu/~alee/g++/g++.html
Then, we need to have python installed. Details on how to install it are illustrated here https://www.journaldev.com/30076/install-python-windows-10
Make sure you select pip to be installed in the optional features step.
Finally, there are some python libraries that are required and can be installed via pip using the following commands:
```
pip install Cython
pip install numpy
pip install pandas
pip install pathlib
```
### 1. Pre-contracting network:
To use the contraction hierarchies router. First, we build the contracted network and save it. To do that:
```
cd scripts/
python contract_network.py [NETWORK_PATH]
```
NETWORK_PATH is the path to the network to contract. A sub-directory 'base/' is expected with two files describing the network: 'nodes.csv' and 'edges.csv'.
This will save the contracted network in [...]data/networks/[NETWORK_NAME]/ch_graph.
The saved contracted graph will consist of two files:
* node_order.csv: This contains the order by which the nodes were contracted
* chgraph.csv: The contracted network edges and shortcuts

More details about the formats of the files in "docs/documentation/data_specification/network.md"

### 2. Building router:

In src/routing/contraction_hierarchies_router, the router is built using the following command
```
python setup.py build_ext --inplace
```

### 3. Using the router:

The router is used in the same way the cpp-router is used. See src/routing/NetworkContractionHierarchies.py  (An implementation of the NetworkBasic superclass).