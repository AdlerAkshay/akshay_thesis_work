# Using the customizable contraction hierarchies router
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
## 1. Computing the nested dissection order:

For computing the nested dissection order, first we have to convert the graph into the dimacs format.

```
cd scripts/
python graph_to_dimacs_format.py [NETWORK_PATH]
```

The graph will then be saved in dimacs format under [NETWORK_PATH]/base/graph_dimacs.graph.
Then, to compute the nested dissection order we use KaHIP https://github.com/KaHIP/KaHIP
There is an installation guide in the github page. (Tested on ubuntu 20.04)
After installing KaHIP, we compute the nested dissection order with the following command.

```
// In the KaHIP main directory
./deploy/node_ordering [path-to-dimacs-graph (i.e. NETWORK_PATH/base/graph_dimacs.graph)]
```

The output of the node ordering program is written to a file 'tmpnodeordering' move it to the network base folder under the name 'cch_nd_order'.

```
mv tmpnodeordering [NETWORK_PATH]/base/cch_nd_order
```

At last, replace the tab spaces ('\t') in the file with commas (',') (e.g. via 'change all occurences' option in any editor)

P.S: This would be better wrapped in a script. However, the KaHIP program did not work for me on Windows, so I had to run it on a Linux VM. (Maybe for future improvements, this ?)

## 2. Pre-contracting the network:

```
cd scripts/
python contract_network.py [NETWORK_PATH]
```
NETWORK_PATH is the path to the network to contract. A sub-directory 'base/' is expected with two files describing the network: 'nodes.csv' and 'edges.csv'.
This will save the contracted network in [...]data/networks/[NETWORK_NAME]/cch_graph.
The saved contracted graph will consist of two files:
* phase_one_edges.csv: The graph after contracting nodes following the nested dissection order
* phase_two_edges.csv: The final contracted graph after triangle enumeration of the customizable contraction hierarchies

More details about the formats of the files in "docs/documentation/data_specification/network.md"

### 3. Building router:

In src/routing/customizable_contraction_hierarchies_router, the router is built using the following command
```
python setup.py build_ext --inplace
```

### 4. Using the router:

The router is used in the same way the cpp-router is used. See src/routing/NetworkCustomizableContractionHierarchies.py  (An implementation of the NetworkBasic superclass).