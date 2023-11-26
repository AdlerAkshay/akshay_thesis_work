# Zone System

## General Information (Mandatory)

* geometry is saved as polygons in data/zones/{zone_system_name}/polygon_definition.geojson
* reference system of these polygons is saved in data/zones/{zone_system_name}/crs.info which simply contains epsg:{epsg_code}
* general information is saved in data/zones/{zone_system_name}/general_information.csv with following inputs

Column Name | Data Type | Description
-- | -- | --
zone_id | int | mandatory; this index will be used as reference; should be a range starting from 0!
zone_name | str | optional
park_cost_scale_factor | float | optional; required for some frameworks, will be set to 1 if not available; in cent (per trip)
toll_cost_scale_factor | float | optional; required for some frameworks, will be set to 1 if not available; in cent per km
add_park_search_duration | float | optional; can be used to model park search traffic and access/egress times between parking lot and actual destination in certain zones
offer_first_last_mile | bool | optional; set to True if not available

* future: additional files for further aggregation levels, spatial correlation matrices etc.


## Network-Specific Information

* matching of a zone system on a street network
* definition on node and edge level
* currently, there is a redundancy in defining both nodes and edges as edges are counted to the zone of the origin
* for future: an edge can have two zones; the second entry only contains a zone_id if the edge crosses zones (can be useful for Cordon toll)

### Node - Zone Relation (Mandatory)

* saved in data/zones/{zone_system_name}/{network_name}/node_zone_info.csv

Column Name | Data Type | Description
-- | -- | --
node_index | int | node id of network
zone_id | int | zone id of zone-system
is_centroid | int | indicates if node is zone centroid (mulitple centroids possible per zone!) 1: is centroid, 0: otherwise

Some zone systems contain various aggregation levels. In this case, there are following optional column names
Column Name | Data Type | Description
-- | -- | --
zone_agg 1 | int | zone id of aggregated zone-system lvl 1
zone_agg 2 | int | zone id of aggregated zone-system lvl 2
... | int | ...


### Edge - Zone Relation (optional)

* definition of exit_zone_id allows a quick filter for all Cordon sections of a certain area
* saved in data/zones/{zone_system_name}/{network_name}/edge_zone_info.csv

Column Name | Data Type | Description
-- | -- | --
from_node | int | node id of network
to_node | int | node_id of network
zone_id | int | zone id of zone-system of from_node
exit_zone_id | int | -1 if from_node and to_node are in same zone; zone_id of to_node else

### NFD Functions (optional)

* only required for routing/NetworkDynamicNFDClusters.py
* generated with regression: function Flow = Flow(Density)
* two possible definitions: via piecewise linear function or via polynomial fit (polynomial fit is prioritized if both are available)
* data/zones/{zone_system_name}/{network_name}/nfd_coefs_poly.csv -> polynomial fit: generally of degree 3 (but could also be higher or lower degree function)

Column Name | Data Type | Description
-- | -- | --
zone_id | int | zone id of zone-system
a_0 | float | constant coefficient
a_1 | float | linear coefficient
a_2 | float | quadratic coefficient
a_3 | float | cubic coefficient

* data/zones/{zone_system_name}/{network_name}/nfd_reg_{zone_name}.csv -> piecewise linear function definition

Column Name | Data Type | Description
-- | -- | --
Density | float | density value in NFD in veh/km
Flow | float | respective flow value in veh/h

### Cluster Travel Time Functions (Optional)

* only required for routing/NetworkDynamicNFDClusters.py
* functions of form f_c^t = v0 (Density / Flow - v1)
* information saved in file data/zones/{zone_system_name}/{network_name}/cluster_tt_factors.csv

Column Name | Data Type | Description
-- | -- | --
zone_id | int | zone id of zone-system
zone_name | str | optional: zone name
v0 | float | linear coefficient
v1 | float | part of constant coefficient

### Density-Dependent Cluster Toll Functions (Optional)

* only required for routing/NetworkDynamicNFDClusters.py (?)
* dynamic toll based on density within clusters
* information saved in data/zones/{zone_system_name}/{network_name}/toll_model_{cluster_name}.csv

Column Name | Data Type | Description
-- | -- | --
density | float |
toll_coefficient | float |


### Background Traffic Information (Optional)

* file: data/zones/{zone_system_name}/{network_name}/background_traffic.csv
* only requird for routing/NetworkDynamicNFDClusters.py
* contains 'background vehicles' driving in/out/through network clusters per hour

Column Name | Data Type | Description
-- | -- | --
zone_id | int | zone id of zone-system
zone_name | str | optional: zone name
cluster_network_length | float | in km; total lane-specific street length to consider to approximate density (i.e. length of each lane is added!)
bg_density 0-1 | int | in veh/km; average density of concurrent background vehicles in cluster zone_id between 00:00 and 01:00
bg_density 1-2 | int | in veh/km; average density of concurrent background vehicles in cluster zone_id in respective time interval
bg_density 2-3 | int | in veh/km; average density of concurrent background vehicles in cluster zone_id in respective time interval
bg_density 3-4 | int | in veh/km; average density of concurrent background vehicles in cluster zone_id in respective time interval
bg_density 4-5 | int | in veh/km; average density of concurrent background vehicles in cluster zone_id in respective time interval
bg_density 5-6 | int | in veh/km; average density of concurrent background vehicles in cluster zone_id in respective time interval
bg_density 6-7 | int | in veh/km; average density of concurrent background vehicles in cluster zone_id in respective time interval
bg_density 7-8 | int | in veh/km; average density of concurrent background vehicles in cluster zone_id in respective time interval
bg_density 8-9 | int | in veh/km; average density of concurrent background vehicles in cluster zone_id in respective time interval
bg_density 9-10 | int | in veh/km; average density of concurrent background vehicles in cluster zone_id in respective time interval
bg_density 10-11 | int | in veh/km; average density of concurrent background vehicles in cluster zone_id in respective time interval
bg_density 11-12 | int | in veh/km; average density of concurrent background vehicles in cluster zone_id in respective time interval
bg_density 12-13 | int | in veh/km; average density of concurrent background vehicles in cluster zone_id in respective time interval
bg_density 13-14 | int | in veh/km; average density of concurrent background vehicles in cluster zone_id in respective time interval
bg_density 14-15 | int | in veh/km; average density of concurrent background vehicles in cluster zone_id in respective time interval
bg_density 15-16 | int | in veh/km; average density of concurrent background vehicles in cluster zone_id in respective time interval
bg_density 16-17 | int | in veh/km; average density of concurrent background vehicles in cluster zone_id in respective time interval
bg_density 17-18 | int | in veh/km; average density of concurrent background vehicles in cluster zone_id in respective time interval
bg_density 18-19 | int | in veh/km; average density of concurrent background vehicles in cluster zone_id in respective time interval
bg_density 19-20 | int | in veh/km; average density of concurrent background vehicles in cluster zone_id in respective time interval
bg_density 20-21 | int | in veh/km; average density of concurrent background vehicles in cluster zone_id in respective time interval
bg_density 21-22 | int | in veh/km; average density of concurrent background vehicles in cluster zone_id in respective time interval
bg_density 22-23 | int | in veh/km; average density of concurrent background vehicles in cluster zone_id in respective time interval
bg_density 23-24 | int | in veh/km; average density of concurrent background vehicles in cluster zone_id in respective time interval