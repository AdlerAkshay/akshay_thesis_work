# 1. Input Data

The data/ directory will not be part of the git-distribution. Generally, input data should be saved on the local disk and (if allowed) on the server (TUM-VT / Sync&Share).
Beware that confidential data does not become accessible for everybody. 

## Network Data

* {network_name} denotes the title of a network
* there are various routing-modules which are based on different preprocessing scripts; the preprocessed data are also saved in the respective network directory
* the specification of all network csv and geojson files is given in the respective files under documentation/data_specification/*.md
* each network has to have following mandatory directory and file structure:
<br/>networks/
<br/>networks/{network_name}/
<br/>networks/{network_name}/base/
<br/>networks/{network_name}/base/nodes.csv
<br/>networks/{network_name}/base/edges.csv
<br/>networks/{network_name}/base/nodes_all_infos.geojson
<br/>networks/{network_name}/base/edges_all_infos.geojson
* if the coordinate frame is not WGS84, an additional file states the used reference system
<br/>networks/{network_name}/base/crs.info
* in case network travel times are deterministic, but vary over time, the edge travel times are saved in following structure:
<br/>networks/{network_name}/{scenario_time}/
<br/>networks/{network_name}/{scenario_time}/edges_td_att.csv
* additionally, the NetworkTable routing module requires fastest node-to-node travel time and distance tables for each travel time directory
<br/>networks/{network_name}/ff/
<br/>networks/{network_name}/ff/tables/
<br/>networks/{network_name}/ff/tables/nn_fastest_distance.npy
<br/>networks/{network_name}/ff/tables/nn_fastest_travel_time.npy
<br/>networks/{network_name}/{scenario_time}/tables/
<br/>networks/{network_name}/{scenario_time}/tables/nn_fastest_distance.npy
<br/>networks/{network_name}/{scenario_time}/tables/nn_fastest_travel_time.npy
* network dynamics: this file defines loading of time dependent travel time files or(!) travel time factors (optional)
<br/>networks/{network_name}/{nw_dynamics_f}.csv
* the DynamicNFDTable routing module requires the ff/tables and the network nfd file
<br/>networks/{network_name}/base/nfd.csv

## Spatial Aggregation

* spatial aggregation into zones is necessary for several use cases, e.g. vehicle repositioning, pricing, tolling, NFD clustering
* {zone_system_name} denotes the name of a GIS zone division
* in the {network_name} subdirectory, the GIS data are matched to an existing network
* definition of respective file formats in documentation/data_specification/*.md
* data structure:
<br/> zones/
<br/> zones/{zone_system_name}/
<br/> zones/{zone_system_name}/general_information.csv
<br/> zones/{zone_system_name}/polygon_definition.geojson
<br/> zones/{zone_system_name}/crs.info
<br/> zones/{zone_system_name}/{network_name}/
<br/> zones/{zone_system_name}/{network_name}/node_zone_info.csv
<br/> zones/{zone_system_name}/{network_name}/edge_zone_info.csv

* required for DynamicNFDnetwork: relation between NFD and network travel time factor
<br/> zones/{zone_system_name}/{network_name}/cluster_tt_factors.csv

* required for DynamicNFDnetwork: cluster NFDs either defined by polynomial function coefficients or piecewise linear function (per zone)
<br/> zones/{zone_system_name}/{network_name}/nfd_coefs_poly.csv
<br/> zones/{zone_system_name}/{network_name}/nfd_reg_{zone_name}.csv

* required for DynamicNFDnetwork: background traffic (vehicles driving in/out/through network)
<br/> zones/{zone_system_name}/{network_name}/background_traffic.csv

* required for dynamic toll based on density in zone {zone_name}:
<br/> zones/{zone_system_name}/{network_name}/toll_model_{zone_name}.csv

## Demand Data

* {data_title} should be a name reflecting the data source
* raw data and scripts that reduce them to an unmatched trip format (see specification for unmatched trip data) should also remain on the server for clarity
<br/>data/
<br/>data/demand/
<br/>data/demand/{data_title}/
<br/>data/demand/{data_title}/raw

### Trip/Request/User Data

* the script matching trip data to a given network {network_name} can be found in src/demand/pp/
* see documentation/data_specification/trips.md for a format specification of trips_X.csv (where "X" can be replaced with any title given to the trip file)
* data structure:
<br/>data/demand/{data_title}/matched/
<br/>data/demand/{data_title}/matched/{network_name}/
<br/>data/demand/{data_title}/matched/{network_name}/trips_X.csv

### Demand Forecast Data

* {zone_system_name} refers to the name of a zone-system definition
* {temporal_resolution} refers to the time aggregation given in "hh_mm"
* different forecasts methods are saved as different columns; "trips" refers to a perfect forecast for the given spatio-temporal resolution
* see documentation/data_specification/agg_X.csv for a format specification of agg_X.csv (where "X" can be replaced with any title given to the forecast file)
* data structure:
<br/>data/demand/{data_title}/aggregated/
<br/>data/demand/{data_title}/aggregated/{zone_system_name}/
<br/>data/demand/{data_title}/aggregated/{zone_system_name}/{temporal_resolution}
<br/>data/demand/{data_title}/aggregated/{zone_system_name}/{temporal_resolution}/agg_{X}.csv
<br/>data/demand/{data_title}/aggregated/{zone_system_name}/{temporal_resolution}/agg_od_{X}.csv

## Vehicle Data

* saving vehicle data on the server reduces the time to research for new studies
* specification under documentation/data_specification/vehicle_type.csv
<br/>data/vehicles/
<br/>data/vehicles/EV_type1_20200411.csv
<br/>data/vehicles/EV_type2_20200411.csv

## Fleet Control Data

### Active Fleet Size Data

* can be used for simulations with flexible fleet size, where fleet size is time controlled
* specification under documentation/data_specification/active_vehicles.csv
<br/>data/fleetctrl/elastic_fleet_size/
<br/>data/fleetctrl/elastic_fleet_size/active_vehicle_sample.csv

### Initial Vehicle Distribution

* can be used to specify the vehicle distribution of the initially created vehicles
* specification at documentation/data_specification/init_veh_dist.csv
* {network_name} corresponds to the network the nodes of the init distribution are matched onto
<br/>data/fleetctrl/initial_vehicle_distribution/
<br/>data/fleetctrl/initial_vehicle_distribution/{network_name}/init_veh_dist.csv

### Pricing Data

* can be used to define time dependent elastic pricing or utilization dependent pricing
* {pricing_file} corresponds to the name of the applied pricing_file [possible scenario input]
<br/>data/fleetctrl/elastic_pricing/{pricing_file}.csv
* # TO-DO # 

### Sharing Rate Data

* can be used to scale down forecasts for demanded and incoming vehicles in each zone
* see sharing_rates.md for the specification of the file
* {sharing_rate_file} corresponds to the name of the applied sharing_rate_file [possible scenario input]
<br/>data/fleetctrl/estimated_sharing_rates/{sharing_rate_file}.csv

## Public Transport Data

* preprocessing can be used to reduce query computation time
* data specification in raw follows typical gtfs guidelines
* see documentation/data_specification/stat_to_stat.md for format specification of preprocessed station-to-station data
* {gtfs_title} is the title of the data and should contain the operating area
<br/>data/public_transport/
<br/>data/public_transport/{gtfs_title}


## Infrastructure Data

* infrastructure data can be used to add additional information to certain network nodes, e.g. access points for customers, boarding points, charging stations, depots, parking spaces
* the format of these data files are specified in documentation/data_specification/*.md
* {gis_name} reflects the spatial area of the data and in this directory, all data are referenced by coordinates
* if the data are stored in another reference system then WGS84, a crs.info is saved as well
<br/>data/infra/
<br/>data/infra/{gis_name}
<br/>data/infra/{gis_name}/crs.info
<br/>data/infra/{gis_name}/access_points.geojson
<br/>data/infra/{gis_name}/boarding_points.geojson
<br/>data/infra/{gis_name}/public_charging_stations.geojson
<br/>data/infra/{gis_name}/depots.geojson
* after matching to a network, the respective files are saved in csv file in the {network_name} directory subdirectory
<br/>data/infra/{gis_name}/{network_name}
<br/>data/infra/{gis_name}/{network_name}/access_points.csv
<br/>data/infra/{gis_name}/{network_name}/boarding_points.csv
<br/>data/infra/{gis_name}/{network_name}/public_charging_stations.csv
<br/>data/infra/{gis_name}/{network_name}/depots.csv


# 2. Study Definition Data

* each study consists of multiple steps that are unique for each study and are supposed to be stored in the same
directory.
* all studies are collected in the "studies"-directory
* when creating a new study {study_name}, a new study-folder has to be created within this directory
* the script "create_new_study_directories.py" can be used to create all init directories, described in the following.
<br/>studies/
<br/>studies/{study_name}

## 2.1 Study Specific Preprocessing Files

* preprocessing file that are unique for the specific study should be stored in the folder
<br/>studies/{study_name}/preprocessing

## 2.2 Scenario Definition Data

* a study consists of multiple scenarios; for clarity of scenario definition, simulation parameters are divided in constant parameters (constant within this study) and scenario parameters
* the specification of input parameters (can vary for different simulation environments) in documentation/simulation_input.md
* definition of scenario directory structure for study {study_name}:
<br/>studies/{study_name}/scenarios/
<br/>studies/{study_name}/scenarios/constant_config.csv
<br/>studies/{study_name}/scenarios/scenarios_X.csv

## 2.3 Simulation Output Data

* in order to enable a standard evaluation of simulations, output format should be standardized as well
* the specification of output files can be found in documentation/data_specification/*.md (as output files are numbered, they will appear on top of the specification directory)
* as there can be multiple operators, all operator related output files have a two-piece number code, where the second number is the operator number

### Standard Output

* the directory structure follows the scenario definition structure
<br/>studies/{study_name}/results/
<br/>studies/{study_name}/results/{scenario_name}/
<br/>studies/{study_name}/results/{scenario_name}/0_config.json
<br/>studies/{study_name}/results/{scenario_name}/00_simulation.log
<br/>studies/{study_name}/results/{scenario_name}/1_user_stats_X.csv
<br/>studies/{study_name}/results/{scenario_name}/2-0_op_stats_X.csv

### Environment-Specific Output

* some components, e.g. repositioning, will not be part of every study; if this component is part of the simulations, the output will be stored as well (determine numbering)
<br/>studies/{study_name}/results/{scenario_name}/...

## 2.4 Study Specific Evaluation

* when computing aggregated results from multiple study results, evaluation scripts and files should be stored in the following folder for later clarity and replicability:
<br/>studies/{study_name}/evaluation/
