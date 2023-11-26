# Line- & Schedule-Based Public Transportation Service

## GTFS Data Format
* modules working with static version of GTFS data
* see https://developers.google.com/transit/gtfs/reference for more information
* GTFS data for Germany available under https://www.gtfs.de 
* Transfer.txt has to be generated -> TODO: create Script | intermediary solution: work with OTP and create Travel Time Matrices

## Additional Information
* coordinate reference system: gtfs_dir/crs.info file with one line:
epsg:EPSG_CODE


* station definition file: gtfs_dir/stations.csv

Column Name | Data Type | Description
-- | -- | --
station_id | int | station definition
stop_ids | str (representing list[int]) | separated by ";"; from gtfs 'stops.txt' -> stop_id
route_ids | str (representing list[int]) | separated by ";"; from gtfs 'routes.txt' -> route_id
rail_based | bool | True if it includes metro, ubahn, sbahn, tram, ... line ; False if only bus lines are connected
coordinate_1 | float | x-coordinate (West-East for UTM; lon for GPS)
coordinate_2 | float | y-coordinate (South-North for UTM; lat for GPS)

* additional route information regarding capacity (required for crowding): gtfs_dir/add_route_information.csv

Column Name | Data Type | Description
-- | -- | --
route_id | int | from gtfs 'routes.txt'
route_long_name | str | from gtfs 'routes.txt' or own names
rail_based | bool | True if metro, ubahn, sbahn, tram; False if bus
route_length | float | length of route in km; used to compute costs and emissions of PT
tt | float | trip duration within study area in minutes
energy_per_trip | float | energy in kWh of a single trip
trip_capacity | int | Number of passengers that can be transported by one public transport vehicle at the same time
nr_trips 0-1 | int | number of trips starting at first stop between 00:00 and 01:00
nr_trips 1-2 | int | number of trips starting at first stop between respective time interval
nr_trips 2-3 | int | number of trips starting at first stop between respective time interval
nr_trips 3-4 | int | number of trips starting at first stop between respective time interval
nr_trips 4-5 | int | number of trips starting at first stop between respective time interval
nr_trips 5-6 | int | number of trips starting at first stop between respective time interval
nr_trips 6-7 | int | number of trips starting at first stop between respective time interval
nr_trips 7-8 | int | number of trips starting at first stop between respective time interval
nr_trips 8-9 | int | number of trips starting at first stop between respective time interval
nr_trips 9-10 | int | number of trips starting at first stop between respective time interval
nr_trips 10-11 | int | number of trips starting at first stop between respective time interval
nr_trips 12-13 | int | number of trips starting at first stop between respective time interval
nr_trips 13-14 | int | number of trips starting at first stop between respective time interval
nr_trips 14-15 | int | number of trips starting at first stop between respective time interval
nr_trips 15-16 | int | number of trips starting at first stop between respective time interval
nr_trips 16-17 | int | number of trips starting at first stop between respective time interval
nr_trips 17-18 | int | number of trips starting at first stop between respective time interval
nr_trips 18-19 | int | number of trips starting at first stop between respective time interval
nr_trips 19-20 | int | number of trips starting at first stop between respective time interval
nr_trips 20-21 | int | number of trips starting at first stop between respective time interval
nr_trips 21-22 | int | number of trips starting at first stop between respective time interval
nr_trips 22-23 | int | number of trips starting at first stop between respective time interval
nr_trips 23-24 | int | number of trips starting at first stop between respective time interval


* matched street network node id (network-specific): gtfs_dir/{network_name}/station_nodes.csv

Column Name | Data Type | Description
-- | -- | --
station_id | int | from station definition file
network_node_id | int | node from the street network
transfer_link_time | float | possible transfer time from street network node to station; can be assumed 0 in most cases


* matched nearest stations of stop-only street network nodes (network-specific): gtfs_dir/{network_name}/stop_only_stations.csv

Column Name | Data Type | Description
-- | -- | --
network_stop_id | int | node_index of stop_only node from the street network
nearest_station_id | int | from station definition file
nearest_rail_station_id | int | from rail station definition file with rail_based == True


## Preprocessed Information
* queries from random places can always be reduced to queries between stations
* various methods to preprocess network to increase performance of queries between stations (described below)


### Travel Time Matrix
* used for PtTTMatrix-modules
* can be generated any way, also via OpenTripPlanner
* generation of station-to-station information
* data file gtfs_dir/traveltime_matrix.csv

Column Name | Data Type | Description
-- | -- | --
origin | int | station_id
destination | int | station_id
walk_distance | float | in meters; only access and egress?
travel_time | float | in seconds; also includes transfer times etc.
number_transfers | int | number of transfers on the station-to-station route


### Storing a Number of Routes
* Save up to X (3?) routes in format line-station,line-station per station-to-station pair
* Query: Check stop-times for these X routes
* TODO * after ISTTT | could also be interdisciplinary project for information? -> test several ways of preprocessing: quality of solution (fastest route with least transfers?) vs computation time
* check out section 4 in https://i11www.iti.kit.edu/extra/publications/bdgmpsww-rptn-16.pdf

### LinebasedFleetControl
the LinebasedFleetControl module models public transport vehicles explicitly as fleet vehicles and therefore needs additional input to specify the schedules
following files hav to be specified:
* a station definition file stored at data/pubtrans/{pt_name}/{nw_name}/stations.csv with similar entries as specified above:

Column Name | Data Type | Description
-- | -- | --
station_id | int | id of the station
network_node_index | int | index of the corresponding network node

* a schedule file stored at data/pubtrans/{pt_name}/schedules.csv with entries:

Column Name | Data Type | Description
-- | -- | --
station_id | int | station id of a stop
departure | int | earliest time of departure at the stop
LINE | int/str | line specifier corresponding to this stop
vehicle_type | str | vehicle type serving this stop
trip_id | int | identifier of a trip (stops with same trip_id will be served one after another)
line_vehicle_id | int | vehicle id serving this line (it has to be guaranteed that the vehicle can serve corresponding trips)
