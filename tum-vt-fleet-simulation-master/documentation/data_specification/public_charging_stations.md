# public_charging_stations.csv
Mandatory attributes with explanation of data types.
Additional attributes can be added as long column-names of mandatory attributes are maintained.

Column Name | Data Type | Description
-- | -- | --
charging_station_id | int | unique identifier for each depot
node_index | int | index of node in network (BEWARE: should be unique!)
charging_units | dict_str | power1:number1;power2:number2
public_util | dict_str | hour1:util1;hour2:util2

unit of power : kW
