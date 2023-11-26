# depots.csv
Mandatory attributes with explanation of data types.

Column Name | Data Type | Description
-- | -- | --
charging_station_id | int | unique identifier for each depot
node_index | int | index of node in network (BEWARE: should be unique!)
charging_units | dict_str | power1:number1;power2:number2
max_nr_parking | int | maximum number of vehicles that can park
