# 1_user_stats_X.csv

Mandatory attributes with explanation data types.
Additional attributes can be added as long column-names of mandatory attributes are maintained.

Column Name | Data Type | Discription
-- | -- | --
request_id | int | unique identifier for each request
request_time | int | time (s) a requests gets active/visible for the system
origin_node_index | int | index of origin node in network
destination_node_index | int | index of destination node in network
served_by_operator | int | operator_id that served rq
served_by_vehicle | int | vehicle_id that served rq
pick_up_time | int | time of pick up
drop_off_time |int | time of drop off


Possible additional attributes to be discussed:

Column Name | Data Type | Discription
-- | -- | --
intermediate_stop_str | str | for pooling: vehicle stops between start and end
price | float | price rq paid for trip
driven_distance | float | driven distance from start to end
pick_up_node_index | int | might differ from origin_node_index
drop_off_node_index | int | might_differ from destination_node_index


what to do with multiple partial trips (intermodal)