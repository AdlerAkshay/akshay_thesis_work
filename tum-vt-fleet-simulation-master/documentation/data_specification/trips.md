# trips_X.csv
Mandatory attributes with explanation data types.
Additional attributes can be added as long column-names of mandatory attributes are maintained.

Column Name | Data Type | Description
-- | -- | --
request_id | int | unique identifier for each request
rq_time | int | time (s) a requests gets active/visible for the system
start | int | index of origin node in network
end | int | index of destination node in network

Additional optional attributes, which can be used to model heterogeneous demand.

Column Name | Data Type | Description
-- | -- | --
latest_arrival_time | int | time a rq must/wants to reach destination; not yet implemented!
rq_type | str/int (TODO) | specifies e.g. the type of service a request wants to use
earliest_pickup_time | int | for reservation
latest_pickup_time | int | latest time for pick-up
latest_decision_time | int | latest time for request to decide for an operator before leaving system
max_rel_detour | int | maximum relative detour in percent
max_fare | int | maximum fare a request is willing to pay (in cent)
global trip destination | int (TODO) | for intermodal trips (mod-destination vs trip-destination)
rq_preferences | (TODO) | other parameters for mode choice
number_passenger | int | passengers within one request
