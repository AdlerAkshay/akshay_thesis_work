# 2-0_op_stats_X.csv

Mandatory attributes with explanation data types.
Additional attributes can be added as long column-names of mandatory attributes are maintained.

Column Name | Data Type | Discription
-- | -- | --
operator id | int | operator id of vehicle
vehicle id | int | identifier of vehicle of operator
vehicle entry id | int | counter of entries of corresonding vehicle in stat file
task id | int | index of vehicle task (0 : inactive (no current task), 1: onroute, 2: boarding, 3: waiting, 4: repositioning, 5: charging, ... (TODO,TBD))
start location | str | network position str of start location
end location | str | network position str of end location
start time | int | start time of task
end time | int | end time of task
driven distance | float | driven distance during task
driven_route_str | str | network route str of driven route
n_ob | int | number of customer onboard during task
ob_requests | str | list of rq-ids seperated by ; currently on board
boarding_requests | str | list of rq-ids seperated by ; currently boarding
deboarding_requests | str | list of rq-ids seperated by ; currently deboarding

optional / to be discussed TODO

Column Name | Data Type | Discription
-- | -- | --
route_time_stamps | str | list-string of times nodes in driven routes are reached (e.g. for stochastic/dynamic travel times)
charge_state_start | float | level of charge at task start (0.0 - 1.0)
charge_state_end | float | level of charge at task end (0.0 - 1.0)
currently_assigned_request | str | list-str of request-ids assigned to be served (at start or end?)
...|  |