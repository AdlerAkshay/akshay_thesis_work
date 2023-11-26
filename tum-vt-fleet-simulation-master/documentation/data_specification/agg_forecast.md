# agg_X.csv
Mandatory attributes with explanation of data types.
{fc method} has to be specified in the scenario inputs with the variable G_FC_TYPE

Column Name | Data Type | Description
-- | -- | --
time | int | simulation time in seconds
zone_id | int | index of zone in zone system (BEWARE: should be unique!)
out {fc method} | float | number of outgoing trips for forecast method {fc method}
in {fc method} | float | number of incoming trips for forecast method {fc method}

The following forecasts methods are defined until now:
fc method | Description
-- | -- 
perfect_trips | these "forecasts" are generated from actual aggregation of trip files, thereby making them perfect forecast for the number of trips with respect to the chosen resolution
perfect_pax | these "forecasts" are generated from actual aggregation of trip files; instead of aggregating the number of trips, the number of passengers are aggregated, though


# agg_od_X.csv

Column Name | Data Type | Description
-- | -- | --
time | int | simulation time in seconds
out_zone_id | int | index of zone in zone system (BEWARE: should be unique!)
in_zone_id | int | index of zone in zone system (BEWARE: should be unique!)
{fc_method} | float | number of trips from out_zone_id to in_zone_id for forecast method {fc method}

The following forecasts methods are defined until now:
fc method | Description
-- | -- 
perfect_trips | these "forecasts" are generated from actual aggregation of trip files, thereby making them perfect forecast for the number of trips with respect to the chosen resolution
perfect_pax | these "forecasts" are generated from actual aggregation of trip files; instead of aggregating the number of trips, the number of passengers are aggregated, though
