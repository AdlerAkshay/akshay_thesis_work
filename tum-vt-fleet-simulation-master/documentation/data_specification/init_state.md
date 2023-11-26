# init_state.csv
* vehicle locations and utilization at the end of a simulation period (and possible the start of the next is recorded)
* the vehicles are already positioned to the final vehicle location; if the vehicle remains on the middle of a link, it is positioned at the start node of this link
* the vehicles are simply counted as blocked
* attribute fields:

column_name | data_type | comment
-- | -- | --
operator_id | int |
vehicle_id | int |
final_node_index | int | 
final_time | int | in seconds; remember to calculate modulo 24*3600 to not block the vehicle for the full next day
final_soc | float | 
