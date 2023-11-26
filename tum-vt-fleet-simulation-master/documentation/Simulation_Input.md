# General Approach
* TODO: bring in same format as module input parameters (should already be the correct in most cases)
* Scenarios are set up in scenarios/{study_name}/ directory
* General approach:
** simulation modules test in their init phase whether the input is complete
** single parameter inputs are handled via the config.csv & scenarios.csv and they refer to additional input data files where necessary
* Division into config.csv and scenarios.csv
** config.csv contains parameters that are constant throughout the study
** scenarios.csv contain parameters that are varied in the study
* The format of additional input data files is specified in more detail in the respective files in the documentation/data_specification/ directory
* In order to ensure the correct format of input/output data columns according to the specification, the string-variables representing the column names are treated as global variables in src/misc/globals.py
* Input format of more complex structures:
** list: ";" separated inputs. Example: "12;15;8"
** dict: ":" separation of key and value, ";" separation of key, value tuples. Example: "v1:12;v2:48"
* all inputs (strings, values of list, keys and values of dict) run through following conversion process:
1. float conversion
2. int conversion of floats with |x - int(x)| < EPS (EPS = 0.0001)
3. bool conversion of strings "True" and "False"
4. None conversion of string "None"


# General Scenario Input Parameters

## Mandatory General Scenario Input Parameters

Parameter Name | Type | Comment
-- | -- | --
sim_env | str | see section [Simulation Environments](#-simulation-environments) for choice strings and short descriptions
study_name | str | upper level output directory
scenario_name | str | lower level output directory name
initial_state_scenario | str | lower level output directory name of scenario that determines the initial state; random positions will be generated if that does not exist
network_type | str | see section [Network Modules](#-network-modules) for choice strings and short descriptions
network_name | str | name of network data set
demand_name | str | name of demand data set (requires a match-subdirectory for network_name for simulation)
rq_file | str | name of traveler data set
random_seed | int | random seed for np.random and random modules
replay_flag | bool | activates the output of "node1:time1;node2:time2" output for replay visualization of simulation
route_output_flag | bool | True: record path as list of nodes
realtime_plot | int | Options for python based real-time visualization for each time step of the simulation. <ul><li>**0**: (default) No real-time plot.</li><li>**1**: Uses a separate CPU process to plot in a new window (Should not be used while running multiple simulations in parallel). </li><li>**2**: Silently saves each plot as images in results folder. It uses the same CPU process as of the main simulation.  </li></ul> 
realtime_plot_veh_states | int | Only the vehicles with the listed status will be displayed in the real-time plots. The integer vehicle status codes are found in the file "src\misc\globals.py" and can be listed using semicolons, e.g. "0;1;10".
start_time | int | start time in seconds
end_time | int | end time in seconds
time_step | int | time step in seconds
nr_operators | int | number of operators

## Simulation Environment-Specific / Optional Scenario Input Parameters
* one of the request type formulations has to be used
* random vehicle distribution in case initial_state_scenario is not set

Parameter Name | Type | Comment
-- | -- | --
rq_type | | One user-type for the whole simulation; some environments have fixed type
rq_type_distribution | | One user-type distribution over the complete study area
rq_type_od_distribution | | Different user-type distributions per zone
initial_state_scenario | |
zone_system_name | str | basename of zone system directory
zone_correlation_file | str | basename of zone-to-zone reachability correlation file {ZCF}.npz; squared correlation file has to be named {ZCF}_squared.npz
forecast_f | str | name of forecast file
fc_type | str | name of forecast method; see data_specification/agg_forecast.md for more details
park_cost_scale | float | determines overall scaling of zone-dependent park costs (at the moment: total park cost per trip)
toll_cost_scale | float | determines overall scaling of zone-dependent toll costs (in cent per meter)
fc_type | | 
gtfs_name | | 
co2_per_kWh | float | for computation of emissions based on electric propulsion
walking_speed | float | walking speed in m/s
min_IM_MOD_distance | float | minimum distance in meters between origin/destination and next (rail-based) PT station to be considered for first/last mile trip
eval_share_daily_fix_cost | float | in [0,1]; if only part of a day is simulated, the AMoD vehicle fix costs should be reduced to a certain share
nw_tt_factor_f | str | name of csv file containing the scaling factors per travel time
infra_allow_street_parking | bool | default: True; if False, idle vehicles are sent to the nearest depots

# Simulation Environments
* Simulation environments are loaded via the scenario parameter "sim_env". A specific simulation environment is loaded with a certain string assigned for the environment.
* The description can either be written in the field or reference to a paper.

Python Module | sim_env | Short Description
-- | -- | --
src.SequentialModalChoiceDynamicNetwork | SMCDNS | ISTTT 2020, Florian Dandl

# Network Modules
* module for network definition and routing calls
* network_type determines the module which is used for a simulation
* description either as text or reference to paper

Python Module | network_type | Short Description
-- | -- | --
src.routing.NetworkDynamicNFDClusters | DynamicNFDNetwork | ISTTT 2020, Florian Dandl

# Public Transport Modules
* GTFS-based and Matrix-based PT modules
* Matrix-based PT modules allow change of frequencies

Python Module | sim_env | Short Description
-- | -- | --

Parameter Name | Simulation Modules | Comment
-- | -- | --
pt_base_fare | all | base fare for a PT trip in cent
pt_cost_per_km | all | operating costs of PT sytem in cent/km
pt_freq_scale | *Matrix* | scale factor to change PT frequency
pt_freq_scale_hours | *Matrix* | list of hours of day in which scale factor is applied (separated by ";")


# Traveler Classes
* ...

Class | rq_type | Short Description
-- | -- | --
ChoicePtPvAmodInter | ChoicePtPvAmodInter | chooses between private vehicle, public transportation, AMoD and intermodal AMoD options, see ISTTT for further details

## Traveler Constraints
* These parameters can be used for Travelers that choose betwen using or rejecting AMoD.

Parameter Name | Comment | used in Traveler Classes
-- | -- | --
user_min_wait_time | in seconds after request time | OpenConstraintRequest
user_max_wait_time | in seconds after request time | OpenConstraintRequest
user_max_constant_detour_time | in seconds | OpenConstraintRequest
user_max_detour_time_factor | in %; relative detour factor | OpenConstraintRequest
user_price_sensitivity_func_dict | function definition per dictionary | 

* These parameters can be used for Travelers that choose between modes.
Parameter Name | Comment | used in Traveler Classes
-- | -- | --
value_of_time | in cent per second | ChoicePtPvAmodInter
pt_transfer_penalty | in cent per transfer | ChoicePtPvAmodInter
private_vehicle_full_costs_per_m | in cent | ChoicePtPvAmodInter
private_vehicle_mode_choice_intercept | in cent; can be used for calibration | ChoicePtPvAmodInter


## Mode-Choice Specific Parameters
* Depending on the traveler model, various coefficients are necessary to describe the utility of different mobility options

Parameter Name | Type | unit | Comment
-- | -- | -- | --
value_of_time | float | cent per second | coefficient used to describe the cost for spending time waiting, in-vehicle, ...
private_vehicle_mode_choice_intercept | float | cent | value to describe the comfort associated with using a private vehicle; used for calibration
private_vehicle_full_costs_per_km | float | cent per meter | coefficient to describe the full costs associated with driving a private vehicle
pt_transfer_penalty | float | cent | coefficient to penalize a public transport transfer


# Operator Modules
* Operator modules provide all databases and interfaces between the high-level simulation flow and lowest-level strategy-functions (which can be shared among operator modules), thereby ensuring a valid and constistent combination of sub-strategies for e.g. request response, customer assignment, fleet update, repositioning, charging, ...
* The idea is that not all studies have to integrate all sub-strategies and take care of all attributes and processes that are related to these.
* Input format:
** all operator attributes have to start with "op_"
** if all operators use the same attribute, a single string is sufficient and will be applied for each operator
** for attributes different among operators, a "|" separated string has to be used to represent the list; the number of list entries has to be nr_operators
   -> WARNING: THIS IS NOT THE WAY IT IS IMPLEMENTED CURRENTLY <-

An operator attribute dictionary (for each operator) contains the "Parameter Name" as key and the respective value. Since the scenario definition should be saved in a csv file, where each row describes a scenario, only string-like elements are available.
# Mandatory Operator Scenario Input Attributes (for all simulation modules)

Parameter Name | Comment
-- | --
op_module | see section [Operator Modules](#-operator-modules) for choice strings and short descriptions
op_fleet_composition | string of format: "veh_type1:number1;veh_type2:number2"

Operator Module | operator_module_str | Comment
-- | -- | --
src.fleetctrl.PoolingInsertionOnlyWithDynPricing | PoolingInsertionOnlyWithPricing | ISTTT 2020, Florian Dandl

# Optional Operator Strategy Method Activation
Following parameters can be set in order to run simulations with specific strategies. They might require further hyperparameters that have to be set with the scenario input!
If they are not set, the operator will use no strategy. An error is thrown if an unknown string is entered.

Operator Module | operator_module_str | Comment
-- | -- | --
Charging Module | op_charging_method | see ... for list of options 
Repositioning Module | op_repo_method | see ... for list of options 
Dynamic Pricing Module | op_dyn_pricing_method | see ... for list of options 
Dynamic Fleetsizing Module | op_dyn_fs_method | see ... for list of options


# Optional Operator Scenario Input Attributes (for all simulation modules)
* direct route travel time includes boarding process
* refer to Roman's pictures
* the parameters should be sorted by module/topic

Parameter Name | Default Value | Comment
-- | -- | --
op_init_veh_distribution | None | file basename in directory \data\fleetctrl\initial_vehicle_distribution\{nw_name}\; file format: see data specification
op_vr_control_func_dict | | this input sets the control function for customer assignment; dictionary with required entry "func_key: str" and possible additional entries for further parameters; check out respective src.fleetctrl.X.objectives
op_min_wait_time | 0 | in seconds
op_max_wait_time | 100000 | in seconds
op_max_detour_time_factor | None | in %; relative to direct route travel time & boarding time
op_add_constant_detour_time | None | in seconds
op_max_constant_detour_time | None | in seconds
op_min_detour_time_window | None | in seconds
op_const_boarding_time | 0 | in seconds
op_add_boarding_time | 0 | in seconds; additional boarding time per boarding/alighting person
op_base_fare | 0 | cent
op_distance_fare | 0 | cent per m
op_time_fare | 0 | cent per second
op_min_standard_fare | 0 | cent
op_util_surge_price_scale_dict | 0:1.0;1:1.0 | piecewise linear function definition of surge pricing factor; utilization:price
op_zone_price_scale_dict | | pre-defined price factor per zone; zone_id: scale-factor
op_depot_ownership | None | for example "0", "0;2", or "0|1;2|0;3" (the ints represents charging_station_ids); None means that all operators own all depots
op_min_charging_time | 900 | minimum time that a charging unit schedule should be free in order to be considered for assignment
op_repo_method | | repositioning method
op_repo_timestep | | time step of repositioning algorithms
op_supply_fc_type | "default" | "default": based on current state and vehicle tasks + departure forecast; "idle_plus_trip_fc": based on currently idle + arrival and departure forecasts
op_repo_horizons | | list string can be given; "0;900" means that the forecast starts from sim_time and goes to sim_time + 900; algorithms with more than 2 values possible
op_repo_lock | | flag whether repositioning tasks should be locked
op_repo_add_infos | | dictionary-string that can contain additional parameters of repositioning algorithms
op_repo_discount_factor | | discount factor for multi-step repositioning approaches
op_repo_exp_trip_profit | | expected profit from additional user trip
op_repo_sharing_rate_file | | sharing rate file that can be used to scale down expected demanded and incoming vehicles
op_repo_quadratic_benefit_threshold | | number of vehicle surplus/deficit from which on a quadratic objective term should have larger impact than a linear objective term
op_dyn_pricing_func_dict | | expected demand curve; dictionary with required entry "func_key: str" and possible additional entries for further parameters
op_dyn_pricing_max_factor | | maximum surge pricing factor (used in OneStepForecast)
op_dyn_pricing_log_par_a | | parameter describing expected logistic demand sensitivity  (see OneStepForecast)
op_dyn_pricing_log_par_b | | parameter describing expected logistic demand sensitivity  (see OneStepForecast)
op_short_term_horizon | 900 | in seconds; requests with earliest pick-up time more than this time in the future are regarded as reservation requests
op_max_VR_con | | number of vr connections an operator should allow (heuristic for customer vehicle assignment)
op_max_request_plans | | number of vehicle plans that are returned from insertion_with_heuristics
op_rh_immediate_max_routes | | number of routes that are returned from vehicle search in immediate_insertion_with_heuristics (not implemented for AlonsoMora)
op_rh_reservation_max_routes | | number of routes that are returned from vehicle search in reservation_insertion_with_heuristics (not implemented for AlonsoMora)
op_rvh_nr_direction | | number of vehicles that are selected by RV heuristic looking for directionality in insertion_with_heuristics
op_batch_rvh_nr_direction | | same as above, but this parameter allows for another value in the batching process; falls back on op_rvh_nr_direction if not set
op_rvh_nr_least_load | | number of vehicles that are selected by RV heuristic based on least load in insertion_with_heuristics
op_batch_rvh_nr_least_load | | same as above, but this parameter allows for another value in the batching process; falls back on op_rvh_nr_least_load if not set
op_rvh_AM_nr_check_assigned_rrs | | number of vehicles that are selected by RV heuristic based on RR graph (only valid for fleetcontrols based on AlonsoMora)
op_rvh_AM_nr_test_best_insertions | | number of vehicles that are selected by RV heuristic based on cfv of tested insertions (only valid for fleetcontrols based on AlonsoMora)
op_vpi_nr_plans | | number of vehicle plans of a single vehicle that are kept in process of insertion_with_heuristics (not implemented for AlonsoMora)


## Control-Specific Operator Scenario Input Attributes for vr_control_func_dict (pooling)
* used to switch between different control functions for pooling

func_key | required additional keys | Comment
-- | -- | --
total_distance | | This function evaluates the driven distance according to a vehicle plan.
total_system_time | | This function evaluates the total spent time of a vehicle according to a vehicle plan.
distance_and_user_times | vot | This function combines the total driving costs and the value of customer time; value of time ("vot") has to be given in cent per second.
