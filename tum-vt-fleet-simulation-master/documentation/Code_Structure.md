# Code Structure
* To-Do: how to handle the integration and documentation of new moduls?
* To-Do? Bring content into table format:

Path | Description
-- | --
src/egrep_todo.py | script to search for "# to-do # ..." in src/ directory

## Main Directory
* scripts to start multiple simulations/visualization/upper level programs
* standard: run_scenarios_from_csv.py
<br/>run_scenarios_from_csv.py
<br/>run_simulation_with_visualization.py
<br/>visualize_replay_from_results.py
<br/>ISTTT_bayesian_optimization.py

## Fleet Simulation Code Directory
* shared via gitlab
* new projects > branches, can be merged back when succesfully implemented
<br/>src/

## Shared Miscellaneous Contents
* contains global variable definitions (for input and output) to enable quick overview of string definitions and consistency
* mathematical functions
<br/>src/misc
<br/>src/misc/globals.py
<br/>src/misc/distributions.py

## Simulation Environments
* see Simulation_Environment.md for further information
* control the simulation flow and call shared moduls, e.g.
<br/>src/FleetSimulationPattern.py
<br/>src/SequentialUserDecisions.py
<br/>src/BatchUserDecisions.py
<br/>src/BatchOperationDecisions.py
<br/>src/DynamicSequentialUserDecisions.py
<br/>src/MobiToppSlaveSimulation.py
<br/>src/AimsunSimulation.py

## Fleet Control
* contains current vehicle plans
* calls submodules to compute new plans
<br/>src/fleetctrl/
<br/>src/fleetctrl/MobiTopp.py
<br/>src/fleetctrl/EasyRide.py

### Customer Assignment
* different data-base and solution approaches for hailing / pooling operation
* differentiation between immediate (request-triggered) action and batch actions
<br/>src/fleetctrl/hailing/
<br/>src/fleetctrl/hailing/immediate/
<br/>src/fleetctrl/hailing/immediate/nearest_neighbor.py
<br/>src/fleetctrl/hailing/batch/
<br/>src/fleetctrl/hailing/batch/global_optimization.py
<br/>src/fleetctrl/pooling/
<br/>src/fleetctrl/pooling/immediate/
<br/>src/fleetctrl/pooling/immediate/insertion.py
<br/>src/fleetctrl/pooling/batch/
<br/>src/fleetctrl/pooling/batch/request_build.py
<br/>src/fleetctrl/pooling/batch/tree_build.py
<br/>src/fleetctrl/pooling/batch/optimize_v_rb.py

### Other Fleet Assignments
* special strategies for e.g. repositioning, charging, pricing
<br/>src/fleetctrl/repos/
<br/>src/fleetctrl/charging/
<br/>src/fleetctrl/pricing/

## Network and Routing Code
* the routing modules for simulations are saved in the main routing directory
* module-shared, extractor and preprocessing scripts are in subdirectories
<br/>src/routing/
<br/>src/routing/NetworkModuleDesign.py
<br/>src/routing/NetworkBasic.py
<br/>src/routing/imports/
<br/>src/routing/imports/Router.py
<br/>src/routing/imports/PriorityQueue_python3.py
<br/>src/routing/extractors/
<br/>src/routing/extractors/aimsun_export_converter.py
<br/>src/routing/extractors/aimsn_export_network.py
<br/>src/routing/extractors/visum_converter.py 
<br/>src/routing/extractors/osm.py
<br/>src/routing/preprocessing/
<br/>src/routing/preprocessing/reduce_network.py
<br/>src/routing/preprocessing/pp_contraction_hierarchy.py
<br/>src/routing/preprocessing/pp_table.py
<br/>src/routing/preprocessing/add_stop_nodes_and_edges.py
<br/>src/routing/preprocessing/remove_node.py

## Visualization Module
* this module is used for run_simulation_with_visualization and visualize_replay_from_results
* the module is responsible for the socket communication
<br/>src/visual/
<br/>src/visual//VisualizeFleet.py
<br/>src/visual//static/
<br/>src/visual//templates/

## Public Transport Module
* allows queries to a public transport GTFS feed
* returns travel information by Public transport
<br/>src/pubtrans/
<br/>src/pubtrans/PtModuleDesign.py
<br/>src/pubtrans/PtScheduleQuery.py
<br/>src/pubtrans/PtCrowding.py

## Demand Module
* the traveler module contains different models for travelers (no constraints, max wait time, max detout time, mode choice, ...)
* the demand forecast module can be used to interpolate forecast data if they are not available
* the subdirectories contain scripts to preprocess demand data
<br/>src/demand/
<br/>src/demand/TravelerModels.py
<br/>src/demand/DemandForecast.py
<br/>src/demand/projections/
<br/>src/demand/projections/create_uniform_demand.py
<br/>src/demand/projections/project_data_to_network.py
<br/>src/demand/projections/scale_demand.py
<br/>src/demand/forecasting/
<br/>src/demand/forecasting/aggregate_trip_data.py
<br/>src/demand/forecasting/create_dow_pod_forecast.py
<br/>src/demand/forecasting/create_online_forecasts.py

## Infrastructure-Related Moduls
* these moduls should provide consistent models for simulations environments
<br/>src/infra/
<br/>src/infra/Zoning.py
<br/>src/infra/Parking.py
<br/>src/infra/Charging.py

# Evaluation
* the evaluation should be split into general and case-specific evaluations
* as output is standardized, evaluation scripts can be transfered between projects
<br/>src/evaluation/
<br/>src/evaluation/general/
<br/>src/evaluation/hailing/
<br/>src/evaluation/pooling/
<br/>src/evaluation/charging/
