# -------------------------------------------------------------------------------------------------------------------- #
# functions to add development content to dictionaries in init_modules.py


def add_dev_simulation_environments():
    """This function adds simulation environments in the development stage as options. Additionally, some legacy names remain available for a limited amount
    of time.

    :return: dictionary of additional module options
    """
    add_sim_env_dict = {}  # str -> (module path, class name)
    # Development Simulation Environments
    add_sim_env_dict["ExchangeRequests"] = ("dev.BrokerSimulation", "ExchangeRequestsSimulation")
    add_sim_env_dict["BrokerBaseSimulation"] = ("dev.BrokerSimulation", "BrokerBaseSimulation")
    add_sim_env_dict["AimsunControlledFleetsim"] = ("dev.AimsunControlledFleetsim", "AimsunFleetSimulation")
    add_sim_env_dict["SUMOcontrolledSim"] = ("dev.SUMOcontrolledSim", "SUMOcontrolledSim")
    add_sim_env_dict["MoiaFleetSimulation"] = ("dev.MoiaFleetSimulation", "MoiaFleetSimulation")
    add_sim_env_dict["MoiaTestMultiModalFleetsim"] = ("dev.MoiaFleetSimulation", "MoiaTestMultiModalFleetsim")
    add_sim_env_dict["TestMultiStepRideHailingSimulation"] = ("dev.TestMultiStepRideHailingSimulation", "TestMultiStepRideHailingSimulation")
    add_sim_env_dict["CombineIRSpoolingNCMsimulation"] = ("dev.CombineIRSpoolingNCMsimulation", "CombineIRSpoolingNCMsimulation")
    # Legacy (string names)
    add_sim_env_dict["RPtest"] = ("src.BatchOfferSimulation", "BatchOfferSimulation")
    add_sim_env_dict["BMWStudyFleetSimulation"] = ("src.BatchOfferSimulation", "BatchOfferSimulation")
    add_sim_env_dict["VariableBoardingPointsFleetsim"] = ("src.BatchOfferSimulation", "BatchOfferSimulation")
    add_sim_env_dict["RidePoolingImmediateOfferSimulation"] = ("src.ImmediateDecisionsSimulation", "ImmediateDecisionsSimulation")
    add_sim_env_dict["ImmediateOfferSimulation"] = ("src.ImmediateDecisionsSimulation", "ImmediateDecisionsSimulation")
    return add_sim_env_dict


def add_dev_routing_engines():
    """This function adds routing engines in the development stage as options.

    :return: dictionary of additional module options
    """
    add_re_dict = {}  # str -> (module path, class name)
    add_re_dict["NetworkDynamicNFDClusters"] = ("dev.routing.NetworkDynamicNFDClusters", "DynamicNFDNetwork")
    add_re_dict["NetworkAimsunCoupling"] = ("dev.routing.NetworkAimsunCoupling", "NetworkAimsunCoupling")
    add_re_dict["NetworkAimsunCouplingCpp"] = ("dev.routing.NetworkAimsunCouplingCpp", "NetworkAimsunCouplingCpp")
    add_re_dict["NetworkBasicSumoCoupling"] = ("dev.routing.NetworkBasicSumoCoupling", "NetworkBasicSumoCoupling")
    return add_re_dict


def add_request_models():
    """This function adds request/traveler models in the development stage as options. Additionally, some legacy names remain available for a limited amount
    of time.

    :return: dictionary of additional module options
    """
    add_tm_dict = {}  # str -> (module path, class name)
    add_tm_dict["ChoicePtPvAmodInter"] = ("dev.demand.TravelerModels", "ChoicePtPvAmodInter")
    add_tm_dict["BMWStudyRequest"] = ("dev.demand.TravelerModels", "BMWStudyRequest")
    add_tm_dict["IRSStudyRequest"] = ("dev.demand.TravelerModels", "IRSStudyRequest")
    add_tm_dict["ExChangeRequest"] = ("dev.demand.TravelerModels", "ExChangeRequest")
    add_tm_dict["MOIATestRequest"] = ("dev.demand.TravelerModels", "MOIATestRequest")
    add_tm_dict["IndividualConstraintRequest"] = ("src.demand.TravelerModels", "IndividualConstraintRequest")
    add_tm_dict["PriceSensitiveIndividualConstraintRequest"] = ("src.demand.TravelerModels", "PriceSensitiveIndividualConstraintRequest")
    return add_tm_dict


def add_fleet_control_modules():
    """This function adds fleet control models in the development stage as options.

    :return: dictionary of additional module options
    """
    add_op_dict = {}  # str -> (module path, class name)
    add_op_dict["MOIAfleetcontrol"] = ("dev.fleetctrl.MoiaFleetControl", "MOIAfleetcontrol")
    add_op_dict["DoubleAlgorithmFleetcontrol"] = ("dev.fleetctrl.DoubleAlgorithmFleetcontrol", "DoubleAlgorithmFleetcontrol")
    add_op_dict["VariableBoardingPointsFleetcontrol"] = ("dev.fleetctrl.VariableBoardingPointsFleetcontrol", "VariableBoardingPointsFleetcontrol")
    add_op_dict["HailingNNIRSOnly"] = ("dev.fleetctrl.HailingNNOnly", "HailingNNIRSOnly")
    add_op_dict["HailingBatchOptimizationSumObjectivesBase"] = ("dev.fleetctrl.HailingBatchOptimizationSumObjectives", "HailingBatchOptimizationSumObjectivesBase")
    add_op_dict["HailingBatchOptimizationSumObjectivesIRSInformation"] = ("dev.fleetctrl.HailingBatchOptimizationSumObjectives", "HailingBatchOptimizationSumObjectivesIRSInformation")
    add_op_dict["HailingBatchOptimizationSumObjectivesIRSAssignment"] = ("dev.fleetctrl.HailingBatchOptimizationSumObjectives", "HailingBatchOptimizationSumObjectivesIRSAssignment")
    add_op_dict["HailingBatchOptimizationSumObjectivesIRSDecision"] = ("dev.fleetctrl.HailingBatchOptimizationSumObjectives", "HailingBatchOptimizationSumObjectivesIRSDecision")
    add_op_dict["CombineIRSpoolingNCMfleetcontrol"] = ("dev.fleetctrl.CombineIRSpoolingNCMfleetcontrol", "CombineIRSpoolingNCMfleetcontrol")
    add_op_dict["MarvinPoolingFleetControl"] = ("dev.fleetctrl.MarvinPoolingFleetControl", "MarvinPoolingFleetControl")
    add_op_dict["EasyRideBusFltctr"] = ("dev.fleetctrl.EasyRideBusFltctr", "EasyRideBusFltctr")
    add_op_dict["BasePrivateVehiclesController"] = ("dev.fleetctrl.PrivateVehicleControlBase", "BasePrivateVehiclesController")
    add_op_dict["PrivateVehicleWithBookedCharging"] = ("dev.fleetctrl.PrivateVehicleControlBase", "PrivateVehicleWithBookedCharging")
    add_op_dict["LinebasedFleetControl"] = ("dev.fleetctrl.LinebasedFleetControl", "LinebasedFleetControl")
    add_op_dict["RPP_APP_FleetControl"] = ("dev.fleetctrl.RPP_APP_FleetControl", "RPP_APP_FleetControl")
    return add_op_dict


def add_repositioning_modules():
    """This function adds repositioning models in the development stage as options.

    :return: dictionary of additional module options
    """
    add_repo_dict = {}  # str -> (module path, class name)
    add_repo_dict["MOIARepoPavone"] = ("dev.fleetctrl.repositioning.MOIARepoPavone", "MOIARepoPavone")
    add_repo_dict["MOIARepoPavoneRelative"] = ("dev.fleetctrl.repositioning.MOIARepoPavoneRelative", "MOIARepoPavoneRelative")
    add_repo_dict["DensityLWS"] = ("dev.fleetctrl.repositioning.DensityBasedRepositioning", "LinearWeightedSumRepositioning")
    add_repo_dict["DensityLWSRA"] = ("dev.fleetctrl.repositioning.DensityBasedRepositioning", "LWSReducedAvailabilityRepositioning")
    add_repo_dict["DensityLTS"] = ("dev.fleetctrl.repositioning.DensityBasedRepositioning", "LinearTwoStepRepositioning")
    add_repo_dict["DensityQTS"] = ("dev.fleetctrl.repositioning.DensityBasedRepositioning", "QuadraticTwoStepRepositioning")
    add_repo_dict["DensityQDLSTS"] = ("dev.fleetctrl.repositioning.DensityBasedRepositioning", "QuadraticDeficitLinearSurplusTwoStepRepositioning")
    return add_repo_dict


def add_charging_strategy_modules():
    """This function adds charging strategy models in the development stage as options.

    :return: dictionary of additional module options
    """
    add_cs_dict = {}  # str -> (module path, class name)
    add_cs_dict["Threshold_D"] = ("dev.fleetctrl.charging.thresholds", "ChargingThresholdDepot")
    return add_cs_dict


def add_dynamic_pricing_strategy_modules():
    """This function adds dynamic pricing strategy models in the development stage as options.

    :return: dictionary of additional module options
    """
    add_dp_dict = {}  # str -> (module path, class name)
    add_dp_dict["OneStepForecastDP"] = ("dev.fleetctrl.pricing.OneStepForecastDP", "OneStepForecastDP")
    return add_dp_dict


def add_dynamic_fleetsizing_strategy_modules():
    """This function adds dynamic fleet sizing strategy models in the development stage as options.

    :return: dictionary of additional module options
    """
    add_dfs_dict = {}  # str -> (module path, class name)
    return add_dfs_dict


def add_reservation_strategy_modules():
    """This function adds reservation strategy models in the development stage as options.

    :return: dictionary of additional module options
    """
    add_res_dict = {}  # str -> (module path, class name)
    add_res_dict["ForwardBatchOptimization"] = ("dev.fleetctrl.reservation.ForwardBatchOptimization", "ForwardBatchOptimization")
    add_res_dict["InsertionRevelationHorizon"] = ("dev.fleetctrl.reservation.InsertionRevelationHorizon", "InsertionRevelationHorizon")
    add_res_dict["GraphContractionTSP"] = ("dev.fleetctrl.reservation.GraphContractionTSP", "GraphContractionTSP")
    add_res_dict["SpecRRClusterBatchOptimization"] = ("dev.fleetctrl.reservation.SpecRRClusterBatchOptimization", "SpecRRClusterBatchOptimization")
    return add_res_dict


def add_ride_pooling_batch_optimizer_modules():
    """This function adds ride pooling batch optimization models in the development stage as options.

    :return: dictionary of additional module options
    """
    add_rbo_dict = {}  # str -> (module path, class name)
    add_rbo_dict["ParallelTempering"] = ("dev.fleetctrl.pooling.batch.ParallelTempering.ParallelTemperingAssignment", "ParallelTemperingAssignment")
    return add_rbo_dict
