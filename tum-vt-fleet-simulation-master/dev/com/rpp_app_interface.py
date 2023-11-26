import sys
import os
import socket
from time import sleep

ENCODE = "utf-8"

TUM_FS_PATH = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
if not TUM_FS_PATH in sys.path:
    sys.path.append(TUM_FS_PATH)

# TODO :
RPP_CONFIG_PATH = os.path.join(TUM_FS_PATH, "studies", "RPP_App")

# REGULAR BEGINN OF SCRIPT

from time import time
import traceback
import datetime
import pandas as pd
import geopandas as gpd
import logging
import json
import rtree
from shapely.geometry import Polygon, Point

from src.misc.globals import *
from src.FleetSimulationBase import FleetSimulationBase
from src.simulation.Offers import TravellerOffer, Rejection
from src.simulation.Vehicles import SimulationVehicle
from src.demand.demand import SlaveDemand
import src.misc.config as config
from src.misc.init_modules import load_fleet_control_module
from dev.fleetctrl.RPP_APP_FleetControl import RPP_APP_FleetControl

LOG = logging.getLogger(__name__)

RECORD_COM = True

PARCEL_SIZE_STR_TO_INT = {  # TODO specify ints !!
    "TINY": 1,
    "SMALL": 2,
    "MEDIUM": 4,
    "LARGE": 8
}

REQUEST_STATES = [
    "NONE",  # An error state used ONLY inside the Back-End
    "PENDING",  # The request was created and the Fleet Control is calculating if it can be fulfilled
    "ACCEPTED",  # The request was accepted by the Fleet Control
    "DECLINED",  # The request was declined by the Fleet Control
    "COMING",  # The driver is currently driving towards the pickup location
    "WAITING",  # The driver is at the pickup location and wating for the customer
    "ACTIVE",  # The request is currently being fulfilled
    "COMPLETED",  # The request was fulfilled
    "CANCELLED"  # The request was cancelled by the user
]

VEHICLE_TYPE_TO_SIM_VEH_TYPE = {
    # "RICKSHAW_SEATS_2_TRUNK_NONE": 0,
    # "RICKSHAW_SEATS_1_TRUNK_MEDIUM": 1,
    # "RICKSHAW_SEATS_0_TRUNK_LARGE": 2,
    "TUM_RICKSHAW" : "tum_rickshaw",
    "LEDERHOSEN_RICKSHAW" : "lederhosen_rickshaw",
    "LASTEN_RAD" : "lastenrad"
}

TIME_FORMAT_STR = '%Y-%m-%d %H:%M:%S'
REFERENCE_TIME = datetime.datetime(1992, 6, 6)

BACKEND_EPSG = 4326


def string_time_to_abs_int(time_str):
    """ this function converts the time format used in the backend
        (2021-04-28 14:45:16.43)
        to an absolute int in seconds 
    :param time_str: string of current time
    :return: int of absolute time in seconds """
    time_strct = datetime.datetime.strptime(time_str, TIME_FORMAT_STR)
    seconds = (time_strct - REFERENCE_TIME).total_seconds()
    return seconds


def log_message(msg):
    print("Debug: " + msg)


class RppAppSimulationVehicle(SimulationVehicle):
    """ this class is used to keep track of the vehicles with information coming from the backend and retrieve
    new routing tasks for the drivers 
    TODO maybe its usefull to perform map match in this class to include information of the planned route
    """

    def __init__(self, operator_id, vehicle_id, vehicle_data_dir, vehicle_type, routing_engine, rq_db, op_output,
                 record_route_flag, replay_flag):
        super().__init__(operator_id, vehicle_id, vehicle_data_dir, vehicle_type, routing_engine, rq_db, op_output,
                         record_route_flag, replay_flag)
        self.new_task_available = False  # indicates if new information has to be sent to the backend
        LOG.debug("vehicle init {}".format(self))

    def assign_vehicle_plan(self, list_route_legs, sim_time, force_ignore_lock=False):
        """This method enables the fleet control modules to assign a plan to the simulation vehicles. It ends the
        previously assigned leg and starts the new one if necessary.
        :param list_route_legs: list of legs to assign to vehicle
        :type list_route_legs: list of VehicleRouteLeg
        :param sim_time: current simulation time
        :type sim_time: int
        :param force_ignore_lock: this parameter allows overwriting locked VehicleRouteLegs, except a boarding process allready started
        :type force_ignore_lock: bool
        """
        self.new_task_available = True
        LOG.debug("new vehicle plan assigned {}".format([str(x) for x in list_route_legs]))
        # transform rq from PlanRequest to SimulationRequest (based on RequestBase)
        # LOG.info(f"Vehicle {self.vid} before new assignment: {[str(x) for x in self.assigned_route]} at time {sim_time}")
        for vrl in list_route_legs:
            boarding_list = [self.rq_db[prq.get_rid()] for prq in vrl.rq_dict.get(1,[])]
            alighting_list = [self.rq_db[prq.get_rid()] for prq in vrl.rq_dict.get(-1,[])]
            vrl.rq_dict = {1:boarding_list, -1:alighting_list}
        #LOG.debug(f"Vehicle {self.vid} received new VRLs {[str(x) for x in list_route_legs]} at time {sim_time}")
        #LOG.debug(f"  -> current assignment: {self.assigned_route}")
        start_flag = True
        if self.assigned_route:
            if not list_route_legs or list_route_legs[0] != self.assigned_route[0]:
                if not self.assigned_route[0].locked:
                    self.end_current_leg(sim_time)
                else:
                    LOG.warning("assign_vehicle_plan(): Trying to assign new VRLs instead of a locked VRL.")
                    LOG.warning("vid : {}".format(self.vid))
                    LOG.warning("currently assigned: {}".format([str(x) for x in self.assigned_route]))
                    LOG.warning("new: {}".format([str(x) for x in list_route_legs]))
                    end_leg = True
                    if len(self.assigned_route) > 0:
                        LOG.warning("current additional infos: {}".format(self.assigned_route[0].additional_str_infos()))
                        if self.assigned_route[0].status == VRL_STATES.BOARDING:
                            LOG.warning(f" -> even currently boarding!")
                            if list_route_legs[0].status in G_DRIVING_STATUS:
                                LOG.warning(f" -> remove first moving task: {list_route_legs[0]}")
                                list_route_legs = list_route_legs[1:]
                            end_leg = False
                    if len(list_route_legs) > 0:
                        LOG.warning("new additional infos: {}".format(list_route_legs[0].additional_str_infos()))
                    if end_leg:
                        self.end_current_leg(sim_time)
            else:
                start_flag = False

        self.assigned_route = list_route_legs
        if list_route_legs:
            if start_flag:
                self.start_next_leg_first = True
        
        if self.start_next_leg_first:
            self.start_next_leg(sim_time)

    def update_position(self, position, simulation_time):
        """ this function is used to updated the position of the vehicles
        :param position: new network position
        :param simulation_time: current simulation time
        """
        LOG.debug(f"update position {self.pos} -> {position}")
        if self.status in G_DRIVING_STATUS:
            # TODO store the whole position?
            if len(self.cl_driven_route) == 0 or self.cl_driven_route[-1] != position[0]:
                self.cl_driven_route.append(position[0])
                self.cl_driven_route_times.append(simulation_time)
            self.pos = position
            ca = self.assigned_route[0]
            # TODO this should not be always necessary!
            self.cl_remaining_route = self.routing_engine.return_best_route_1to1(self.pos, ca.destination_pos)
            self.new_task_available = True
        else:
            if position != self.pos:
                LOG.warning("vehicle {} moved in a non moving task? {} <-> {}".format(self.vid, position, self.pos))
                self.pos = position

    def start_boarding(self, position, simulation_time):
        """ this method is triggered from the backend in case a vehicle starts a boarding process
        which needs to set a block that the current boarding process is fixed
        :param position: network position of the boarding process
        :param simulation_time: current simulation time
        :return tuple of list_ended_vrls, list_planned_boarding_pax, list planned alighting pax"""
        # end the current leg and store information
        list_ended_vrls = []
        if len(self.assigned_route) == 0:
            LOG.warning("start boarding without assigned route!")
            return
        LOG.info(f"vid {self.vid} starts boarding with status {self.status} and current leg {self.assigned_route[0]}")
        if self.status != VRL_STATES.BOARDING:
            LOG.info("  -> end leg")
            self.update_position(position, simulation_time)
            ended_vrl = self.end_current_leg(simulation_time)
            list_ended_vrls.append(ended_vrl)
            LOG.info(f"  -> now with status {self.status} and current leg {self.assigned_route[0]}")
        # beginn new leg
        if not self.assigned_route:
            LOG.error(f"no boarding planned for vehicle {self}!")
            return list_ended_vrls, [], []
        else:
            self.start_next_leg(simulation_time)
            ca = self.assigned_route[0]
            while (self.status != VRL_STATES.BOARDING and self.status != VRL_STATES.PLANNED_STOP):  # try to correct according to planned route -> find next boarding stop
                LOG.warning("the next planned leg is no boarding leg!")
                LOG.warning(f"{self}")
                LOG.warning(f"{[x for x in self.assigned_route]}")
                if len(self.assigned_route) == 0:
                    LOG.warning(" -> early break")
                    self.status = VRL_STATES.BOARDING
                else:
                    list_ended_vrls.append(self.end_current_leg(simulation_time))
                    self.start_next_leg(simulation_time)
                    ca = self.assigned_route[0]

            list_boarding_pax = [rq.get_rid_struct() for rq in ca.rq_dict.get(1, [])]
            list_start_alighting_pax = [rq.get_rid_struct() for rq in ca.rq_dict.get(-1, [])]
            for rq_obj in ca.rq_dict.get(1, []):
                self.pax.append(rq_obj)
            ca.locked = True  # always lock boarding until backend triggers end of boarding
            return list_ended_vrls, list_boarding_pax, list_start_alighting_pax

    def end_boarding(self, position, boarded_requests, alighted_requests, cancelled_requests, undeliverable_parcels,
                     simulation_time):
        """ this method is triggerd from the backend after a vehicle finished a boarding task
        :param position: network position of the boarding process
        :param boarded_requests: list of request ids (persons and parcels) that entered the vehicle
        :param alighted_requests: list of request ids (persons and parcels) that alighted the vehicle
        :param cancelled_requests: list of request ids (persons and parcels) that didnt enter the vehicle, but were planned to do so
        :param undeliverable_parcels: list of request ids (should only be parcels) that didn alight the vehicle, but were planned to do so
        :param simulation_time: current simulation time
        """
        LOG.debug(f"end boarding: {position} boarded: {boarded_requests} alighted: {alighted_requests} cancelled: {cancelled_requests} undeliverable: {undeliverable_parcels} time {simulation_time}")
        LOG.debug(f"pax before: {[str(x) for x in self.pax]}")
        if self.status != VRL_STATES.BOARDING:
            LOG.warning("no boarding startet that could end! {}".format([x for x in self.assigned_route]))
        self.update_position(position, simulation_time)
        # TODO treat parcels and persons differently!
        for rid in cancelled_requests:
            try:
                self.pax.remove(self.rq_db[rid])
            except:
                LOG.warning(f"couldnt remove cancelled request {rid} from {self.pax} | {self.assigned_route[0]}")
        list_boarding_pax = []
        list_alighting_pax = []
        for rid in boarded_requests:
            if not self.rq_db[rid] in self.pax:
                self.pax.append(self.rq_db[rid])
            list_boarding_pax.append(self.rq_db[rid])
        for rid in alighted_requests:
            rq_obj = self.rq_db[rid]
            try:
                self.pax.remove(rq_obj)
            except:
                LOG.warning(f"Could not remove passenger {rq_obj.get_rid_struct()} from vehicle {self.vid}"
                            f" at time {simulation_time}")
            list_alighting_pax.append(rq_obj)
        ended_ca = self.end_current_leg(simulation_time, boarded_rq_objs=list_boarding_pax,
                                        deboarded_rq_objs=list_alighting_pax)
        if len(self.assigned_route) > 0:
            self.cl_start_time = simulation_time
            self.cl_start_pos = self.pos
            self.cl_start_soc = self.soc
            self.cl_driven_distance = 0.0
            self.cl_driven_route = []
            ca = self.assigned_route[0]
            self.status = ca.status
        LOG.debug(f"pax after: {[str(x) for x in self.pax]}")
        return ended_ca, list_boarding_pax, list_alighting_pax

    def end_current_leg(self, simulation_time, boarded_rq_objs=[], deboarded_rq_objs=[]):
        ca = self.assigned_route[0]
        LOG.debug(f"Vehicle {self.vid} ends a VRL and starts {ca.__dict__} at time {simulation_time}")
        LOG.debug(f" -> from {self.cl_start_pos} to {self.pos}")
        # record
        record_dict = {}
        record_dict[G_V_OP_ID] = self.op_id
        record_dict[G_V_VID] = self.vid
        record_dict[G_VR_STATUS] = self.status.display_name
        record_dict[G_VR_LOCKED] = ca.locked
        record_dict[G_VR_LEG_START_TIME] = self.cl_start_time
        record_dict[G_VR_LEG_END_TIME] = simulation_time
        if self.cl_start_pos is None:
            LOG.error(
                f"current cl starting point not set before! {self.vid} {self.status.display_name} {self.cl_start_time}")
            raise EnvironmentError
        record_dict[G_VR_LEG_START_POS] = self.routing_engine.return_position_str(self.cl_start_pos)
        record_dict[G_VR_LEG_END_POS] = self.routing_engine.return_position_str(self.pos)
        record_dict[G_VR_LEG_DISTANCE] = self.cl_driven_distance
        record_dict[G_VR_LEG_START_SOC] = self.cl_start_soc
        record_dict[G_VR_LEG_END_SOC] = self.soc
        record_dict[G_VR_CHARGING_POWER] = ca.power
        record_dict[G_VR_CHARGING_UNIT] = ""  # no charging implemented
        record_dict[G_VR_TOLL] = self.cl_toll_costs
        record_dict[G_VR_OB_RID] = ";".join([str(rq.get_rid_struct()) for rq in self.pax])
        record_dict[G_VR_NR_PAX] = sum([rq.nr_pax for rq in self.pax])
        # record passengers adopt rq_dict
        ca.rq_dict[1] = boarded_rq_objs
        ca.rq_dict[-1] = deboarded_rq_objs
        list_boarding_pax = [rq.get_rid_struct() for rq in boarded_rq_objs]
        list_alighting_pax = [rq.get_rid_struct() for rq in deboarded_rq_objs]
        record_dict[G_VR_BOARDING_RID] = ";".join([str(rid) for rid in list_boarding_pax])
        record_dict[G_VR_ALIGHTING_RID] = ";".join([str(rid) for rid in list_alighting_pax])
        if self.record_route_flag:
            record_dict[G_VR_NODE_LIST] = ";".join([str(x) for x in self.cl_driven_route])
        if self.replay_flag:
            route_length = len(self.cl_driven_route)
            route_replay_str = ";".join([f"{self.cl_driven_route[i]}:{self.cl_driven_route_times[i]}" \
                                         for i in range(route_length)])
            record_dict[G_VR_REPLAY_ROUTE] = route_replay_str
        # default status and shift to next leg
        self.reset_current_leg(simulation_time)
        self.assigned_route = self.assigned_route[1:]
        self.op_output.append(record_dict)
        self.new_task_available = True
        return ca

    def start_next_leg(self, simulation_time):
        if self.assigned_route:
            LOG.debug(f"start_next_leg {self.vid} : {self.assigned_route[0]}")
            LOG.debug(f"Vehicle {self.vid} starts new VRL {self.assigned_route[0].__dict__} at time {simulation_time}")
            self.cl_start_time = simulation_time
            self.cl_start_pos = self.pos
            self.cl_start_soc = self.soc
            self.cl_driven_distance = 0.0
            self.cl_driven_route = []
            ca = self.assigned_route[0]
            self.status = ca.status
            if self.pos != ca.destination_pos:
                if ca.route and self.pos[0] == ca.route[0]:
                    self.cl_remaining_route = ca.route
                else:
                    self.cl_remaining_route = self.routing_engine.return_best_route_1to1(self.pos, ca.destination_pos)
                try:
                    self.cl_remaining_route.remove(self.pos[0])
                except ValueError:
                    # TODO # check out after ISTTT
                    LOG.warning(f"First node in position {self.pos} not found in currently assigned route {ca.route}!")
                self.cl_driven_route.append(self.pos[0])
                self.cl_driven_route_times.append(simulation_time)
                self.cl_remaining_time = None
        else:
            LOG.debug(f"start_next_leg {self.vid} : no")

    def reset_current_leg(self, simulation_time):
        # current info
        self.status = VRL_STATES.IDLE
        # current leg (cl) info
        self.cl_start_time = simulation_time
        self.cl_start_pos = self.pos
        self.cl_start_soc = self.soc
        self.cl_toll_costs = 0
        self.cl_driven_distance = 0.0
        self.cl_driven_route = []  # list of passed node_indices
        self.cl_driven_route_times = []  # list of times at which nodes were passed; only filled for replay flag
        self.cl_remaining_route = []  # list of remaining nodes to next stop
        self.cl_remaining_time = None
        self.cl_locked = False

    def set_initial_state(self, fleetctrl, routing_engine, state_dict, start_time, veh_init_blocking=True):
        r = super().set_initial_state(fleetctrl, routing_engine, state_dict, start_time,
                                      veh_init_blocking=veh_init_blocking)
        self.reset_current_leg(start_time)
        return r


class RppAppFleetSimulation(FleetSimulationBase):
    def __init__(self, scenario_parameters):
        super().__init__(scenario_parameters)
        self.rpp_ctrl: RPP_APP_FleetControl = self.operators[0]
        self.fs_time = -1

        self.external_to_internal_id = {}  # internal (within tum fs) and external (backend) rq_ids might differ (parcels with str "p_{id}")
        self.internal_to_external_id = {}

        self.number_vehicles = len(self.sim_vehicles)
        self.driver_id_to_vehicle_id = {}  # a driver is no vehicle in backend (vehicle_id -> driver_id)
        self.vehicle_id_to_driver_id = {}  # driver_id -> vehicle_id

        self.nodes_to_lat_lon_coordinate = {}  # backend is in lon lat representation
        self.r_tree = rtree.index.Index()  # to efficently find closes nodes
        # load network information for map matching and coordinate transformation
        crs_f = os.path.join(self.dir_names[G_DIR_NETWORK], "base", "crs.info")
        if not os.path.isfile(crs_f):
            raise FileNotFoundError("network crs file not found: {}".format(crs_f))
        n_crs = None
        n_epsg = None
        with open(crs_f) as fh_in:
            n_crs = {"init": fh_in.read().strip()}
            n_epsg = int(n_crs["init"][5:])
        node_gdf = gpd.read_file(os.path.join(self.dir_names[G_DIR_NETWORK], "base", "nodes_all_infos.geojson"),
                                 crs=n_crs)
        node_gdf.crs = n_crs
        node_gdf = node_gdf.to_crs({"init": f"epsg:{BACKEND_EPSG}"})
        edges_gdf = gpd.read_file(os.path.join(self.dir_names[G_DIR_NETWORK], "base", "edges_all_infos.geojson"),
                                 crs=n_crs)
        edges_gdf.crs = n_crs
        edges_gdf = edges_gdf.to_crs({"init": f"epsg:{BACKEND_EPSG}"})
        for node_id, p in zip(node_gdf["node_index"].values, node_gdf["geometry"].values):
            self.nodes_to_lat_lon_coordinate[node_id] = (p.x, p.y)
            self.r_tree.insert(node_id, (p.x, p.y, p.x, p.y))
            
        edges_gdf.set_index(["from_node", "to_node"], inplace=True)
        
        self.edges_gdf = edges_gdf
        
        self.oa_poly: Polygon = None
        if scenario_parameters.get("operating_area") is not None:
            oa_name = scenario_parameters.get("operating_area")
            print(self.dir_names)
            f = pd.read_csv(os.path.join(self.dir_names[G_DIR_DATA], "zones", oa_name, "oa.txt"))
            poly_list = [(x, y) for x, y in zip(f["lon"].values, f["lat"].values)]
            LOG.debug(f"load oa polygon: {poly_list}")
            self.oa_poly = Polygon(poly_list)

        log_message("Nodes and Edges are loaded")

        # deactivate all vehicles at beginning
        for vid in range(len(self.rpp_ctrl.sim_vehicles)):
            self.rpp_ctrl.deactivate_vehicle(vid, 0)
            self.sim_vehicles[(0, vid)].new_task_available = False
            
        self.rid_ignore_next_endboarding = {}   # external_rid -> if the deboarding process of this rid has to be ignored when called next (cancelled parcel while deboarding)

    @staticmethod
    def get_directory_dict(scenario_parameters):
        """
        This function provides the correct paths to certain data according to the specified data directory structure.
        :param scenario_parameters: simulation input (pandas series)
        :return: dictionary with paths to the respective data directories
        """
        # TODO # include zones and forecasts later on
        study_name = scenario_parameters[G_STUDY_NAME]
        scenario_name = scenario_parameters[G_SCENARIO_NAME]
        network_name = scenario_parameters[G_NETWORK_NAME]
        # demand_name = scenario_parameters[G_DEMAND_NAME]
        zone_name = scenario_parameters.get(G_ZONE_SYSTEM_NAME, None)
        infra_name = scenario_parameters.get(G_INFRA_NAME, None)
        # fc_type = scenario_parameters.get(G_FC_TYPE, None)
        # gtfs_name = scenario_parameters.get(G_GTFS_NAME, None)
        #
        dirs = {}
        dirs[G_DIR_MAIN] = TUM_FS_PATH
        dirs[G_DIR_DATA] = os.path.join(dirs[G_DIR_MAIN], "data")
        # TODO outputfolder
        dirs[G_DIR_OUTPUT] = os.path.join(dirs[G_DIR_MAIN], "studies", study_name, "results", scenario_name)
        dirs[G_DIR_NETWORK] = os.path.join(dirs[G_DIR_DATA], "networks", network_name)
        dirs[G_DIR_VEH] = os.path.join(dirs[G_DIR_DATA], "vehicles")
        dirs[G_DIR_FCTRL] = os.path.join(dirs[G_DIR_DATA], "fleetctrl")
        if infra_name is not None:
            dirs[G_DIR_INFRA] = os.path.join(dirs[G_DIR_DATA], "infra", infra_name, network_name)
        # dirs[G_DIR_DEMAND] = os.path.join(dirs[G_DIR_DATA], "demand", demand_name, "matched", network_name)
        if zone_name is not None:
            dirs[G_DIR_ZONES] = os.path.join(dirs[G_DIR_DATA], "zones", zone_name, network_name)
        #     if fc_type:
        #         dirs[G_DIR_FC] = os.path.join(dirs[G_DIR_DATA], "demand", demand_name, "aggregated", zone_name, fc_type)
        # if gtfs_name is not None:
        #     dirs[G_DIR_PT] = os.path.join(dirs[G_DIR_DATA], "pubtrans", gtfs_name)
        return dirs
    
    def _load_fleetctr_vehicles(self):
        """ Loads the fleet controller and vehicles """

        # simulation vehicles and fleet control modules
        LOG.info("Initialization of MoD fleets...")
        route_output_flag = self.scenario_parameters.get(G_SIM_ROUTE_OUT_FLAG, True)
        replay_flag = self.scenario_parameters.get(G_SIM_REPLAY_FLAG, False)
        veh_type_list = []
        for op_id in range(self.n_op):
            operator_attributes = self.list_op_dicts[op_id]
            operator_module_name = operator_attributes[G_OP_MODULE]
            self.op_output[op_id] = []  # shared list among vehicles
            if not operator_module_name == "LinebasedFleetControl":
                fleet_composition_dict = operator_attributes[G_OP_FLEET]
                list_vehicles = []
                vid = 0
                for veh_type, nr_veh in fleet_composition_dict.items():
                    for _ in range(nr_veh):
                        veh_type_list.append([op_id, vid, veh_type])
                        tmp_veh_obj = RppAppSimulationVehicle(op_id, vid, self.dir_names[G_DIR_VEH], veh_type,
                                                        self.routing_engine, self.demand.rq_db,
                                                        self.op_output[op_id], route_output_flag,
                                                        replay_flag)
                        list_vehicles.append(tmp_veh_obj)
                        self.sim_vehicles[(op_id, vid)] = tmp_veh_obj
                        vid += 1
                OpClass = load_fleet_control_module(operator_module_name)
                self.operators.append(OpClass(op_id, operator_attributes, list_vehicles, self.routing_engine, self.zones,
                                            self.scenario_parameters, self.dir_names, self.charging_operator_dict["op"].get(op_id, None), list(self.charging_operator_dict["pub"].values())))
            else:
                from dev.fleetctrl.LinebasedFleetControl import LinebasedFleetControl
                OpClass = LinebasedFleetControl(op_id, self.gtfs_data_dir, self.routing_engine, self.zones, self.scenario_parameters, self.dir_names, self.charging_operator_dict["op"].get(op_id, None), list(self.charging_operator_dict["pub"].values()))
                init_vids = OpClass.return_vehicles_to_initialize()
                list_vehicles = []
                for vid, veh_type in init_vids.items():
                    tmp_veh_obj = RppAppSimulationVehicle(op_id, vid, self.dir_names[G_DIR_VEH], veh_type,
                                                        self.routing_engine, self.demand.rq_db,
                                                        self.op_output[op_id], route_output_flag,
                                                        replay_flag)
                    list_vehicles.append(tmp_veh_obj)
                    self.sim_vehicles[(op_id, vid)] = tmp_veh_obj
                OpClass.continue_init(list_vehicles, self.start_time)
                self.operators.append(OpClass)
        veh_type_f = os.path.join(self.dir_names[G_DIR_OUTPUT], "2_vehicle_types.csv")
        veh_type_df = pd.DataFrame(veh_type_list, columns=[G_V_OP_ID, G_V_VID, G_V_TYPE])
        veh_type_df.to_csv(veh_type_f, index=False)
        self.vehicle_update_order = {vid : 1 for vid in self.sim_vehicles.keys()}


    def initialize_operators_and_vehicles(self):
        # TODO specify riksha vehicle types
        veh_type_list = []
        route_output_flag = self.scenario_parameters.get(G_SIM_ROUTE_OUT_FLAG, True)
        replay_flag = self.scenario_parameters.get(G_SIM_REPLAY_FLAG, False)
        for op_id in range(self.n_op):
            self.op_output[op_id] = []  # shared list among vehicles
            operator_attributes = self.list_op_dicts[op_id]
            operator_module_name = operator_attributes[G_OP_MODULE]
            fleet_composition_dict = operator_attributes[G_OP_FLEET]
            list_vehicles = []
            vid = 0
            for veh_type, nr_veh in fleet_composition_dict.items():
                for _ in range(nr_veh):
                    veh_type_list.append([op_id, vid, veh_type])
                    tmp_veh_obj = RppAppSimulationVehicle(op_id, vid, self.dir_names[G_DIR_VEH], veh_type,
                                                          self.routing_engine, self.demand.rq_db,
                                                          self.op_output[op_id], route_output_flag,
                                                          replay_flag)
                    list_vehicles.append(tmp_veh_obj)
                    self.sim_vehicles[(op_id, vid)] = tmp_veh_obj
                    vid += 1
            OpClass = load_fleet_control_module(operator_module_name)
            self.operators.append(OpClass(op_id, operator_attributes, list_vehicles, self.routing_engine, self.zones,
                                          self.scenario_parameters, self.dir_names, self.cdp))
        veh_type_f = os.path.join(self.dir_names[G_DIR_OUTPUT], "2_vehicle_types.csv")
        veh_type_df = pd.DataFrame(veh_type_list, columns=[G_V_OP_ID, G_V_VID, G_V_TYPE])
        veh_type_df.to_csv(veh_type_f, index=False)
        
    def _load_demand_module(self):
        """ Loads some demand modules """

        # demand module
        LOG.info("Initialization of travelers...")
        self.demand = SlaveDemand(self.scenario_parameters, self.user_stat_f)

        if self.zones is not None:
            self.zones.register_demand_ref(self.demand)

    def map_match_coordinates(self, coordinate_dict):
        """ this function matches the given coordinates to node position in the network
        :param coordinate_dict : dict longitude -> val; latitude -> val
        :return: node position in network"""
        y = float(coordinate_dict["latitude"])
        x = float(coordinate_dict["longitude"])
        r = (list(self.r_tree.nearest((x, y, x, y), 1))[0], None, None)
        LOG.debug("mapmatch {} -> {} with {}".format(coordinate_dict, r, self.nodes_to_lat_lon_coordinate[r[0]]))
        return r

    def return_coordinate_dict(self, node_id):
        """ this method is used to convert a node_id to the position representation of the backend
        :param node_id: node index
        :return: dict longitude -> val; latitude -> val
        """
        coords = self.nodes_to_lat_lon_coordinate[node_id]
        return {"latitude": coords[1], "longitude": coords[0]}
    
    def return_coordinate_list(self, route_node_list):
        #only nodes:
        # route_coordinates = [self.return_coordinate_dict(x) for x in route_node_list]
        #more detailed
        route_coordinates = []
        if len(route_node_list) > 0:
            if len(route_node_list) == 1:
                route_coordinates.append(self.return_coordinate_dict(route_node_list[0]))
            else:
                for i in range(1, len(route_node_list)):
                    cur_node = route_node_list[i-1]
                    next_node = route_node_list[i]
                    geo = self.edges_gdf.loc[(cur_node, next_node)]["geometry"]
                    for coords in geo.coords:
                        route_coordinates.append({"latitude": coords[1], "longitude": coords[0]})
        return route_coordinates

    def create_routeDTO(self, sim_vehicle, simulation_time):
        """ this function create route DTOs from assigned routes to communicate to backend

        LegDTO:
            {
                "id":5,
                "destinationPosition": {
                    "longitude": 50.0
                    "latitude": 48.3333
                },
                "destinationAddress": {
                    "street": "Am Graben",
                    "houseNumber": "714",
                    "postalCode": "1234 AB",
                    "city": "Garching (bei München)"
                },
                "nodes": [{"longitude": 50.0,"latitude": 48.3333}, 
                        {"longitude": 50.0,"latitude": 48.3333}, ...],
                "joiningRequests": [1],
                "leavingRequests": [2,4],
                "legDistance": 1360,
                "legDuration": 928
            }  

        RouteDTO:
            {
                "driverId": 1,
                "legDTOs": [LegDTO, LegDTO, ...]
            }

        :param sim_vehicle: simulation vehicle obj
        :param simulation_time: current simulation time
        :return: RouteDTO dict"""

        leg_list = []
        prev_route_node_list = []
        current_pos = sim_vehicle.pos
        for i, vrl in enumerate(sim_vehicle.assigned_route):
            if vrl.status == VRL_STATES.OUT_OF_SERVICE:
                LOG.info("ignore inactive state to send to backend!")
                continue
            if vrl.status in G_DRIVING_STATUS:
                if i == 0:
                    if sim_vehicle.cl_remaining_route:
                        prev_route_node_list = sim_vehicle.cl_remaining_route
                    else:
                        prev_route_node_list = self.routing_engine.return_best_route_1to1(sim_vehicle.pos,
                                                                                          vrl.destination_pos)
                else:
                    if not vrl.route:
                        prev_route_node_list = self.routing_engine.return_best_route_1to1(current_pos,
                                                                                          vrl.destination_pos)
                    else:
                        prev_route_node_list = vrl.route
            else:
                destination_node = vrl.destination_pos[0]
                destination_coords = self.return_coordinate_dict(destination_node)
                route_coordinates = self.return_coordinate_list(prev_route_node_list)
                joining_requests = [self.internal_to_external_id[rq.get_rid_struct()] for rq in vrl.rq_dict.get(1, [])]
                leaving_requests = [self.internal_to_external_id[rq.get_rid_struct()] for rq in vrl.rq_dict.get(-1, [])]
                if len(prev_route_node_list) > 1:
                    tt, dis = self.routing_engine.return_route_infos(prev_route_node_list, 0, 0)
                else:
                    tt, dis = 0, 0
                if tt < vrl.earliest_start_time - simulation_time:
                    tt = vrl.earliest_start_time - simulation_time
                legDTO = {
                    "id": -1,  # only for backend
                    "destinationPosition": destination_coords,
                    "destinationAddress": {  # TODO (needs to be specified?)
                        "street": "not defined",
                        "houseNumber": "not defined",
                        "postalCode": "not defined",
                        "city": "not defined"
                    },
                    "nodes": route_coordinates,
                    "joiningRequests": joining_requests,
                    "leavingRequests": leaving_requests,
                    "legDistance": dis,
                    "legDuration": tt  # TODO this now also specifies the earliest start time
                }
                leg_list.append(legDTO)
                prev_route_node_list = []
            current_pos = vrl.destination_pos
        routeDTO = {
            "driverId": self.vehicle_id_to_driver_id[sim_vehicle.vid],
            "legDTOs": leg_list
        }
        return routeDTO

    def _delete_rid(self, internal_request_id):
        ext_rid = self.internal_to_external_id[internal_request_id]
        del self.internal_to_external_id[internal_request_id]
        del self.external_to_internal_id[ext_rid]

    def create_Person_offer(self, rq_id, origin_coordinates, destination_coordinates, number_passengers=1,
                            earliest_pickup_time=None):
        """This method calls the fleet control to create an offer for a new person request.

        :param rq_id: request id (int) defined in backend
        :param origin_coordinates: dict longitude -> val; latitude -> val
        :param destination_coordinates: dict longitude -> val; latitude -> val
        :param number_passengers: number of passengers in this request
        :param earliest_pickup_time: earliest pickup time; if not given current time is used
        :return: offer dictionary
        """
        answer = None
        reject = False
        if self.oa_poly:
            if not self.oa_poly.contains( Point(float(origin_coordinates["longitude"]), float(origin_coordinates["latitude"]) ) ):
                reject = True
                answer = "Rejected: Start coordinates not in operating area!"
            if not self.oa_poly.contains( Point(float(destination_coordinates["longitude"]), float(destination_coordinates["latitude"]) ) ):
                if reject:
                    answer = "Rejected: Start and destination coordinates not in operating area!"
                else:
                    reject = True
                    answer = "Rejected: Destination coordinates not in operating area!"
        LOG.info("oa check: {}".format(answer))
        if not reject:
            o_node = self.map_match_coordinates(origin_coordinates)[0]
            d_node = self.map_match_coordinates(destination_coordinates)[0]
            ept = self.fs_time
            if earliest_pickup_time is not None:
                ept = earliest_pickup_time
            rq_info_dict = {G_RQ_ID: rq_id, G_RQ_ORIGIN: o_node, G_RQ_DESTINATION: d_node, G_RQ_TIME: self.fs_time,
                            G_RQ_EPT: ept, G_RQ_PAX: number_passengers}
            if o_node != d_node and o_node >= 0 and d_node >= 0:
                rq_series = pd.Series(rq_info_dict)
                rq_series.name = rq_id
                LOG.info("new request: {}".format(rq_info_dict))
                rq_obj = self.demand.add_request(rq_series, 0, self.routing_engine, self.fs_time)
                self.rpp_ctrl.user_request(rq_obj, self.fs_time)
                offer = self.rpp_ctrl.get_current_offer(rq_obj.get_rid_struct())
                LOG.info(f" -> offer: {offer}")
                rq_obj.receive_offer(0, offer, self.fs_time)
                if not offer.service_declined():
                    self.rpp_ctrl.user_confirms_booking(rq_obj.get_rid_struct(), self.fs_time)
                    answer = "Accepted: Driver is on the way!"
                else:
                    answer = "Rejected: Currently no driver available!"
                self.external_to_internal_id[rq_id] = rq_obj.get_rid_struct()
                self.internal_to_external_id[rq_obj.get_rid_struct()] = rq_id
            else:
                LOG.info("automatic decline: no offer for {}".format(rq_info_dict))
                offer = Rejection(rq_id, 0)
                answer = "Rejected: Start and destination at same location!"
                # self.external_to_internal_id[rq_id] = rq_id
                # self.internal_to_external_id[rq_id] = rq_id
        else:
            offer = Rejection(rq_id, 0)
        offer.additional_offer_parameters["answer"] = answer
        return offer

    def create_Parcel_offer(self, rq_id, origin_coordinates, destination_coordinates, parcel_size=1,
                            earliest_pickup_time=None):
        """This method calls the fleet control to create an offer for a new parcel request.

        :param rq_id: request id (int) defined in backend
        :param origin_coordinates: dict longitude -> val; latitude -> val
        :param destination_coordinates: dict longitude -> val; latitude -> val
        :param number_passengers: number of passengers in this request
        :param earliest_pickup_time: earliest pickup time; if not given current time is used
        :return: offer dictionary
        """
        answer = None
        reject = False
        if self.oa_poly:
            if not self.oa_poly.contains( Point(float(origin_coordinates["longitude"]), float(origin_coordinates["latitude"]) ) ):
                reject = True
                answer = "Rejected: Start coordinates not in operating area!"
            if not self.oa_poly.contains( Point(float(origin_coordinates["longitude"]), float(origin_coordinates["latitude"]) ) ):
                if reject:
                    answer = "Rejected: Start and destination coordinates not in operating area!"
                else:
                    reject = True
                    answer = "Rejected: Destination coordinates not in operating area!"
        LOG.info("oa check: {}".format(answer))
        if not reject:
            o_node = self.map_match_coordinates(origin_coordinates)[0]
            d_node = self.map_match_coordinates(destination_coordinates)[0]
            ept = self.fs_time
            if earliest_pickup_time is not None:
                ept = earliest_pickup_time
            rq_info_dict = {G_RQ_ID: rq_id, G_RQ_ORIGIN: o_node, G_RQ_DESTINATION: d_node, G_RQ_TIME: self.fs_time,
                            G_RQ_EPT: ept, G_RQ_PA_SIZE: parcel_size}
            if o_node != d_node and o_node >= 0 and d_node >= 0:
                rq_series = pd.Series(rq_info_dict)
                rq_series.name = rq_id
                rq_obj = self.demand.add_parcel_request(rq_series, 0, self.routing_engine, self.fs_time)
                self.rpp_ctrl.user_request(rq_obj, self.fs_time)
                offer = self.rpp_ctrl.get_current_offer(rq_obj.get_rid_struct())
                rq_obj.receive_offer(0, offer, self.fs_time)
                if not offer.service_declined():
                    self.rpp_ctrl.user_confirms_booking(rq_obj.get_rid_struct(), self.fs_time)
                    answer = "Accepted: Driver is on the way!"
                else:
                    answer = "Rejected: Currently no driver available!"
                self.external_to_internal_id[rq_id] = rq_obj.get_rid_struct()
                self.internal_to_external_id[rq_obj.get_rid_struct()] = rq_id
            else:
                LOG.info("automatic decline: no offer for {}".format(rq_info_dict))
                offer = Rejection(rq_id, 0)
                answer = "Rejected: Start and destination at same location!"
                # self.external_to_internal_id[rq_id] = rq_id
                # self.internal_to_external_id[rq_id] = rq_id
        else:
            offer = Rejection(rq_id, 0)
        offer.additional_offer_parameters["answer"] = answer
        return offer

    def remove_user(self, agent_id):
        """
        This method removes the user from the simulation; it is not specified if this is a decline of an offer or a no show
        :param agent_id: int id of backend (can differ with internal ids, because parcels are specified with p_{id})
        """
        try:
            internal_id = self.external_to_internal_id[agent_id]
        except KeyError:
            LOG.warning(f"request {agent_id} not found to delete -> do nothing")
            return
        to_delete, ignore_next_endboarding = self.rpp_ctrl.user_cancels_request(internal_id, self.fs_time)
        if to_delete:
            self.demand.user_cancels_request(internal_id, self.fs_time)
            try:
                del self.external_to_internal_id[agent_id]
            except KeyError:
                pass
            try:
                del self.internal_to_external_id[internal_id]
            except:
                pass
        if ignore_next_endboarding:
            self.rid_ignore_next_endboarding[agent_id] = 1

    def start_boarding(self, driver_id, position, simulation_time):
        """
        this function is triggered when a vehicle started a boarding process
        :param driver_id: driver id
        :param position: lon lat of current position
        :param simulation_time: current simulation time
        """
        network_position = self.map_match_coordinates(position)
        vehicle_id = self.driver_id_to_vehicle_id[driver_id]  # REMOVED -1
        ended_vrls, planned_boarding_pax, planned_alighting_pax = self.sim_vehicles[(0, vehicle_id)].start_boarding(
            network_position, simulation_time)
        for rid in planned_boarding_pax:
            self.demand.record_boarding(rid, vehicle_id, 0, simulation_time)
            self.rpp_ctrl.acknowledge_boarding(rid, vehicle_id, simulation_time)
        self.rpp_ctrl.receive_status_update(vehicle_id, simulation_time, ended_vrls, force_update=True)

    def end_boarding(self, driver_id, position, boarded_requests, alighted_requests, cancelled_requests,
                     undeliverable_parcels, simulation_time):
        """
        this function is triggered when a vehicle ended the boarding process
        :param vehicle_id: driver id
        :param position: network position of the boarding process
        :param boarded_requests: list of request ids (persons and parcels) that entered the vehicle
        :param alighted_requests: list of request ids (persons and parcels) that alighted the vehicle
        :param cancelled_requests: list of request ids (persons and parcels) that didnt enter the vehicle, but were planned to do so
        :param undeliverable_parcels: list of request ids (should only be parcels) that didn alight the vehicle, but were planned to do so
        :param simulation_time: current simulation time
        """
        nw_position = self.map_match_coordinates(position)
        vehicle_id = self.driver_id_to_vehicle_id[driver_id]
        cancelled_requests_trans = []
        for x in cancelled_requests:
            rid = self.external_to_internal_id.get(x)
            if self.rid_ignore_next_endboarding.get(x) is not None:
                LOG.info(f"ignore boarding of app request {x}! undeliverable parcel?")
                del self.rid_ignore_next_endboarding[x]
                continue
            if rid is not None:
                cancelled_requests_trans.append(rid)
            else:
                LOG.warning(f"app requests {x} not found! allready cancelled?")
        boarded_requests_trans = []
        for x in boarded_requests:
            rid = self.external_to_internal_id.get(x)
            if self.rid_ignore_next_endboarding.get(x) is not None:
                LOG.info(f"ignore boarding of app request {x}! undeliverable parcel?")
                del self.rid_ignore_next_endboarding[x]
                continue
            if rid is not None:
                boarded_requests_trans.append(rid)
            else:
                LOG.warning(f"app requests {x} not found! allready cancelled?")
        alighted_requests_trans = []
        for x in alighted_requests:
            rid = self.external_to_internal_id.get(x)
            if self.rid_ignore_next_endboarding.get(x) is not None:
                LOG.info(f"ignore boarding of app request {x}! undeliverable parcel?")
                del self.rid_ignore_next_endboarding[x]
                continue
            if rid is not None:
                alighted_requests_trans.append(rid)
            else:
                LOG.warning(f"app requests {x} not found! allready cancelled?")
        undeliverable_parcels_trans = []
        for x in undeliverable_parcels:
            rid = self.external_to_internal_id.get(x)
            if self.rid_ignore_next_endboarding.get(x) is not None:
                LOG.info(f"ignore boarding of app request {x}! undeliverable parcel?")
                del self.rid_ignore_next_endboarding[x]
                continue
            if rid is not None:
                undeliverable_parcels_trans.append(rid)
            else:
                LOG.warning(f"app requests {x} not found! allready cancelled?")

        for rid in cancelled_requests_trans:
            self.remove_user(self.internal_to_external_id[rid])
            # demand trigger cancel request
        for rid in undeliverable_parcels_trans:
            LOG.warning("undeliverable parcels not implemented yet!")
            pass
        ended_ca, list_boarding_pax, list_alighting_pax = self.sim_vehicles[(0, vehicle_id)].end_boarding(nw_position,
                                                                                                          boarded_requests_trans,
                                                                                                          alighted_requests_trans,
                                                                                                          cancelled_requests_trans,
                                                                                                          undeliverable_parcels_trans,
                                                                                                          simulation_time)
        # TODO : how to synchronize boarded request ids?
        for rq_obj in list_boarding_pax:
            rid = rq_obj.get_rid_struct()
            LOG.debug(f"rid {rid} boarding at {simulation_time}")
            self.demand.record_boarding(rid, vehicle_id, 0, simulation_time)
            self.rpp_ctrl.acknowledge_boarding(rid, vehicle_id, simulation_time)
        for rq_obj in list_alighting_pax:
            rid = rq_obj.get_rid_struct()
            # record user stats at beginning of alighting process
            LOG.debug(f"rid {rid} deboarding at {simulation_time}")
            self.demand.record_alighting_start(rid, vehicle_id, 0, simulation_time)
            # # record user stats at end of alighting process
            self.demand.user_ends_alighting(rid, vehicle_id, 0, simulation_time)
            self.rpp_ctrl.acknowledge_alighting(rid, vehicle_id, simulation_time)
            self._delete_rid(rid)
        # send update to operator
        self.rpp_ctrl.receive_status_update(vehicle_id, simulation_time, [ended_ca], True)
        self.rpp_ctrl.trigger_end_boarding(vehicle_id, simulation_time)

    def update_driver_position(self, driver_id, position, simulation_time):
        """ this function is triggered when a new update of the position is sent from the backend
        :param driver_id: id of driver
        :param position: lon lat coordinate dict
        :param simulation_time: current simulation time
        """
        nw_position = self.map_match_coordinates(position)
        sim_vid = None
        try:
            sim_vid = self.driver_id_to_vehicle_id[driver_id]
        except KeyError:
            LOG.warning(f"update position of driver {driver_id} but not activated yet?")
        if sim_vid is not None and self.sim_vehicles.get( (0, sim_vid) ):
            self.sim_vehicles[(0, sim_vid)].update_position(nw_position, simulation_time)

    def activate_driver(self, driver_id, external_veh_type, position, simulation_time):
        """ this function is used to activate a new driver for service
        :param driver_id: id driver
        :param external_veh_type: ENUM (TODO?) of specific vehicle id defined in backend
        :param position: lon lat coordinate dict
        :param simulation_time: current simulation time
        """
        LOG.debug(f"activate driver {driver_id} with veh_type {external_veh_type} at time {simulation_time}")
        if self.driver_id_to_vehicle_id.get(driver_id) is not None:
            LOG.warning(f"driver {driver_id} allready has a vehicle! {self.vehicle_id_to_driver_id}")
            return
        # veh_id = VEHICLE_TYPE_TO_VEHICLE_ID.get(external_veh_type) - 1
        # if veh_id is None:
        #     raise EnvironmentError(f"vehicle type {external_veh_type} unknown!")
        # veh_id = driver_id - 1
        for c in range(self.number_vehicles):
            if self.vehicle_id_to_driver_id.get(c) is None and VEHICLE_TYPE_TO_SIM_VEH_TYPE[external_veh_type] == self.sim_vehicles[(0, c)].veh_type:
                veh_id = c
                break
        if external_veh_type == "NONE":
            LOG.warning("activate NONE driver!")
            for c in range(self.number_vehicles):
                if self.vehicle_id_to_driver_id.get(c) is None and "LEDERHOSEN_RICKSHAW" == self.sim_vehicles[(0, c)].veh_type:
                    veh_id = c
                    break 
        if veh_id is None:
            raise NotImplementedError(f"Couldnt find free vehicle id. Either vehicles havent been deactivated or deactivation is not yet implemented correctly!")
        if veh_id >= len(self.sim_vehicles):
            raise EnvironmentError("Driver ID exceeds fleet size. Vehicles not in sync with backend?")
        nw_position = self.map_match_coordinates(position)
        self.sim_vehicles[(0, veh_id)].update_position(nw_position, simulation_time)
        self.driver_id_to_vehicle_id[driver_id] = veh_id
        self.vehicle_id_to_driver_id[veh_id] = driver_id
        self.rpp_ctrl.activate_vehicle(veh_id, simulation_time)
        LOG.info(f"activate driver {driver_id} with vehicle id {veh_id} and veh type {external_veh_type}")
        # TODO information of vehicle type not used currently!

    def deactivate_driver(self, driver_id, position, simulation_time):
        """ this function is used to deactivate a driver (all tasks have to be finished, but no new tasks
        will be assigned)
        :param driver_id: id of driver
        :param position: lon lat coordinates TODO why needed here?
        :param simulation: current simulation time"""
        veh_id = self.driver_id_to_vehicle_id.get(driver_id)
        if veh_id is None:
            LOG.info(f"no vehicle for driver {driver_id} found! {self.vehicle_id_to_driver_id}")
            return
        self.rpp_ctrl.deactivate_vehicle(veh_id, simulation_time)
        #LOG.warning("VEHICLES ARE NOT REALLY DEACTIVED CURRENTLY (TO FINISH THE CURRENTLY ASSIGNED TASK) TODO")
        # TODO how handle driver to vid? here the driver might still be in vehicle for a while
        try:
            del self.driver_id_to_vehicle_id[driver_id]
        except KeyError:
            pass
        try:
            del self.vehicle_id_to_driver_id[veh_id] 
        except KeyError:
            pass

    def get_new_routeDTOs(self):
        """ this function returns updates for vehicle route
        :return: list routeDTOs"""
        routeDTOs = []
        for simvid, sim_vehicle in self.sim_vehicles.items():
            if sim_vehicle.new_task_available:
                #print(simvid, sim_vehicle.vid)
                if self.vehicle_id_to_driver_id.get(sim_vehicle.vid) is None:
                    LOG.warning(f"new route for vehicle {simvid} but not driver assigned!")
                    sim_vehicle.new_task_available = False
                    continue
                routeDTO = self.create_routeDTO(sim_vehicle, self.fs_time)
                sim_vehicle.new_task_available = False
                routeDTOs.append(routeDTO)
        return routeDTOs
    
    def rebuild_database(self, driver_infos, person_rq_list, parcel_rq_list, ob_request_tuples, driver_to_route):
        """ this function is used to reset the fleetctrl in case it crashed and is restarted. all information is sent by the backend and its current state
        :param driver_infos: list of tuple (driver_id, vehicle_type, last position, state)
        :param person_rq_list: list of tuple (rq_id, origin_coordinates, destination_coordinates, number_passengers, earliest_pickup_time, state)
        :param parcel_rq_list: list of tuple (rq_id, origin_coordinates, destination_coordinates, parcel_size, earliest_pickup_time, state)
        :param ob_request_tuples: list of (rq_id, driver_id) for rq ob of driver
        :param driver_to_route: dict driver_id -> list of tuple (pos, boarding dict +1 -> list boarding rids; -1 -> list deboarding rids"""
        
        r_ob_dict = {rq_id : driver_id for rq_id, driver_id in ob_request_tuples}
        d_ob_dict = {}
        for rq_id, driver_id in ob_request_tuples:
            if d_ob_dict.get(driver_id):
                d_ob_dict[driver_id].append(rq_id)
            else:
                d_ob_dict[driver_id] = [rq_id]
        
        rq_id_to_state = {}
        rid_objectes = {}
        person_rq_objects = []
        for rq_id, origin_coordinates, destination_coordinates, number_passengers, earliest_pickup_time, state in person_rq_list:
            o_node = self.map_match_coordinates(origin_coordinates)[0]
            d_node = self.map_match_coordinates(destination_coordinates)[0]
            ept = earliest_pickup_time
            rq_info_dict = {G_RQ_ID: rq_id, G_RQ_ORIGIN: o_node, G_RQ_DESTINATION: d_node, G_RQ_TIME: self.fs_time,
                            G_RQ_EPT: ept, G_RQ_PAX: number_passengers}
            if o_node != d_node and o_node >= 0 and d_node >= 0:
                rq_series = pd.Series(rq_info_dict)
                rq_series.name = rq_id
                LOG.info("new request: {}".format(rq_info_dict))
                rq_obj = self.demand.add_request(rq_series, 0, self.routing_engine, self.fs_time)
                person_rq_objects.append(rq_obj)
                self.external_to_internal_id[rq_id] = rq_obj.get_rid_struct()
                self.internal_to_external_id[rq_obj.get_rid_struct()] = rq_id
                rq_id_to_state[rq_id] = state
                rid_objectes[rq_id] = rq_obj
                
        parcel_rq_objects = []
        for rq_id, origin_coordinates, destination_coordinates, parcel_size, earliest_pickup_time, state in parcel_rq_list:
            o_node = self.map_match_coordinates(origin_coordinates)[0]
            d_node = self.map_match_coordinates(destination_coordinates)[0]
            ept = earliest_pickup_time
            rq_info_dict = {G_RQ_ID: rq_id, G_RQ_ORIGIN: o_node, G_RQ_DESTINATION: d_node, G_RQ_TIME: self.fs_time,
                            G_RQ_EPT: ept, G_RQ_PA_SIZE: parcel_size}
            if o_node != d_node and o_node >= 0 and d_node >= 0:
                rq_series = pd.Series(rq_info_dict)
                rq_series.name = rq_id
                rq_obj = self.demand.add_parcel_request(rq_series, 0, self.routing_engine, self.fs_time)
                parcel_rq_objects.append(rq_obj)
                self.external_to_internal_id[rq_id] = rq_obj.get_rid_struct()
                self.internal_to_external_id[rq_obj.get_rid_struct()] = rq_id
                rq_id_to_state[rq_id] = state
                rid_objectes[rq_id] = rq_obj
                
        for driver_id, vehicle_type, last_position, state in driver_infos:
            route = driver_to_route.get(driver_id, [])
            #self.activate_driver(driver_id, vehicle_type, last_position)
            if len(route) == 0 and len(d_ob_dict.get(driver_id, [])) == 0 and state != "ACTIVE_BOARDING" and state != "END_SHIFT_BOARDING":
                LOG.info(f"driver {driver_id} should be good to go!")
                self.activate_driver(driver_id, vehicle_type, last_position, self.fs_time)
            else:
                LOG.info(f"driver with previous assignments: {driver_id}")
                LOG.info(f"     -> {state} {d_ob_dict.get(driver_id, [])} {route}")
                # check feasible on-board state
                currently_ob = []
                associated_rids = {}
                for i in range(len(route)-1, 0, -1):
                    for d_r in route[i][1].get(-1, []):
                        if rid_objectes.get(d_r):
                            currently_ob.append(d_r)
                            associated_rids[d_r] = 1
                    for b_r in route[i][1].get(1, []):
                        if rid_objectes.get(b_r):
                            currently_ob.remove(b_r)
                            associated_rids[b_r] = 1
                if state != "ACTIVE_BOARDING" and state != "END_SHIFT_BOARDING":
                    for d_r in route[0][1].get(-1, []):
                        if rid_objectes.get(d_r):
                            currently_ob.append(d_r)
                            associated_rids[d_r] = 1
                    for b_r in route[0][1].get(1, []):
                        if rid_objectes.get(b_r):
                            currently_ob.remove(b_r)
                            associated_rids[b_r] = 1   
                            
                for o_r in d_ob_dict.get(driver_id, []):
                    if rid_objectes.get(o_r):
                        associated_rids[o_r] = 1
                if len(associated_rids) == 0:
                    continue
                    
                request_collection = [rid_objectes[rid] for rid in associated_rids]
                def convert_boarding_dict(boarding_dict):
                    bd = [self.external_to_internal_id[rid] for rid in boarding_dict.get(1, [])]
                    dbd = [self.external_to_internal_id[rid] for rid in boarding_dict.get(-1, [])]
                    return {1: bd, -1: dbd}                    
                if sorted(currently_ob) == sorted(d_ob_dict.get(driver_id, [])):
                    LOG.info(" -> route complete!")
                    simulate_boarding = False
                    deboarding = True
                    if len(currently_ob) != 0:
                        simulate_boarding = True
                        route = [(last_position, {1 : currently_ob, -1: []})] + route[:]
                        if state == "ACTIVE_BOARDING" or state == "END_SHIFT_BOARDING":
                            LOG.warning("currently boarding")
                            deboarding = False
                    self.activate_driver(driver_id, vehicle_type, last_position, self.fs_time)
                    mapmatched_route = [(self.map_match_coordinates(r[0]), convert_boarding_dict(r[1])) for r in route]
                    self.rpp_ctrl.rebuild_vehicle_route(self.driver_id_to_vehicle_id[driver_id], mapmatched_route, request_collection, self.fs_time)
                    for rq_obj in request_collection:
                        rq_obj.receive_offer(0, self.rpp_ctrl.get_current_offer(rq_obj.get_rid_struct()), self.fs_time)
                    if simulate_boarding:
                        self.start_boarding(driver_id, last_position, self.fs_time)
                        if deboarding:
                            self.end_boarding(driver_id, last_position, currently_ob, [], [], [], self.fs_time)
                    # if state == "ACTIVE_BOARDING" or state == "END_SHIFT_BOARDING":
                    #     self.start_boarding(driver_id, last_position, self.fs_time)
                    if state == "END_SHIFT_DRIVING" or state == "END_SHIFT_BOARDING":
                        self.deactivate_driver(driver_id, last_position, self.fs_time)
                else:
                    LOG.info(f" -> stop missing: {currently_ob} {d_ob_dict.get(driver_id, [])}")
                    raise NotImplementedError("restart not complete! (route and on-board rids dont fit!)")

    def step(self, sim_time):
        """This method determines the simulation flow in a time step.
            # 1) update fleets and network
            # 2) get new travelers, add to undecided request
            # 3) sequential processes for each undecided request: request -> offer -> user-decision
            # 4) periodically for waiting requests: run decision process -> possibly leave system (cancellation)
            # 5) periodically operator: call ride pooling optimization, repositioning, charging management
            # 6) trigger charging infra 

        :param sim_time: new simulation time
        :return: None
        """
        # 5)
        self.fs_time = sim_time
        self.rpp_ctrl.time_trigger(sim_time)      


class RppAppInterface:
    def __init__(self):
        """This method initiates the interface class between the RPP Backend and the Fleetsimulation

        """

        LOG.info("Initialize Interface")

        e = datetime.datetime.now()
        cur_t_str = f"{e.year}_{e.month}_{e.day}_{e.hour}_{e.minute}"

        cfg = config.ConstantConfig(os.path.join(RPP_CONFIG_PATH, "scenarios", "rpp_service_config.yaml"))
        cfg[G_SCENARIO_NAME] = "{}_{}".format(cfg[G_SCENARIO_NAME], cur_t_str)
        self.fs_obj = RppAppFleetSimulation(cfg)
        self.routing_enging = self.fs_obj.routing_engine  # for map matching

        # this dictionary is used to match function strings from backend to methods
        self.string_to_function = {
            "UPDATE_DRIVER_POSITIONS": self.update_driver_position,
            "START_BOARDING": self.start_boarding,
            "END_BOARDING": self.end_boarding,
            "ACTIVATE_DRIVER": self.activate_driver,
            "DEACTIVATE_DRIVER": self.deactivate_driver,
            "ADD_REQUEST": self.add_request,
            "REMOVE_REQUEST": self.remove_request,
            "CHANGE_REQUEST": self.change_request,
            "RESTART_FLEETCTRL": self.restart_fleetctrl
        }

        self.start_time = None  # absolute time str from start of established connection to backend
        self.int_start_time = None  # int of absolute time in seconds of start of established connection to backend
        self.current_time = 0  # int relative time in seconds passed since connection to backend has been established

        self.current_request_DTOs = {}  # dictionary request id -> request_DTO

        self.new_request_DTOs = {}  # updates for requests i.e. offers to send to back end (dict rq_id -> 1)
        self.new_route_DTOs = {}  # updates for vehicles to send to back end  (dict veh_id -> routeDTO )

        out = self.fs_obj.dir_names[G_DIR_OUTPUT]
        self.com_f = os.path.join(out, "com.log")
        with open(self.com_f, "w") as f:
            f.write("")
        # set up communication need to be handle
        self.com = None

    def int_to_string_time(self, time):
        """ this function returns the string representation of time for the backend
        :param time: int of simulation time
        :return: string of absolute time representation """
        abs_time = self.int_start_time + time
        time_struct = datetime.timedelta(0, abs_time) + REFERENCE_TIME
        return time_struct.strftime(TIME_FORMAT_STR)

    def establish_com_to_backend(self):
        # Used for python test
        """ this method always runs and collects information from the input-stream to collect updates from the backend and writes new controll updates to the output-stream"""
        # have to set a boundary length for message
        LOG.info("Good to go!")
        # initialize the communication
        current_msg = ""
        full_msg = None
        try:
            while (self.com != None and self.com.is_alive()):
                try:
                    c_stream_msg = self.com.recv(2048)
                except:
                    LOG.warning("couldnt decode message!")
                    continue
                # if recv empty message it means the socket is closed
                if c_stream_msg == "":
                    LOG.warning("empty msg recieved from backend -> close socket")
                    break
                # recv the message check if the full message is acquired
                if "\n" in c_stream_msg:
                    tmp = c_stream_msg.split("\n")
                    full_msg = current_msg + tmp[0]
                    #current_msg = tmp[1]
                    current_msg = tmp[1] if tmp[1].strip() != 'heartbeat' else ''
                else:
                    full_msg = None
                    current_msg += c_stream_msg

                # handle the full message    
                if full_msg is not None:
                    print("full msg")
                    print(full_msg)
                    print("")
                    if full_msg.startswith("heartbeat"):
                        continue
                    new_task = ""
                    if RECORD_COM:
                        with open(self.com_f, "a") as f:
                            f.write("INCOMING:\n")
                            f.write(full_msg)
                            f.write("\n")
                    try:
                        new_task = json.loads(full_msg)
                    except:
                        LOG.error("no json object incoming? {}".format(full_msg))
                        log_message("no json object incoming? {}".format(full_msg))
                        continue
                    if (new_task != ""):
                        self.update_time(new_task["currentTime"])
                        self.string_to_function[new_task["function"]](new_task["data"])
                        self.answer_to_backend()
        except:
            #traceback.print_exc()
            var = traceback.format_exc()
            LOG.error(var)
        finally:
            log_message("closing connect")
            #send error
            answer_dict = {
                "function": "ERROR",
                "data": ""
            }
            if RECORD_COM:
                with open(self.com_f, "a") as f:
                    f.write(json.dumps(answer_dict))
                    f.write("\n")
            if self.com is not None:
                self.com.send(json.dumps(answer_dict))
            else:
                LOG.warning("self.com not initialized!!")            
            
            self.com.close_connect()

    def answer_to_backend(self):
        """ this method writes json-meassages to the console which are read by the backend """
        # update requestDTOs
        if RECORD_COM:
            with open(self.com_f, "a") as f:
                f.write("OUTGOING:\n")
        for rq_id in self.new_request_DTOs.keys():
            answer_dict = {
                "function": "REQUEST_UPDATE",
                "data": self.current_request_DTOs[rq_id]
            }
            if RECORD_COM:
                with open(self.com_f, "a") as f:
                    f.write(json.dumps(answer_dict))
                    f.write("\n")
            if self.com is not None:
                self.com.send(json.dumps(answer_dict))
            else:
                LOG.warning("self.com not initialized!!")
        self.new_request_DTOs = {}
        # update routeDTOs
        new_routeDTOs = self.fs_obj.get_new_routeDTOs()
        for routeDTO in new_routeDTOs:
            answer_dict = {
                "function": "VEHICLE_ROUTE",
                "data": routeDTO
            }
            if RECORD_COM:
                with open(self.com_f, "a") as f:
                    f.write(json.dumps(answer_dict))
                    f.write("\n")
            if self.com is not None:
                self.com.send(json.dumps(answer_dict))
            else:
                LOG.warning("self.com not initialized!!")
        self.fs_obj.record_stats(force=True)

    def test_com(self, json_msg):
        msg = json_msg
        if RECORD_COM:
            with open(self.com_f, "a") as f:
                f.write("INCOMING:\n")
                f.write(msg)
                f.write("\n")
        try:
            if (msg != 'heartbeat'):
                new_task = json.loads(msg)
        except:
            LOG.error("no json object incoming? {}".format(msg))
            return
        
        self.update_time(new_task["currentTime"])
        self.string_to_function[new_task["function"]](new_task["data"])
        self.answer_to_backend()

    def update_time(self, str_time):
        """ this function is used to update the time in the interface and the fleet simulation environment
        :param time_str: string representation of time in back end
        """
        if self.start_time is None or self.int_start_time is None:
            self.start_time = str_time
            self.int_start_time = string_time_to_abs_int(str_time)
        new_time = string_time_to_abs_int(str_time) - self.int_start_time
        if new_time >= self.current_time:
            self.current_time = new_time
            self.fs_obj.step(new_time)
        elif new_time < self.current_time:
            LOG.warning("incoherent time")

    def start_boarding(self, data_field):
        """ this method is used to inform, that a boarding process started
        data field from back end:
        {
            "driverId": 5,
            "position": {"longitude": 50.4001, "latitude": 21.2024}
        }
        :param data_field: dict as specified as above
        """
        driver_id = int(data_field["driverId"])
        position = data_field["position"]
        self.fs_obj.start_boarding(driver_id, position, self.current_time)

    def end_boarding(self, data_field):
        """ this method is used to inform, that a boarding process has ended
        data field from back end:
        {
            "driverId": 5,
            "position": {"longitude": 50.4001, "latitude": 21.2024},
            "joiningRequests": [12, 29, 51, 70],
            "leavingRequests": [4, 21, 87],
            "cancelledRequests": [],
            "undeliverableParcelRequests": [30]
        }
        :param data_field: dict as specified as above
        """
        driver_id = int(data_field["driverId"])
        position = data_field["position"]
        list_joining_requests = [int(x) for x in data_field["joiningRequests"]]
        list_leaving_requests = [int(x) for x in data_field["leavingRequests"]]
        list_cancelled_requests = [int(x) for x in data_field["cancelledRequests"]]
        list_undeliverable_parcel_requests = [int(x) for x in data_field["undeliverableParcelRequests"]]
        self.fs_obj.end_boarding(driver_id, position, list_joining_requests, list_leaving_requests,
                                 list_cancelled_requests, list_undeliverable_parcel_requests, self.current_time)

    def activate_driver(self, data_field):
        """ this function activates a driver (vehicle) in the fleet simulation
        backend data field:
        {
            "function": "ACTIVATE_DRIVER",
            "data": {
                "driverId": 5,
                "vehicleType": VehicleType,
            "position": {"longitude": 50.4001, "latitude": 21.2024}
            },
            "currentTime": "2021-04-28 14:45:16.43"
        }
        :param data_field: dict as specified as above
        """
        driver_id = int(data_field["driverId"])
        veh_type = data_field["vehicleType"]
        position = data_field["position"]
        self.fs_obj.activate_driver(driver_id, veh_type, position, self.current_time)

    def deactivate_driver(self, data_field):
        """ this function is used to deactivate a driver (vehicle) in the fleet simulation
        backend data field:
        {
            "driverId": 5,
            "position": {"longitude": 50.4001, "latitude": 21.2024}
        }
        :param data_field: dict as specified as above
        """
        driver_id = int(data_field["driverId"])
        position = data_field["position"]
        self.fs_obj.deactivate_driver(driver_id, position, self.current_time)

    def add_request(self, data_field):
        """ this function is used to communicate a new request
        backend data field:
        {
            "request": RequestDTO
            },
        }
        :param data_field: dict as specified as above
        """
        rq_dict = data_field["request"]
        rq_id = int(rq_dict["reqId"])
        rq_type = rq_dict["type"]
        origin_coords = rq_dict["startPosition"]
        destination_coords = rq_dict["goalPosition"]
        if rq_type == "PERSON":  # TODO check if these are the right enums or if they are ints
            number_passengers = rq_dict["personInformation"]["numberOfPassengers"]
            offer = self.fs_obj.create_Person_offer(rq_id, origin_coords, destination_coords,
                                                    number_passengers=number_passengers)
        elif rq_type == "PARCEL":  # same TODO # TODO other parcelInformations
            parcel_size = PARCEL_SIZE_STR_TO_INT[rq_dict["parcelInformation"]["parcelSize"]]
            offer = self.fs_obj.create_Parcel_offer(rq_id, origin_coords, destination_coords, parcel_size=parcel_size)
        # update RequestDTO
        if rq_dict["state"] != "PENDING":
            raise EnvironmentError("an initial request with state that is not PENDING? {}".format(rq_dict))
        else:
            rq_dict["reason"] = offer.get("answer")
            if offer.service_declined():
                rq_dict["state"] = "DECLINED"
            else:
                rq_dict["state"] = "ACCEPTED"
                rq_dict["startTime"] = self.int_to_string_time(offer[G_OFFER_WAIT] + self.current_time)
                # TODO # specify drop off time for parcels; other attributes for persons? expacted travel time?
        self.current_request_DTOs[rq_id] = rq_dict
        self.new_request_DTOs[rq_id] = 1

    def remove_request(self, data_field):
        """ this function is used to communicate that a request has to be removed (i.e. cancelled)
        backend data field:
        {
            "reqId": 254
        }
        :param data_field: dict as specified as above
        """
        rid = int(data_field["request"]["reqId"])
        rq_dict = data_field["request"]
        rq_dict["reason"] = "Request Cancelled"
        rq_dict["state"] = "CANCELLED"
        self.new_request_DTOs[rid] = 1
        self.current_request_DTOs[rid] = rq_dict
        self.fs_obj.remove_user(rid)
        # TODO # do we need to update the state here? -> yes

    def change_request(self, data_field):
        """ this function is called in case a request changed some parameters
        backend data field:
        {
            "request": RequestDTO
        }
        :param data_field: dict as specified as above
        """
        # for now: remove old request and make a new request TODO ! -> entscheidung der fleet ctrl (packetgröße, anzahl passgiere)
        rq_dict = data_field["request"]
        self.remove_request(rq_dict)
        self.add_request(data_field)

    def update_driver_position(self, data_field):
        """ this function is used to update the position of vehicles
        data field from backend:
        {
            "1": {"longitude": 50.4001, "latitude": 21.2024},
            "2": {"longitude": 50.4001, "latitude": 21.2024}
        }
        keys refer to triver ids
        :param data_field: dict as specified as above
        """
        for driver_id_str, position in data_field.items():
            driver_id = int(driver_id_str)
            self.fs_obj.update_driver_position(driver_id, position, self.current_time)
            
    def restart_fleetctrl(self, data_field):
        """ this function is used when the fleetctrl has to be restart.
        current fleet states are sent by th backend
        data field from backend:
        {
            "timestamp": "2022-06-12 19:00:00.01",
            "activeRequests": [RequestDTO1, RequestDTO2, ...]    # list of all requests that are still in the system
            "activeDrivers": [DriverDTO1, DriverDTO2, ...]    # list of all drivers
            "onboardRequests": [(requestID1, driverID1), (requestID2, driverID1), (requestID3, driverID2), ...]    # list of tuples telling i.e. requestID1 is on-board of driverID1,...
            "activeRoutes" : [RouteDTO1, RouteDTO2, ...]    # list of currently assigned Routes
        }
        """
        LOG.info("restart fleetctrl")
        driver_infos = []
        for driverDTO in data_field["activeDrivers"]:
            LOG.info(f"driver: {driverDTO}")
            driver_infos.append( (driverDTO["id"], driverDTO["currentVehicleType"], driverDTO["lastTimestampedPosition"]["position"], driverDTO["state"]) )
            
        person_rq_infos = []
        parcel_rq_infos = []
        for rqDTO in data_field["activeRequests"]:
            LOG.info(f"request: {rqDTO}")
            rid = int(rqDTO["reqId"])
            orig = rqDTO["startPosition"]
            dest = rqDTO["goalPosition"]
            ept = string_time_to_abs_int(rqDTO["startTime"]) - self.int_start_time
            typ = rqDTO["type"]
            state = rqDTO["state"]
            if state == "PENDING":  # reject unanswered requests -> they should request again
                rqDTO["state"] = "DECLINED"
                rqDTO["reason"] = "Server Error! Please try again in a minute!"
                self.current_request_DTOs[rqDTO["reqId"]] = rqDTO 
                self.new_request_DTOs[rqDTO["reqId"]] = 1
                continue
            if typ == "PERSON":
                number_passengers = rqDTO["personInformation"]["numberOfPassengers"]
                person_rq_infos.append( (rid, orig, dest, number_passengers, ept, state) )
            else:
                parcel_size = PARCEL_SIZE_STR_TO_INT[rqDTO["parcelInformation"]["parcelSize"]]
                parcel_rq_infos.append( (rid, orig, dest, parcel_size, ept, state) )
                
        ob_request_tuples = data_field["onboardRequests"]
        LOG.info(f"onboard requests {ob_request_tuples}")
        
        driver_to_route = {}
        for routeDTO in data_field["activeRoutes"]:
            driver_id = routeDTO["driverId"]
            legDTOs = routeDTO["legDTOs"]
            route = []
            for legDTO in legDTOs:
                boarding_dict = {
                    1 : legDTO["joiningRequests"],
                    -1 : legDTO["leavingRequests"]
                }
                pos = legDTO["destinationPosition"]
                route.append( (pos, boarding_dict) )
            LOG.info(f"driver {driver_id} : {route}")
            driver_to_route[driver_id] = route
            
        self.fs_obj.rebuild_database(driver_infos, person_rq_infos, parcel_rq_infos, ob_request_tuples, driver_to_route)
        

    def open_com(self):
        self.com = RppAppSocket(self.fs_obj)

    def close_com(self):
        # should be logged
        self.com.close_connect()
        log_message("closing connection")
        self.com = None


class RppAppSocket:
    """This method initiates the communcation socket between rpp and the python fleet simulation module.
    param fs_obj: instance of FleetSimulation; fs_obj.scenario_parameters contains all scenario parameters  
    """

    def __init__(self, fs_obj: FleetSimulationBase):
        # self.server_ip=fs_obj.scenario_parameters.get("HOST", "localhost")
        self.server_ip = "localhost"
        self.fleet_sim = fs_obj
        self.server_connection = socket.socket()
        self.server_port = fs_obj.scenario_parameters[G_COM_SOCKET_PORT]
        self.server_connection.bind((self.server_ip, self.server_port))
        self.client_connection = None
        self.alive = False
        self.wait_connect()

    def recv(self, num_bytes: int):
        """ this function is called to receive byte message
        :param num_bytes: number of bytes to read
        """

        byte_msg = self.client_connection.recv(num_bytes)
        msg = byte_msg.decode(ENCODE)
        if (not 'heartbeat' in msg):
            log_message("mess received" + msg)
        return msg

    def send(self, msg: str):
        """ this function is called to send message
        :param msg: message to send
        """
        # should sending signal be logged?
        msg += '\r\n'
        byte_msg = msg.encode(ENCODE)
        self.client_connection.sendall(byte_msg)
        log_message("data sent " + msg)

    def wait_connect(self):
        """ this function is called to open connection and listen to connect
        """
        log_message("Tumfleet is ready. Waiting for Connection to Server: ....")
        self.server_connection.listen()
        self.client_connection, client_address = self.server_connection.accept()
        log_message("Successful connected by " + str(client_address))
        self.alive = True

    def is_alive(self):
        """ getter alive
        """
        return self.alive

    def close_connect(self):
        """ this function is called to close connection and end establish_back_end loop
        """
        self.server_connection.close()
        log_message("Socket closed")
        self.alive = False
