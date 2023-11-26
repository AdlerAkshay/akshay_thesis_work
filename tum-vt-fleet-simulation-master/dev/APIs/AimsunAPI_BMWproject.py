""" 
THIS FILE HAS TO BE LOADED INTO AIMSUN
FOLLOWING PATHS HAVE TO BE SET THEREFORE:
SITEPACKAGES: PATH TO YOUR PYTHON PACKEGES
TUM_FLEET_SIMULATION_ABS_PATH: PATH TO TUM_FLEET_SIMULATION DIRECTORY
"""

import os
import sys
SITEPACKAGES = r"C:\ProgramData\Anaconda3\envs\roman\Lib\site-packages"
SITEPACKAGE_TO_REMOVE = r'C:\ProgramData\Anaconda3\Lib\site-packages'   
#ROMAN LAPTOP
#TUM_FLEET_SIMULATION_ABS_PATH = r'C:\Users\ge37ser\Documents\Coding\TUM_VT_FleetSimulation\tum-vt-fleet-simulation'
#ThreadRipper4000
TUM_FLEET_SIMULATION_ABS_PATH = r'C:\Users\ge37ser\Documents\tum-vt-fleet-simulation'

# for p in sys.path:
#     print(p, SITEPACKAGE_TO_REMOVE, p == SITEPACKAGE_TO_REMOVE)

if SITEPACKAGE_TO_REMOVE in sys.path:
    sys.path.remove(SITEPACKAGE_TO_REMOVE)
    print("remove {} from sys-path!".format(SITEPACKAGE_TO_REMOVE))
if SITEPACKAGES not in sys.path:
    sys.path.append(SITEPACKAGES)
if TUM_FLEET_SIMULATION_ABS_PATH not in sys.path:
    sys.path.append(TUM_FLEET_SIMULATION_ABS_PATH)

# for p in sys.path:
#     print(p)

# -------------------------------------------------------------------------------------------------------------------- #
# standard distribution imports
# -----------------------------
import traceback
import importlib

# additional module imports (> requirements)
# ------------------------------------------
import numpy as np
import pandas as pd

import socket
#import json
import pickle
import datetime
import time
import logging 
import cProfile, pstats, io
import csv

# aimsun imports
# -------------------------------------------------------
from AAPI import *
from PyANGBasic import *
from PyANGKernel import *
from PyANGConsole import *
#from PyANGAimsun import *

# system, model
system = GKSystem.getSystem()
model = GKSystem.getSystem().getActiveModel()

# fleet sim imports
from src.misc.globals import *

STAT_INT = 60
ENCODING = "utf-8"
LOG_COMMUNICATION = True
LARGE_INT = 100000
RECV_SIZE = 1024
HEADERSIZE = 20

LOG = logging.getLogger(__name__)

# =============================================================================================#
#NETWORK STAT FUNCTIONS
# ==============================================================================================#

def wrtieNetInfo(Path):

    NumSec  = AKIInfNetNbSectionsANG()
    NumJunc = AKIInfNetNbJunctions()
    '''
    NetInfo = open (Path + 'NetInfo.csv' ,'w')
    NetInfo.write ('NumSections,' + 'NumNodes' + '\n')
    NetInfo.write ( str(NumSec) + ',' + str(NumJunc) )
    NetInfo.close
    '''
    FileName1 = os.path.join(Path, 'MUCSimInfo.csv')
    if not os.path.isfile(FileName1):
        with open(FileName1, 'w') as SecInfo:
            Sec_writer = csv.writer(SecInfo,lineterminator='\n')
            Sec_writer.writerow([ 'LinkID','TravelTime (sec)','Speed (km/h)', 'Density (Veh/km)','Count (veh)','SimTime','DayTime'])
    #SecInfo.close
    
    FileName2 = os.path.join(Path,'AreaInfo.csv')
    if not os.path.isfile(FileName2):
        with open(FileName2, 'w') as SecInfo:
            Sec_writer = csv.writer(SecInfo,lineterminator='\n')
            Sec_writer.writerow([ 'ID', 'length', 'nLanes', 'laneWidth'])
    #SecInfo.close
    
    return 0


def writeSectionInfo(time,timeSta, Path):

    DetectionInt = AKIEstGetIntervalStatistics()
    NumSec = AKIInfNetNbSectionsANG() # number of sections
    FileName1 = os.path.join(Path, 'MUCSimInfo.csv')
    with open(FileName1, 'a') as SecInfo:
        Sec_writer = csv.writer(SecInfo,lineterminator='\n')
        for el in range(NumSec):
            idOfSection = AKIInfNetGetSectionANGId (el)
            SectionStat = AKIEstGetParcialStatisticsSection(idOfSection,time,0)
            #FileName1 = Path + 'MUCSimInfo.csv'
            # with open(FileName1, 'a') as SecInfo:
        #     Sec_writer = csv.writer(SecInfo,lineterminator='\n')
            Sec_writer.writerow([idOfSection, SectionStat.TTa , SectionStat.Sa , SectionStat.Density , SectionStat.count , time,timeSta])
        #SecInfo.close
 
    return 0

# ============================================================================================= #
#AIMSUN API FUNCTIONS#
# ============================================================================================= #

aimsun_controller = None
statistic_interval = -1
statistic_gathered = False
last_statistic_fetched = -1.0

profiler_enabled = False
if profiler_enabled:
    profiler = cProfile.Profile()
    profiler.enable()


def AAPILoad():

    return 0

def AAPIInit():
    global aimsun_controller    #init RP-Simulator
    aimsun_controller = AimsunVehicleController()
    aimsun_controller.set_external_init()

    global statistic_gathered
    statistic_gathered = AKIIsGatheringStatistics()
    if not statistic_gathered:
        LOG.warning("No network statistics gathered in Aimsun model!")
    global statistic_interval
    statistic_interval = AKIEstGetIntervalStatistics()
    LOG.info("New network statistics every {} seconds".format(statistic_interval))

    wrtieNetInfo(aimsun_controller.dir_names[G_DIR_OUTPUT])

    return 0

def AAPIManage(time, timeSta, timeTrans, acycle):
    return 0

def AAPIPostManage(time, timeSta, timeTrans, acycle):
    # LOG.debug("post manage statstics? {} {}".format(timeSta, AKIEstIsNewStatisticsAvailable()))

    global aimsun_controller    #call main simulator function

    if AKIEstIsNewStatisticsAvailable():
        # AKIPrintString('New Interval')
        writeSectionInfo(time,timeSta,aimsun_controller.dir_names[G_DIR_OUTPUT])
        print('write statistics at {}'.format(timeSta))
    # if AKIEstIsNewStatisticsAvailable():
    #     aimsun_controller.process_travel_time_statistics(timeSta)
    aimsun_controller.timeTrigger(timeSta, time)
    # # LOG.debug("here2")
    # if statistic_gathered:
    #     global last_statistic_fetched
    #     if timeSta - last_statistic_fetched >= statistic_interval:
    #         # LOG.debug("fetching statistics at {}".format(timeSta))
    #         if AKIEstIsNewStatisticsAvailable():
    #             LOG.warning("statistics have been reset when reading statistics!")
    #         sec_data, turn_data = readNewNetworkStatistics(timeSta)
    #         aimsun_controller.setNewTravelTimeData(sec_data, turn_data, timeSta)
    #         last_statistic_fetched = np.math.floor(timeSta)
    #         # LOG.debug("new last statistic fetched: {}".format(last_statistic_fetched))
    # if not AKIEstIsNewStatisticsAvailable():    #update travel times in external network
    #     # LOG.debug("here3")
    #     # LOG.debug("__________________")
    #     # LOG.debug("NEW STATISTICS!!! {}".format(timeSta))
    #     # LOG.debug("__________________")
    #     # LOG.debug("interval of statistics {}".format(AKIEstGetIntervalStatistics() ) )
    #     # LOG.debug("is gathering? {}".format(AKIIsGatheringStatistics() ) )
    #     # LOG.debug("here4")
    #     sec_data, turn_data = readNewNetworkStatistics(timeSta)
    #     # LOG.debug("here5")
    #     aimsun_controller.setNewTravelTimeData(sec_data, turn_data, timeSta)

    return 0

def AAPIFinish():
    global aimsun_controller    #call main simulator function
    aimsun_controller.end()
    global profiler_enabled
    if profiler_enabled:
        global profiler
        profiler.disable()
        s = io.StringIO()
        #sortby = SortKey.CUMULATIVE
        ps = pstats.Stats(profiler, stream=s).sort_stats('cumtime') #tottime cumtime
        ps.print_stats(50)
        print(s.getvalue())
    return 0


def AAPIUnLoad():
    return 0


def AAPIPreRouteChoiceCalculation(time, timeSta):
    return 0


def AAPIEnterVehicle(idveh, idsection):
    return 0

def AAPIExitVehicle(idveh, idsection):
    global aimsun_controller
    aimsun_controller.triggerVehExitsSystem(idveh, idsection)
    return 0


def AAPIEnterPedestrian(idPedestrian, originCentroid):
    return 0



def AAPIExitPedestrian(idPedestrian, destinationCentroid):
    return 0


def AAPIEnterVehicleSection(idveh, idsection, atime):
    # check for vehicles entered new section for routing
    global aimsun_controller
    aimsun_controller.triggerVehEnterNextSection(idveh, atime, idsection)
    return 0


def AAPIExitVehicleSection(idveh, idsection, atime):
    # check for vehicles exited a section for write back
    global aimsun_controller
    aimsun_controller.triggerVehExitsSection(idveh, atime, idsection)
    return 0 

# ========================================================================================================= #
# FUNCTIONS NEEDED FOR FLEET SIMULATION #
# ========================================================================================================= #

def get_directory_dict(scenario_parameters):
    """
    This function provides the correct paths to certain data according to the specified data directory structure.
    :param scenario_parameters: simulation input (pandas series)
    :return: dictionary with paths to the respective data directories
    """
    study_name = scenario_parameters[G_STUDY_NAME]
    scenario_name = scenario_parameters[G_SCENARIO_NAME]
    network_name = scenario_parameters[G_NETWORK_NAME]
    demand_name = scenario_parameters[G_DEMAND_NAME]
    zone_name = scenario_parameters.get(G_ZONE_SYSTEM_NAME, None)
    fc_type = scenario_parameters.get(G_FC_TYPE, None)
    fc_t_res = scenario_parameters.get(G_FC_TR, None)
    gtfs_name = scenario_parameters.get(G_GTFS_NAME, None)
    infra_name = scenario_parameters.get(G_INFRA_NAME, None)
    #
    dirs = {}
    dirs[G_DIR_MAIN] = TUM_FLEET_SIMULATION_ABS_PATH
    dirs[G_DIR_DATA] = os.path.join(dirs[G_DIR_MAIN], "data")
    dirs[G_DIR_OUTPUT] = os.path.join(dirs[G_DIR_MAIN], "studies", study_name, "results", scenario_name)
    dirs[G_DIR_NETWORK] = os.path.join(dirs[G_DIR_DATA], "networks", network_name)
    dirs[G_DIR_VEH] = os.path.join(dirs[G_DIR_DATA], "vehicles")
    dirs[G_DIR_FCTRL] = os.path.join(dirs[G_DIR_DATA], "fleetctrl")
    dirs[G_DIR_DEMAND] = os.path.join(dirs[G_DIR_DATA], "demand", demand_name, "matched", network_name)
    if zone_name is not None:
        dirs[G_DIR_ZONES] = os.path.join(dirs[G_DIR_DATA], "zones", zone_name, network_name)
        if fc_type is not None and fc_t_res is not None:
            dirs[G_DIR_FC] = os.path.join(dirs[G_DIR_DATA], "demand", demand_name, "aggregated", zone_name, str(fc_t_res))
    if gtfs_name is not None:
        dirs[G_DIR_PT] = os.path.join(dirs[G_DIR_DATA], "pubtrans", gtfs_name)
    if infra_name is not None:
        dirs[G_DIR_INFRA] = os.path.join(dirs[G_DIR_DATA], "infra", infra_name, network_name)
    return dirs

def get_centroid_connections():
    """reads from network static information which section is connected directly or indirectly (via connceted node) to a centroid
    and other centroid connection information (TODO) """
    section_to_target_centroid_id = {}  # sections that might be targeting a centroi
    aimsun_id_to_direct_target_centroid = {}    # section_id/turn_id that are directly connected to a centroid, which is a target
    aimsun_id_to_direct_orign_centroid = {}     # section_id/turn_id that are directly connected to a centroid, which is an origin
    number_centroids = AKIInfNetNbCentroids()
    for i in range(number_centroids):
        cen_id = AKIInfNetGetCentroidId(i)
        cen = AKIInfNetGetCentroidInf(cen_id)
        n_dest = cen.NumConnecTo
        n_org = cen.NumConnecFrom
        con_dest = []
        con_org = []
        for j in range(n_dest):
            x = boolp()
            connector_id = AKIInfNetGetIdObjectANGofOriginCentroidConnector(cen_id, j, x)
            aimsun_id_to_direct_target_centroid[connector_id] = cen_id
            if x.value():   #connected to section
                sec = connector_id
                section_to_target_centroid_id[sec] = cen_id
            else:   #connected to node
                n_turns = AKIInfNetGetNbTurnsInNode(connector_id)
                for k in range(n_turns):
                    dest_sec = AKIInfNetGetDestinationSectionInTurn(connector_id, k)
                    org_sec = AKIInfNetGetOriginSectionInTurn(connector_id, k)
                    section_to_target_centroid_id[dest_sec] = cen_id
                    section_to_target_centroid_id[org_sec] = cen_id
        for j in range(n_org):
            x = boolp()
            connector_id = AKIInfNetGetIdObjectANGofDestinationCentroidConnector(cen_id, j, x)
            aimsun_id_to_direct_orign_centroid[connector_id] = cen_id
    return section_to_target_centroid_id, aimsun_id_to_direct_orign_centroid, aimsun_id_to_direct_target_centroid

def readNewNetworkStatistics(timeSta):
    """ reads section and turn statistics to update travel times in external network
    :param timeSta: current simulation time in aimsun
    :return: tuple (sec_data, turn_data) ; sec_data: dict aimsun_id -> avg_travel_time; turn_data: dict turn_id -> avg_travel_time
    """
    #section meassured travel times
    number_sections = AKIInfNetNbSectionsANG()
    sec_data = {}
    # LOG.debug("read network statistics: {}".format(timeSta))
    # LOG.debug("first sections")
    for i in range(number_sections):
        sec_id = AKIInfNetGetSectionANGId(i)
        sec = AKIEstGetCurrentStatisticsSection(sec_id, 0)
        #sec = AKIEstGetParcialStatisticsSection(sec_id, timeSta, 0 )
        tt = sec.TTa
        ttdv = sec.TTd
        ## LOG.debug(f"id  {sec_id}, {sec.Id}, {tt}, {ttdv}, {sec.report}")
        sec_data[sec_id] = tt
        # if tt > 0:
        #     # LOG.debug(f"sec {sec_id} : tt {tt} | {ttdv}")
    #turn meassured travel times
    number_turns = AKIInfNetNbTurns()
    turn_data = {}
    # LOG.debug("now turns")
    for i in range(number_turns):
        # LOG.debug("ick")
        turn_id = AKIInfNetGetTurnId(i)
        # LOG.debug("ick")
        try:
            turn = AKIInfNetGetTurnInf(turn_id)
            origin = turn.originSectionId
            destination = turn.destinationSectionId
            # LOG.debug("asdf")
            turn_dat = AKIEstGetCurrentStatisticsTurning(origin, destination, 0)
            # LOG.debug("asdf")
            if turn_dat.report != 0:
                LOG.warning("error getting statistic for {}".format(turn_id))
            #turn_dat = AKIEstGetParcialStatisticsTurning(origin, destination, timeSta, 0) 
            tt = turn_dat.TTa
            ttdv = turn_dat.TTd
            # LOG.debug(f"id  {sec_id}, {turn_id}, {tt}, {ttdv}, {turn_dat.report}")
        except:
            LOG.error("error reading turn {}".format(turn_id))
            tt = -1
        turn_data[turn_id] = tt
        #if tt > 0:
            # LOG.debug(f"turn {turn_id} : tt {tt}")
    return sec_data, turn_data

# ============================================================================================================== #
# API CLASS FOR CONTROLLING AIMSUN VEHICLES AND COMMUNICATION TO FLEET SIMULATION MODULE
# ============================================================================================================== #

class AimsunVehicleController():
    def __init__(self):
        #self.scenario_parameters = scenario_parameters

        self.server_ip = "localhost"
        self.server_port = 12345
        self.server_connection = socket.socket()
        # self.server_connection.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADRR, 1)
        self.server_connection.connect((self.server_ip, self.server_port))

        self.last_stat_report_time = datetime.datetime.now()

        self.dir_names = None
        self.log_f = None
        self.log_file = None
        self.routing_engine = None

        self.last_vid_id = 0
        self.active_aimsun_vids = {}    # aimsun_vid_id -> AimsunVehObj

        self.aimsun_vid_to_fs_opvid = {} # aimsun_vid -> (op_id, vid)
        self.fs_opvid_to_aimsun_vid = {} # (op_id, vid) -> aimsun_vid

        self.section_to_centroid_id = {}    # connection aimsun seciton id -> centroid id
        self.centroid_ids = {}  # centroid id -> 1
        self.centroid_id_list = []  # list of centroid ids (order can be shuffled)
        self.aimsun_id_to_direct_orign_centroid = {}    # connection aimsun object id (section_turn) -> origin centroid
        self.aimsun_id_to_direct_target_centroid = {}   # connection aimsun object id(section/turn) -> destination centroid

        self.vehicles_reached_destination = {} # vids_reached_destination: dict (opid, vid) -> list of (passed_aimsun_sections, time_of_passing)

        self.vid_virtual_queue = {} # try to create aimsun vehicles again # (op_id, vid) -> (current_pos, node_list)
        self.fs_id_to_allready_driven = {} #if a vehicle has been deleted unintentionally -> reset with a new one

        self.last_int_timetrigger = -1
        self.sim_start_time = None  #will be set with first time trigger

        self.vehicles_entered_or_exit_section = {} #dict of veh_id -> list of (flag, veh_id, time, sec) since last time step, flag = +1 enteredSection, -1 exitSection
        self.request_another_vehicle_update = {}    #dict of (flag, veh_id) -> 1 -> call new vehicle actions (that werent possible/sucesful last step) | flag = 1: another next setion setting; flag = 2: another lange change call
        # find a vehicle type in aimsun
        # TODO this would have to be in every simulation
        # note no dublicate names in aimsun possible!
        self.veh_type_pos = 1
        self.gathering_statistics_interval = None

        self.new_travel_time_data_available = False # flag to indicate if new travel time data from the network is here
        self.new_sec_travel_time_data = {}  # filled if travel_time_data is here aimsun section id -> average travel time
        self.new_turn_travel_time_data = {} # filled if travel_time_data is here aimsun turn id -> avergage travel time

        # manually gathring travel time data because aimsun sucks
        self.aimsun_vehicle_entered_section = {}    # aimsun_vehicle_id -> sec, time
        self.aimsun_vehicle_exit_section = {}   # aimsun_vehicle_id -> sec, time

        self.section_id_to_travel_time_list = {}    # aimsun_section_id -> list of travel_times for passing
        self.turn_to_travel_time_list = {}       # (aimsun_origin_section_id, aimsun_destination_section_id) -> list of travel times for passing

        self.pv_requests_to_start_from_last_step = []

    def format_object_and_send_msg(self, obj):
        """ sending objects via socket to fleet simulation
        !!! HEADERSIZE have to be the same on both sides !!!!
        :param obj: corresponding message to be sent (will be pickeled)
        """
        # json_content = json.dumps(obj)
        # msg = json_content + "\n"
        pickle_content = pickle.dumps(obj)
        re = pickle.loads(pickle_content)
        #msg = bytes(f"{len(msg):<{HEADERSIZE}}", 'utf-8')
        byte_msg = bytes(f"{len(pickle_content):<{HEADERSIZE}}", ENCODING) + pickle_content
        if LOG_COMMUNICATION:
            prt_str = f"sending: {obj} to {self.server_connection}\n" + "-" * 20 + "\n"
            with open(self.log_f, "a") as fhout:
                fhout.write(prt_str)
        #byte_msg = msg.encode(ENCODING)
        self.server_connection.send(byte_msg)

    def await_message(self):
        """ collects the data from socket an creates the message
        :return: corresponding message
        """
        if LOG_COMMUNICATION and self.log_f:
            prt_str = f"run server mode\n" + "-" * 20 + "\n"
            with open(self.log_f, "a") as fhout:
                fhout.write(prt_str)
        new_msg = True
        full_msg = b""
        await_response = True
        print("await message")
        while await_response:
            byte_stream_msg = self.server_connection.recv(RECV_SIZE)
            #print("await response. got {}".format(byte_stream_msg))
            time_now = datetime.datetime.now()
            if time_now - self.last_stat_report_time > datetime.timedelta(seconds=STAT_INT):
                self.last_stat_report_time = time_now
                if LOG_COMMUNICATION:
                    prt_str = f"time:{time_now}\ncurrent_msg:{full_msg}\nbyte_stream_msg:{byte_stream_msg}\n" \
                                + "-" * 20 + "\n"
                    with open(self.log_f, "a") as fhout:
                        fhout.write(prt_str)
            if not byte_stream_msg:
                continue

            if new_msg:
                #print("new msg len:",byte_stream_msg[:HEADERSIZE])
                msglen = int(byte_stream_msg[:HEADERSIZE])
                new_msg = False

            full_msg += byte_stream_msg

            if len(full_msg)-HEADERSIZE == msglen:
                print("full msg recvd")
                #print(full_msg[HEADERSIZE:])
                new_msg = True
                pickle_information = full_msg[HEADERSIZE:]
                full_msg = b""
                response_obj = pickle.loads(pickle_information)
                print(response_obj)
                return response_obj

    def end(self):
        self.save_vehicle_states()
        send_obj = {"type" : "end_of_simulation"}
        self.format_object_and_send_msg(send_obj)
        print("send {}".format(send_obj))
        self.server_connection.close()

    def set_external_init(self):
        """ waits for init attributes sent from fleet simulation
        """
        response_obj = self.await_message()
        if response_obj["type"] == "init":
            print("got init {}".format(response_obj))
            self.external_init(response_obj["scenario_parameters"])
        else:
            print("WRONG MESSAGE {}".format(response_obj))
            self.end()
        self.section_to_centroid_id, self.aimsun_id_to_direct_orign_centroid, self.aimsun_id_to_direct_target_centroid = get_centroid_connections()    # set centroid connects
        for cen_id in self.section_to_centroid_id.values():
            self.centroid_ids[cen_id] = 1
        self.centroid_id_list = list(self.centroid_ids.keys())
        self.load_vehicle_states()

    def external_init(self, scenario_parameters):
        """ sets addtional init attributes when recieved from fleet simulation module
        :param scenario_parameters: scenario parameter dictionary
        """
        self.scenario_parameters = scenario_parameters
        self.start_time = self.scenario_parameters[G_SIM_START_TIME]
        self.end_time = self.scenario_parameters[G_SIM_END_TIME]
        self.time_step = self.scenario_parameters[G_SIM_TIME_STEP]

        self.dir_names = get_directory_dict(scenario_parameters)
        #self.routing_engine = Network(self.dir_names[G_DIR_NETWORK])

        self.log_f = os.path.join(self.dir_names[G_DIR_OUTPUT], "01_socket_com.txt")
        with open(self.log_f, "w") as fhout:
            fhout.write("")
        self.log_file = os.path.join(self.dir_names[G_DIR_OUTPUT], f"aimsun_api.log")  # logging from this process into an extra file
        print("logging to file {}".format(self.log_file))
        with open(self.log_file, "w") as fhout:
            fhout.write("")
        LOG.level = logging.INFO
        logging.basicConfig(handlers=[logging.FileHandler(self.log_file), logging.StreamHandler()],
                            level=LOG.level, format='%(process)d-%(name)s-%(levelname)s-%(message)s')

        veh_type_name = scenario_parameters.get(G_AIMSUN_VEH_TYPE_NAME, "Car")
        self.veh_type_pos = -1
        for i in range(AKIVehGetNbVehTypes()):
            anyNonAsciiChar = boolp()
            x = AKIConvertToAsciiString(AKIVehGetVehTypeName(i), True, anyNonAsciiChar)
            #x = AKIVehGetVehTypeName(i)
            print("vehtype {} : {}".format(i, x))
            if x == veh_type_name:
                self.veh_type_pos = i
        if self.veh_type_pos < 0:
            print("VEHICLE TYPE {} NOT FOUND!".format(veh_type_name))
            self.veh_type_pos = 1

        vtype_aimsun_obj = model.getCatalog().findByName(veh_type_name)
        prt_str = "Found the following id for vehicle type {0}: {1}".format(veh_type_name, vtype_aimsun_obj.getId())
        vehTypePos = AKIVehGetVehTypeInternalPosition(vtype_aimsun_obj.getId())
        print(prt_str, vehTypePos)
        if vehTypePos > 0:
            self.veh_type_pos = vehTypePos

        self.gathering_statistics_interval = scenario_parameters.get(G_AIMSUN_STAT_INT, 3600)

    def get_not_touching_centroids(self, section_list, shuffle_needed = False):
        """ this function returns two aimsun centroids, that are not directly connected to the route
        (otherwise aimsun vehicle might be removed from simulation there)
        :param section_list: list of aimsun section_ids
        :return: two-tuple of two centroid ids
        """
        touched_centroids = {}
        for sec in section_list:
            cen = self.section_to_centroid_id.get(sec)
            if cen is not None:
                touched_centroids[cen] = 1
        cen_found = 0
        cen_1 = None
        cen_2 = None
        if shuffle_needed:
            np.random.shuffle(self.centroid_id_list)
        for cen in self.centroid_id_list:
            if touched_centroids.get(cen) is None:
                if cen_found == 0:
                    cen_1 = cen
                    cen_found += 1
                elif cen_found == 1:
                    cen_2 = cen
                    cen_found += 1
                    break
        if cen_found != 2 or cen_1 == cen_2:
            LOG.warning("no 2 not touching centroids found for section list {}".format(section_list))
            if cen_found == 1 or cen_1 == cen_2:
                cen_2 = cen_1
                while cen_2 == cen_1:
                    cen_2 = np.random.choice(list(self.centroid_ids.keys()))
            if cen_found == 0:
                LOG.warning("not even 1 not touching centroid found!")
                cen_1 = np.random.choice(list(self.centroid_ids.keys()))
                cen_2 = cen_1
                while cen_2 == cen_1:
                    cen_2 = np.random.choice(list(self.centroid_ids.keys()))
        # LOG.debug("non touching centroids : {} | {}".format(cen_1, cen_2))
        return (cen_1, cen_2)

    def get_start_end_centroids(self, section_list):
        """ this function returns two aimsun centroids, that are not directly connected to the route
        (otherwise aimsun vehicle might be removed from simulation there)
        :param section_list: list of aimsun section_ids
        :return: two-tuple of two centroid ids
        """
        return (self.section_to_centroid_id.get(section_list[0], None), self.section_to_centroid_id.get(section_list[-1], None) )

    def _get_vehicle_position_updates(self):
        """ collects the current locations for all controlled active aimsun vehicles
        :return: dict (op_id, vid) -> aimsun_pos (section_id, rel_pos on section [0,1])
        """
        vid_to_pos = {}
        for aimsun_vid, veh_obj in self.active_aimsun_vids.items():
            #print(f"get veh position update {aimsun_vid} | {veh_obj}")
            vid = self.aimsun_vid_to_fs_opvid[aimsun_vid]
            pos = veh_obj.get_pos()
            vid_to_pos[vid] = pos
        return vid_to_pos

    def _get_vehicles_that_need_rerouting(self):
        to_reroute = []
        for aimsun_vid, veh_obj in self.active_aimsun_vids.items():
            if veh_obj.reroute_needed():
                to_reroute.append(self.aimsun_vid_to_fs_opvid[aimsun_vid])
        return to_reroute

    def _get_vids_reached_destination_stats(self):
        """ returns all vehicles the reached its destination since last call
        :return: dict (opid, vid) -> list of (passed_aimsun_sections, time_of_passing)
        """
        return self.vehicles_reached_destination

    def start_new_routes(self, new_routes_to_start, sim_time):
        """ this function is used to create new aimsun vehicles or change their routes if communicated from the fleet control
        :param new_routes_to_start: dict (op_id, vid) -> (current_aimsun_pos, section_list)
        :param sim_time: current simulation time
        """
        # (op_id, vid) -> (current_pos, node_list)
        self.vid_virtual_queue.update(new_routes_to_start)  # vehicle that coulnt be created in last time step (e.g. due to congestion) -> retry
        new_vid_virtual_queue = {}
        for vid, pos_new_route in self.vid_virtual_queue.items():
            # LOG.debug(f"start new route: {vid} -> {pos_new_route}")
            aimsun_vid = self.fs_opvid_to_aimsun_vid.get(vid, None) # get aimsun id
            if len(pos_new_route) == 2:
                pos, new_route = pos_new_route
                prev_driven_route = []
            else:
                pos, new_route, prev_driven_route = pos_new_route
            if aimsun_vid is not None:  # adopt current route
                # LOG.debug(" -> update route")
                veh_obj = self.active_aimsun_vids[aimsun_vid]
                succesfull = veh_obj.assign_new_route(new_route)
                if not succesfull:
                    self.request_another_vehicle_update[ (1, aimsun_vid) ] = 1
            else:
                # LOG.debug("create new vehicle")
                if len(new_route) == 0:
                    # LOG.debug("unassigned before creation")
                    self.vehicles_reached_destination[vid] = prev_driven_route
                    continue
                new_aimsun_veh = AimsunVehicle(self, pos, new_route, veh_type=self.veh_type_pos, driven_route=prev_driven_route)    # create the vehicle in aimsun
                if new_aimsun_veh.is_created_in_aimsun():   # succesfull
                    aimsun_id = new_aimsun_veh.get_id()
                    # LOG.debug(" -> is created -> id {}".format(aimsun_id))
                    self.active_aimsun_vids[aimsun_id] = new_aimsun_veh # add the vehicle to the dictionaries for tracking
                    self.fs_opvid_to_aimsun_vid[vid] = aimsun_id
                    self.aimsun_vid_to_fs_opvid[aimsun_id] = vid
                    # LOG.debug(self.active_aimsun_vids)
                    # LOG.debug(self.fs_opvid_to_aimsun_vid)
                    # LOG.debug(self.aimsun_vid_to_fs_opvid)
                    if self.fs_id_to_allready_driven.get(vid) is not None:
                        driven_route = self.fs_id_to_allready_driven[vid]
                        LOG.warning("an allready deleted aimsun vid is reacreated ! {} : {} | {}".format(vid, aimsun_id, driven_route))
                        new_aimsun_veh.driven_route = driven_route[:]
                        del self.fs_id_to_allready_driven[vid]
                else: # the vehicle could not be created in aimsun  # these vehicles also need to return a position if queried! TODO
                    new_vid_virtual_queue[vid] = pos_new_route
                    # LOG.debug(" -> didnt work -> retry next time")
        self.vid_virtual_queue = new_vid_virtual_queue

    def timeTrigger(self, simulation_time, rel_sim_time):
        """ this is the main simulation for communication with the fleet simulation
        the new simulation time is set, aimsun vehicle that reached their destination are sent to the fleet simulation,
        new routing tasks are collected from the fleet simulation
        :param simulation_time: current simulation time
        :param rel_sim_time: simulationtime relative from start of simulation (always starts at 0)
        """

        t = int(simulation_time)
        if self.sim_start_time is None:
            self.sim_start_time = t
        # LOG.debug(f"timeTrigger: {simulation_time} -> int time {t} | last time {self.last_int_timetrigger}")
        self.resolveVehicleUpdateRequests()
        self.resolveSectionTriggers()   # update tracked vehicle states that entered or exited a section in aimsun
        ## LOG.debug("here")
        if t == self.last_int_timetrigger:  # aimsun trigger in subseconds; only communicate if the fleet simulation in integer seconds
            return
        else:
            print("timeTrigger at {}".format(t))
            if self.gathering_statistics_interval is not None and t % self.gathering_statistics_interval == 0:
                if t - self.sim_start_time >= self.gathering_statistics_interval:
                    self.process_travel_time_statistics(t, rel_sim_time)
            try:
                # LOG.debug("current vids")
                # for veh in self.active_aimsun_vids.values():
                #     ## LOG.debug(str(veh))
                #     if veh.last_section_trigger > 0:
                #         if rel_sim_time - veh.last_section_trigger > 60.0:
                #             veh_obj = AKIVehTrackedGetInf(veh.vid)
                #             s = veh_obj.idSection
                #             if s < 0:
                #                 sec_from = veh_obj.idSectionFrom
                #                 sec_to = veh_obj.idSectionTo
                #                 LOG.warning("veh {} long time on node! {}s".format(veh, rel_sim_time - veh.last_section_trigger))
                self.last_int_timetrigger = t
                vids_reached_destination = self._get_vids_reached_destination_stats() # collect information from vehicles, that reached their destination and send to fleet simulation
                send_obj = {"type" : "time_trigger", "sim_time" : t, "vehicles_reached_destination" : vids_reached_destination, "new_travel_time_available" : self.new_travel_time_data_available}
                self.format_object_and_send_msg(send_obj)
                self.vehicles_reached_destination = {}
                ## LOG.debug("send {}".format(send_obj))
                wait_for_new_routes = True  # wait for answer
                while wait_for_new_routes:
                    response_obj = self.await_message()
                    if response_obj["type"] == "request_veh_pos_updates":   # a new optimisation step is called; detailed information of active vehicles needed
                        # LOG.debug(response_obj)
                        #["opid_vid_to_pos_dict"] # (op_id, vid) -> aimsun_pos
                        opid_vid_to_pos_dict = self._get_vehicle_position_updates()
                        veh_to_reroute = self._get_vehicles_that_need_rerouting()
                        send_obj = {"type" : "veh_pos_updates", "opid_vid_to_pos_dict" : opid_vid_to_pos_dict, "veh_to_reroute" : veh_to_reroute}
                        self.format_object_and_send_msg(send_obj)
                    elif response_obj["type"] == "request_additional_time_trigger": # this is called after an opt_step as buffer communication to maintain flow
                        send_obj = {"type" : "time_trigger", "sim_time" : t, "vehicles_reached_destination" : {} , "new_travel_time_available" : self.new_travel_time_data_available}
                        self.format_object_and_send_msg(send_obj)
                    elif response_obj["type"] == "request_new_travel_time_data":    # this is the answer query in case the flag self.new_travel_time_available has been True in the time_trigger message
                        # send_obj = {"type" : "new_travel_time_data", "new_section_data" : self.new_sec_travel_time_data, "new_turn_data" : self.new_turn_travel_time_data}
                        send_obj = {"type" : "new_travel_time_data"}
                        self.format_object_and_send_msg(send_obj)
                        self.new_travel_time_data_available = False
                        self.new_sec_travel_time_data = {}
                        self.new_turn_travel_time_data = {}
                    elif response_obj["type"] == "new_routes_to_start":     # recieve new routing tasks
                        new_routes_to_start = response_obj["new_routes_to_start"]   # (op_id, vid) -> route
                        self.start_new_routes(new_routes_to_start, t)
                        list_pv_requests_to_start = response_obj["list_pv_requests_to_start"]
                        self.start_pv_routes(list_pv_requests_to_start)
                        wait_for_new_routes = False
                    else:
                        # LOG.debug("recivieved unknown message or error : {}".format(response_obj))
                        self.end()
                        break
            except:
                error_str = traceback.format_exc()
                # LOG.debug(error_str)
                self.end()
                raise EnvironmentError(error_str)
    
    def triggerVehExitsSystem(self, idveh, idsection):
        """ triggered when a vehicle exits the simulation in aimsun (also non tracked vehicles!)
        currently not doing anything
        :param idveh: aimsun vehicle id
        :param idsection: aimsun section id
        """
        ## LOG.debug("vehicle exit system {}".format(idveh))
        try:
            del self.aimsun_vehicle_entered_section[idveh]
        except:
            pass
        try:
            del self.aimsun_vehicle_exit_section[idveh]
        except:
            pass
        if self.active_aimsun_vids.get(idveh) is not None:  # check if this is a tracked vehicle (fleet vehicle)
            ## LOG.debug(" -> is tracked vehicle")
            aimsun_veh = self.active_aimsun_vids[idveh]
            if len(aimsun_veh.route) != 0:
                pos = (aimsun_veh.route[0], 0.0)
                rest_route = aimsun_veh.route[:]
                driven_route = aimsun_veh.driven_route[:]
                fs_id = self.aimsun_vid_to_fs_opvid[idveh]
                LOG.warning(f"aimsun veh {idveh} | fs id {fs_id} deleted before reaching destination -> create new vid | route {aimsun_veh.route}")
                self.vid_virtual_queue[fs_id] = (pos, rest_route)
                try:
                    del self.active_aimsun_vids[idveh]
                except:
                    pass
                try:
                    del self.aimsun_vid_to_fs_opvid[idveh]
                except:
                    pass
                try:
                    del self.fs_opvid_to_aimsun_vid[fs_id]
                except:
                    pass
                self.fs_id_to_allready_driven[fs_id] = driven_route

    def triggerVehEnterNextSection(self, idveh, atime, idsection):
        """ triggered when a vehicle enters the new section in aimsun (also non tracked vehicles!)
        when this function is called, the vehicle is not within this section at excactly this time.
        therefore it is added to a buffer list (self.vehicles_entered_or_exit_section) to be treated in the next time step (resolveSectionTriggers)
        :param idveh: aimsun vehicle id
        :param atime: simulation time
        :param idsection: section id
        """
        ## LOG.debug("vehicle enter section {}".format(idveh))

        self.enter_section_stats_trigger(idveh, atime, idsection)

        self.aimsun_vehicle_entered_section[idveh] = (idsection, atime)
        if self.active_aimsun_vids.get(idveh) is not None: # check if this is a tracked vehicle (fleet vehicle)
            ## LOG.debug(" -> is tracked vehicle")
            try:
                self.vehicles_entered_or_exit_section[idveh].append( (+1, idveh, atime, idsection) )
            except:
                self.vehicles_entered_or_exit_section[idveh] = [ (+1, idveh, atime, idsection) ]

    def enter_section_stats_trigger(self, idveh, atime, idsection):
        exit_sec, exit_time = self.aimsun_vehicle_exit_section.get(idveh, (None, None))
        if exit_sec is None:
            pass
            ## LOG.debug("exiting of vehicle not registered! {} {} {}".format(idveh, atime, idsection))
        else:
            prev_tt, prev_N = self.turn_to_travel_time_list.get( (exit_sec, idsection), (0,0) )
            new_tt = prev_tt * prev_N/(prev_N + 1.0) + (atime - exit_time)/(prev_N + 1.0)
            new_N = prev_N +1
            self.turn_to_travel_time_list[(exit_sec, idsection)]  = (new_tt, new_N)
            # try:
            #     self.turn_to_travel_time_list[(exit_sec, idsection)].append( atime - exit_time )
            # except:
            #     self.turn_to_travel_time_list[(exit_sec, idsection)] = [atime - exit_time]
            del self.aimsun_vehicle_exit_section[idveh]
        ## LOG.debug("current turn time statistics: {}".format(self.turn_to_travel_time_list))

    def triggerVehExitsSection(self, idveh, atime, idsection):
        """ triggered when a vehicle exits the current section in aimsun (also non tracked vehicles!)
        when this function is called, the vehicle is not within this section at excactly this time.
        therefore it is added to a buffer list (self.vehicles_entered_or_exit_section) to be treated in the next time step (resolveSectionTriggers)
        :param idveh: aimsun vehicle id
        :param atime: simulation time
        :param idsection: section id
        """

        self.exit_section_stats_trigger(idveh, atime, idsection)

        if self.active_aimsun_vids.get(idveh) is not None: # check if this is a tracked vehicle (fleet vehicle)
            ## LOG.debug(" -> is tracked vehicle")
            try:
                self.vehicles_entered_or_exit_section[idveh].append( (-1, idveh, atime, idsection) )
            except:
                self.vehicles_entered_or_exit_section[idveh] = [ (-1, idveh, atime, idsection) ]

    def exit_section_stats_trigger(self, idveh, atime, idsection):
        ## LOG.debug("vehicle exit section {}".format(idveh))
        ent_sec, ent_time = self.aimsun_vehicle_entered_section.get(idveh, (None, None))
        if ent_sec is None:
            #LOG.warning("entering of vehicle not registered! {} {} {}".format(idveh, atime, idsection))
            pass
        else:
            if ent_sec != idsection:
                pass
                #LOG.warning("vehicle exited other section than entered? {} {} <-> {} {} {}".format(ent_sec, ent_time, idveh, atime, idsection))
            else:
                prev_tt, prev_N = self.section_id_to_travel_time_list.get( idsection, (0,0) )
                new_tt = prev_tt * prev_N/(prev_N + 1.0) + (atime - ent_time)/(prev_N + 1.0)
                new_N = prev_N +1
                self.section_id_to_travel_time_list[idsection]  = (new_tt, new_N)
                # try:
                #     self.section_id_to_travel_time_list[idsection].append( atime - ent_time )
                # except:
                #     self.section_id_to_travel_time_list[idsection] = [atime - ent_time]
            del self.aimsun_vehicle_entered_section[idveh]
        ## LOG.debug("current sec time statistics: {}".format(self.section_id_to_travel_time_list))

        self.aimsun_vehicle_exit_section[idveh] = (idsection, atime)

    def resolveSectionTriggers(self):
        """ this function is called within the time triggered to update information based on vehicles that entered or exited a section since the last time step
        it is checked wether vehicles reached their destination -> will be deleted from aimsun
            or if the need to be rerouted
        """
        # LOG.debug("resolveSectionTriggers")
        vid_to_repeat_triggers = {}
        just_deleted = {}
        for vid, section_triggers in self.vehicles_entered_or_exit_section.items():
            repeat_section_trigger = None   #for very small nodes (entry before exit)
            for i, entries in enumerate(section_triggers):
                flag, idveh, atime, idsection = entries
                # LOG.debug(f"{flag} | {idveh} | {atime} | {idsection}")
                if flag == 1:
                    aimsun_veh = self.active_aimsun_vids.get(idveh)
                    if aimsun_veh is None:
                        if just_deleted.get(idveh) is None:
                            LOG.warning("VEHICLE NOT HERE? {}".format(idveh))
                        continue
                    if i == len(section_triggers) - 1:# (atime == last_time and i == len(section_triggers) -2):
                        veh = AKIVehTrackedGetInf(idveh)
                        if veh.idSection < 0:
                            LOG.warning(" -> didnt enter yet {} -> repeat next time {} <-> {}".format(idveh, veh.idSection ,idsection))
                            #repeat_triggers.append( (flag, idveh, atime, idsection) )
                            vid_to_repeat_triggers[idveh] = (flag, idveh, atime, idsection)
                        else:
                            retry_needed = aimsun_veh.enterSection(atime + self.sim_start_time, idsection)    # retry needed for another lane change call
                            if retry_needed:
                                self.request_another_vehicle_update[ (2, idveh)  ] = 1
                    if (i == len(section_triggers) - 2 and abs(section_triggers[i+1][2] - atime) < 0.0001):
                        LOG.info("very small node crossed! vid {} {}".format(idveh, idsection))
                        repeat_section_trigger = entries
                elif flag == -1:
                    aimsun_veh = self.active_aimsun_vids.get(idveh)
                    if vid_to_repeat_triggers.get(idveh):
                        sec = vid_to_repeat_triggers[idveh][-1]
                        if sec == idveh:
                            LOG.info("remove repeated section trigger again! {} {} {}".format(idveh, sec, vid_to_repeat_triggers[idveh]))
                            del vid_to_repeat_triggers[idveh]
                    if aimsun_veh is not None:
                        reached_destination = aimsun_veh.exitSection(atime + self.sim_start_time, idsection)
                        if reached_destination: # vids_reached_destination: dict (opid, vid) -> list of (passed_aimsun_sections, time_of_passing)
                            # LOG.debug(f"delete vehicle {idveh}")
                            driven_route = aimsun_veh.get_driven_route()
                            vid = self.aimsun_vid_to_fs_opvid[idveh]
                            self.vehicles_reached_destination[vid] = driven_route
                            del self.active_aimsun_vids[idveh]
                            del self.aimsun_vid_to_fs_opvid[idveh]
                            del self.fs_opvid_to_aimsun_vid[vid]
                            just_deleted[idveh] = 1
                    #else:
                        # LOG.debug(f"aimsun veh {idveh} missing -> allready kicked in step before?")
            if repeat_section_trigger is not None:
                # LOG.debug("repeat section trigger: {}".format(repeat_section_trigger))
                flag, idveh, atime, idsection = repeat_section_trigger
                if flag == 1:
                    aimsun_veh = self.active_aimsun_vids.get(idveh)
                    if aimsun_veh is None:
                        if just_deleted.get(idveh) is None:
                            LOG.warning("VEHICLE NOT HERE? {}".format(idveh))
                        continue
                    veh = AKIVehTrackedGetInf(idveh)
                    if veh.idSection < 0:
                        LOG.warning(" -> didnt enter yet {} -> repeat next time {} <-> {}".format(idveh, veh.idSection ,idsection))
                        #repeat_triggers.append( (flag, idveh, atime, idsection) )
                        vid_to_repeat_triggers[idveh] = (flag, idveh, atime, idsection)
                    else:
                        retry_needed = aimsun_veh.enterSection(atime + self.sim_start_time, idsection)    # retry needed for another lane change call
                        if retry_needed:
                            self.request_another_vehicle_update[ (2, idveh) ] = 1
        self.vehicles_entered_or_exit_section = vid_to_repeat_triggers

    def resolveVehicleUpdateRequests(self):
        new_request_update = {}
        for flag, vehicle_id in self.request_another_vehicle_update.keys():
            # LOG.debug("resolve update request: {} -> {}".format(flag, vehicle_id))
            aimsun_veh = self.active_aimsun_vids.get(vehicle_id)
            if aimsun_veh is not None:
                if flag == 1:
                    if not aimsun_veh._set_next_section():
                        new_request_update[ (flag, vehicle_id) ] = 1
                if flag == 1:
                    retry_needed = aimsun_veh.adjust_vehicle_lane()
                    if retry_needed:
                        new_request_update[ (flag, vehicle_id) ] = 1
            else:
                LOG.warning("resolveVehicleUpdateRequests: could not find vehicle {} for request {}".format(vehicle_id, flag))
        self.request_another_vehicle_update = new_request_update

    def setNewTravelTimeData(self, sec_data, turn_data, timeSta):
        """ this function is called if new travel time data is available from aimsun
        this needs to be sent to the fleet simulation network to update network travel times
        sets flags and dicts to be sent in the next time step
        :param sec_data: dict aimsun section id -> avg travel time
        :param turn_data: dict aimsun turn id -> avg travel time
        :param timeSta: aimsun simulation time this function is called
        """
        self.new_travel_time_data_available = True
        self.new_sec_travel_time_data = sec_data
        self.new_turn_travel_time_data = turn_data

    def process_travel_time_statistics(self, current_time, absolute_simulation_time):
        """ calculates average travel times for each section and turn and resets data
        :param current time: time of simulation
        :param absolute_simulation_time: only meassure time of simulation time (absolute time = current_time - start_time) """
        # LOG.debug("process travel time statistics")

        # LOG.debug("congestion")
        global statistic_interval
        for idveh, sec_time in self.aimsun_vehicle_entered_section.items():
            sec_id, entered_time = sec_time
            if absolute_simulation_time - entered_time > statistic_interval:
                # LOG.debug("vehicle in congestion: {} {}".format(sec_id, absolute_simulation_time - entered_time))
                prev_tt, prev_N = self.section_id_to_travel_time_list.get( sec_id, (0,0) )
                new_tt = prev_tt * prev_N/(prev_N + 1.0) + (absolute_simulation_time - entered_time)/(prev_N + 1.0)
                new_N = prev_N +1
                self.section_id_to_travel_time_list[sec_id]  = (new_tt, new_N)
                # try:
                #     self.section_id_to_travel_time_list[sec_id].append( current_time - entered_time )
                # except:
                #     self.section_id_to_travel_time_list[sec_id] = [ current_time - entered_time ]

        ## LOG.debug("sections")
        travel_time_df_list = []

        number_sections = AKIInfNetNbSectionsANG()
        for i in range(number_sections):
            sec_id = AKIInfNetGetSectionANGId(i)
            tt = -1
            tt_data = self.section_id_to_travel_time_list.get(sec_id)
            if tt_data is not None:
                tt = tt_data[0] # np.mean(tt_data)
            ## LOG.debug("section {} : {}".format(sec_id, tt))
            self.new_sec_travel_time_data[sec_id] = tt
            travel_time_df_list.append( {"aimsun_id" : sec_id, "travel time" : tt, "is section" : True} )
        self.section_id_to_travel_time_list = {}

        ## LOG.debug("turns")
        number_turns = AKIInfNetNbTurns()
        for i in range(number_turns):
            turn_id = AKIInfNetGetTurnId(i)
            turn = AKIInfNetGetTurnInf(turn_id)
            origin = turn.originSectionId
            destination = turn.destinationSectionId
            tt = -1
            tt_data = self.turn_to_travel_time_list.get( (origin, destination) )
            if tt_data is not None:
                tt = tt_data[0] # np.mean(tt_data)
            ## LOG.debug("turn {} : {}".format(turn_id, tt) )
            self.new_turn_travel_time_data[turn_id] = tt
            travel_time_df_list.append( {"aimsun_id" : turn_id, "travel time" : tt, "is section" : False} )
        self.turn_to_travel_time_list = {}

        self.new_travel_time_data_available = True

        tt_file = os.path.join(self.dir_names[G_DIR_OUTPUT], "current_travel_times.csv")
        #print("store tts in file {}".format(tt_file))
        travel_time_df = pd.DataFrame(travel_time_df_list)
        travel_time_df.to_csv(tt_file, index=False)


    def start_pv_routes(self, list_pv_requests_to_start):
        """ this function generates background traffic vehicles (requests that are net served by the fleet sim)
        list pv_to_start: list aimsun section list
        """
        # LOG.debug("start_pv_routes : {}".format(list_pv_requests_to_start))
        new_pv_requests_to_start_from_last_step = []
        for sec_list in list_pv_requests_to_start + self.pv_requests_to_start_from_last_step:
            start_sec = sec_list[0]
            org_cen, dest_cen = self.get_start_end_centroids(sec_list)
            if org_cen is None or dest_cen is None:
                LOG.warning("no start or end centroid found? {} {} {}".format(org_cen, dest_cen, sec_list))
                continue
            vid = AKIEnterVehTrafficOD(start_sec, 1, org_cen, dest_cen, 0)
            if vid < 0:
                if vid == -7004: #not enough space -> try next step
                    new_pv_requests_to_start_from_last_step.append(sec_list)
                else:
                    LOG.warning("error in creating pv vehicle: {} {} {} {}".format(vid, org_cen, dest_cen, sec_list))
                continue
        self.pv_requests_to_start_from_last_step = new_pv_requests_to_start_from_last_step
        # for start_node, end_node in list_pv_requests_to_start:
        #     org_cen = self.aimsun_id_to_direct_orign_centroid[start_node]
        #     tar_cen = self.aimsun_id_to_direct_target_centroid[end_node]
        #     x = AKIGenerateArrivalTrafficOD(1, org_cen, tar_cen, 1)
        #     if x.report < 0:
        #         LOG.warning("couldnt create aimsun vehicle node {} -> {} (cen {} -> {}) report: {}".format(start_node, end_node, org_cen, tar_cen, x.report))

    def save_vehicle_states(self):
        aimsun_veh_states = []
        for aimsun_id, aimsun_veh in self.active_aimsun_vids.items():
            try:
                fs_id = self.aimsun_vid_to_fs_opvid[aimsun_id]
                pos = aimsun_veh.get_pos()
                driven_route = aimsun_veh.driven_route
                currently_boarding = aimsun_veh.currently_boarding
                route = aimsun_veh.route
                veh_state_dict = {"aimsun_id" : aimsun_id, "fs_id" : "{}".format(fs_id), "pos" : "{}|{}".format(pos[0], pos[1]), "currently_boarding" : currently_boarding,
                    "route" : ";".join([str(x) for x in route]), "driven_route" : ";".join(["{}|{}".format(x[0], x[1]) for x in driven_route])}
                aimsun_veh_states.append(veh_state_dict)
            except:
                LOG.warning("physical {} | {} could not be stored".format(aimsun_id, aimsun_veh))
        for fs_id, pos_rest_route in self.vid_virtual_queue.items():
            try:
                pos, route = pos_rest_route
                driven_route = self.fs_id_to_allready_driven.get(fs_id, [])
                currently_boarding = False
                aimsun_id = None
                veh_state_dict = {"aimsun_id" : aimsun_id, "fs_id" : "{}".format(fs_id), "pos" : "{}|{}".format(pos[0], pos[1]), "currently_boarding" : currently_boarding,
                    "route" : ";".join([str(x) for x in route]), "driven_route" : ";".join(["{}|{}".format(x[0], x[1]) for x in driven_route])}
                aimsun_veh_states.append(veh_state_dict)
            except:
                LOG.warning("virtual {} | {} could not be stored".format(fs_id, pos_rest_route))
        veh_stats_df = pd.DataFrame(aimsun_veh_states)
        x = self.dir_names[G_DIR_OUTPUT]
        state_path = os.path.join(x, "aimsun_init_state_from_last_simulation.csv")
        veh_stats_df.to_csv(state_path)

    def load_vehicle_states(self):
        x = self.dir_names[G_DIR_OUTPUT]
        state_path = os.path.join(x, "aimsun_init_state_from_last_simulation.csv")
        if os.path.isfile(state_path):
            print("load init file!", state_path, os.path.isfile(state_path))
            veh_stats_df = pd.read_csv(state_path)
            routes_to_start = {}
            for kew, row in veh_stats_df.iterrows():
                fs_id = row["fs_id"]
                aimsun_id = row["aimsun_id"]
                pos = row["pos"].split("|")
                pos = (int(pos[0]), float(pos[1]))
                currently_boarding = row["currently_boarding"]
                if type(row["route"]) != str:
                    route = []
                else:
                    route = row["route"].split(";")
                    route = [int(r) for r in route]
                if type(row["driven_route"]) != str:
                    driven_route = []
                else:
                    driven_route = row["driven_route"].split(";")
                    driven_route = [u.split("|") for u in driven_route]
                    driven_route = [(int(a), float(b)) for a, b in driven_route]

                x = AKIInfNetGetSectionANGInf(pos[0])
                if x.report < 0:
                    driven_route.append((pos[0], driven_route[-1][1]) )
                    route = route[1:]
                    if len(route) != 0:
                        pos = (route[0], 0.0)
                if len(route) == 0:
                    self.vehicles_reached_destination[fs_id] = driven_route
                else:
                    routes_to_start[fs_id] = (pos, route, driven_route)
            self.start_new_routes(routes_to_start, -1)

        



# =========================================================================================================
# vehicle class to control the aimsun vehicle
# =========================================================================================================

class AimsunVehicle():
    def __init__(self, aimsun_controler, start_loc, route_section_list, mode = "route", veh_type = 1, occupancy = 0, driven_route = []):
        """ class tracking a vehicle created in the aimsun simulation, stores driven routes and routes the aimsun vehicle
        :param aimsun_controler: AimsunVehicleController obj
        :param start_loc: vehicle position to start (TODO might be no longer necessary)
        :param route_section_list: list of aimsun section ids to drive
        :param mode: task identifier of routing task (route, rellocation, ...); only used for color coding in aimsun TODO use global definitions
        :param veh_type: int value of the entry of the corresponding vehicle type in aimsun database (1 is standard vehicle) -> defines visualisation but also driver model
        :param occupancy: current occupancy of vehicle (only needed for color coding in visualisation)
        :param driven_route: only used when the aimsun simulation is split up to reload the former state
        """ 
        # LOG.debug("init aimsun vehicle: {} {}".format(start_loc, route_section_list))
        self.aimsun_controler = aimsun_controler
        self.route = route_section_list # list aimsun sec_ids
        self.driven_route = driven_route  # list aimsun (sec_ids, time of section exit)
        self.current_position = start_loc   # (aimsun_sec_id, rel_pos in section)

        # create aimsun vehicle
        self.currently_boarding = False # might be usefull for on-street boarding (TODO)

        #AKIEnterVehTrafficOD(int asection, int vehTypePos, int idCentroidOrigin, int idCentroidDest, int tracking)
        end_cen, start_cen = self.aimsun_controler.get_not_touching_centroids(self.route)   # vehicles are not routed from centroid to centroid -> therefore centroids are used that are not directly connected to the current route (otherwise vehicle is deleted there in aimsun)
        # LOG.debug(" now the aimsun vehicle")
        self.vid = AKIEnterVehTrafficOD(self.route[0], veh_type, start_cen, end_cen, 1) # created tracked aimsun vehicle (not succesfull if id < 0)
        # LOG.debug(" -> result: {}".format(self.vid))
        if self.vid == -7006:   # no path between centroids found
            c = 0
            while self.vid == -7006:
                end_cen, start_cen = self.aimsun_controler.get_not_touching_centroids(self.route, shuffle_needed=True)   # vehicles are not routed from centroid to centroid -> therefore centroids are used that are not directly connected to the current route (otherwise vehicle is deleted there in aimsun)
                # LOG.debug(" now the aimsun vehicle")
                self.vid = AKIEnterVehTrafficOD(self.route[0], veh_type, start_cen, end_cen, 1) # created tracked aimsun vehicle (not succesfull if id < 0)
                # LOG.debug(" -> result: {}".format(self.vid))
                c += 1
                if c >= 3:
                    LOG.warning("vehicle couldnt be created after 10 tries because of invalid centroid configuration! on section {}".format(self.route[0]))
                    break
        if self.vid < 0:    # vehicle creation not succesfull
            if self.vid != -7004: #not enough space
                LOG.warning("unknown vehicle creation error {}".format(self.vid))
            return

        # it think this sets e.g. color attributes in simulation
        # ang = ANGConnVehGetGKSimVehicleId(self.vid)
        # ANGConnEnableVehiclesInBatch(True)
        # print("___")
        # print("ANG id : {}".format(ang))
        # print("__")
        # ang_Npassengers = ANGConnGetAttribute(AKIConvertFromAsciiString("Occupancy"))
        # # print("{}".format(ang_Npassengers))
        # # print("____")
        # if occupancy == 0:
        #     occupancy = -1
        # ANGConnSetAttributeValueInt(ang_Npassengers, ang, occupancy)
        # val = ANGConnGetAttributeValueInt(ang_Npassengers, ang)
        # print("val {}".format(val))

        self.next_section_status = 0    # setting for next section not succesfull if this value < 0
        self.false_target_set = False   # TODO dont know (might be from old version)
        self._set_next_section()    # set the next target setion of the vehicle

        self.rerouting_needed = False
        self.last_section_trigger = -1

    def __str__(self):
        veh_obj = AKIVehTrackedGetInf(self.vid)
        s = f"veh {self.vid} report {veh_obj.report} at section {veh_obj.idSection} at lane {veh_obj.numberLane} | or turn {veh_obj.idSectionFrom} -> {veh_obj.idSectionTo}"
        return s

    def is_created_in_aimsun(self):
        """ check if this vehicle has been created in aimsun
        :return: True if created, False if not
        """
        if self.vid < 0:
            return False
        else:
            return True

    def get_id(self):
        """ returns aimsun vehicle id
        :return: int vehicle id in aimsun
        """
        return self.vid

    def get_pos(self):
        """ get current aimsun vehicle position
        :return: tuple (section_id, fraction of section allready moved)
        """
        veh_info = AKIVehTrackedGetInf(self.vid)    # get vehicle info from aimsun
        current_sec_id = veh_info.idSection     # get current section
        if current_sec_id < 0:  # vehicle is not on section but on a turn
            target_sec = veh_info.idSectionTo   # set vehicle to the start of the target section instead
            # LOG.debug("vehicle currently not on section? {} -> {}".format(self.vid, current_sec_id))
            # LOG.debug(" -> take target section {}".format(target_sec))
            if target_sec < 0:
                raise NotImplementedError
            else:
                return (target_sec, 0.0)
        else:
            dis_from_start = veh_info.CurrentPos    # position from start of the section
            sec_info = AKIInfNetGetSectionANGInf(current_sec_id)    # get static information of section
            dis_sec = sec_info.length   # section length to calculate fractional positon on section (from 0 to 1)
            return (current_sec_id, dis_from_start/dis_sec) 

    def reroute_needed(self):
        if self.rerouting_needed:
            self.rerouting_needed = False
            return True
        else:
            return False

    def get_driven_route(self):
        """ returns the driven route of the vehicle
        :return: list of (section id, time of exiting the section)
        """
        return self.driven_route

    def _set_next_section(self):
        """ in case a vehicle entered the next section, a new section has to be set"""
        if self.next_section_status < 0:    #vehicle didnt get turn in last step -> recompute route TODO
            # this might happen if a routing update comes to late and a vehicle didnt get the turn
            LOG.warning("vehicle didnt get right turn! {}".format(self.vid))
            #raise NotImplementedError

        if len(self.route) > 1: # define next vehicle section
            # maybe the centroids have the be checked here in case vehicle went missing
            x = AKIVehTrackedModifyNextSection(self.vid, self.route[1]) # set the next section of the aimsun vehicle
            self.next_section_status = x
            # LOG.debug("set next section {}: {} -> {}".format(self.vid, self.route[1], x))
            if x < 0: # and x != -7014:    #routing failed
                veh = AKIVehTrackedGetInf(self.vid)
                sec = veh.idSection
                if sec < 0:
                    # LOG.debug("_set next section: vehicle not on section!")
                    return False    #request a retry
                else:
                    LOG.error("error setting next section {}: {} | {} -> {} | {} {}".format(self.vid, x, self.route[0], self.route[1], type(self.route[0]), type(self.route[1])))
                    LOG.error("veh current section: {} {}".format(sec, veh.report))
                    self.reroute(sec)
                    #raise NotImplementedError
            else:
                if len(self.route) > 2: # try to set target lane
                    veh = AKIVehTrackedGetInf(self.vid)
                    poss_start_lane = AKIInfNetGetTurningOriginFromLane(self.route[1], self.route[2]) # first lane of current section to reach target section
                    poss_end_lane = AKIInfNetGetTurningOriginToLane(self.route[1], self.route[2]) # last lane of current section to reach target section
                    # LOG.debug("try to set next target lane {} : {} -> {} ({}) : lane {}".format(self.vid, self.route[1], self.route[2], poss_start_lane, poss_end_lane))
                    x = AKIVehTrackedModifyNextTargetLaneInNextSection(self.vid, self.route[1], poss_start_lane)
                    if x < 0:
                        LOG.warning(" -> didnt work {}".format(self.vid))
        return True

    def exitSection(self, time, sec):
        """ a vehicle exited a section: 
        - write back last section statistics
        - update current route plan
        - delete the vehicle if this is the last section in route plan
        - return True if vehicle reached its destination
        :param time: current simulation time
        :param sec: aimsun section id
        :return: True -> vehicle reached end of route; False otherwise
        """
        # LOG.debug(f"vehicle ExitSection {self.vid} {time} {sec} {self.route}")
        # LOG.debug(f"{self}")
        self.last_section_trigger = time
        if len(self.driven_route) == 0 or self.driven_route[-1][0] != sec:
            self.driven_route.append( (sec, time) )
        self.route = self.route[1:]
        if len(self.route) == 0:    # vehicle reached destination
            # LOG.debug(" -> removing")
            x = AKIVehTrackedRemove(self.vid)   # delete the vehicle from aimsun
            # LOG.debug(" -> vehicle is removed: {}".format(x))
            #if x < 0:
                # LOG.debug("removing vehicle went wrong: {} : code {}".format(self.vid, x))
            return True
        return False

    def enterSection(self, time, sec):
        """ check if vehicle is still on track sets the new target section
        :param time: current simulation time
        :param sec: current section
        """
        # LOG.debug("vehicle EnterSection {} {} {} {}".format(self.vid, time, sec, self.route))
        # LOG.debug(f"{self}")
        self.last_section_trigger = time
        if len(self.route) >=1 and not sec == self.route[0]:    # aimsun vehicle not on track
            LOG.info("vehicle not on route plan! {}: {} <> {} |{}".format(self.vid, sec, self.route[0], self.route))
            #self.printVehInfos()
            self.reroute(sec)

        #adjust target lane of current section for turn to next section
        retry_needed = False
        if self.next_section_status != -1:
            if len(self.route) > 1: # the lane has to be adopted manually to get the right turn (not happening by just setting next section) TODO refactor to _set_next_setion()?
                # check if lange change is needed
                retry_needed = self.adjust_vehicle_lane(current_section=sec)

                self._set_next_section()    # set the next section

            #if len(self.route) <= 1:    # a trigger for entering the last setion might be added here (e.g. second row boarding)
                # enter last section TODO
                # LOG.debug(" -> enter last section!")
        return retry_needed

    def adjust_vehicle_lane(self, current_section = None):
        veh = AKIVehTrackedGetInf(self.vid)
        if current_section is None:
            current_section = veh.idSection
            if current_section < 0:
                LOG.warning("adjust vehicle lane while not on section: {} {}".format(current_section, self.vid))
                return True
        poss_start_lane = AKIInfNetGetTurningOriginFromLane(current_section, self.route[1]) # first lane of current section to reach target section
        poss_end_lane = AKIInfNetGetTurningOriginToLane(current_section, self.route[1]) # last lane of current section to reach target section
        current_lane = veh.numberLane   # current vehicle lane
        # LOG.debug(f"vehicle {veh}")
        # LOG.debug("adjust lane needed? current: {} | needed {} - {}".format(current_lane, poss_start_lane, poss_end_lane))
        retry_needed = False
        if current_lane >= 0:
            if current_lane < poss_start_lane:  # move to right lane
                x = AKIVehTrackedModifyLane(self.vid, +1)
                if poss_start_lane - current_lane > 1:
                    # LOG.debug("MULTIPLE LANE CHANGES NEEDED! {}".format(current_section))
                    retry_needed = True
                if x < 0:
                    LOG.warning("AKITrackedModifyLane {} {}".format(self.vid,x))
                    retry_needed = True
            elif current_lane > poss_end_lane:  # move to left lane
                x = AKIVehTrackedModifyLane(self.vid, -1)
                if current_lane - poss_end_lane > 1:
                    # LOG.debug("MULTIPLE LANE CHANGES NEEDED! {}".format(current_section))
                    retry_needed = True
                if x < 0:
                    LOG.warning("AKITrackedModifyLane {} {}".format(self.vid,x))
                    retry_needed = True
        else:
            # LOG.debug("vehicle not on section? {} {}".format(veh.idSection, veh.numberLane))
            retry_needed = True
        return retry_needed

    def assign_new_route(self, route_section_list):
        """ this function is used to adopt the current route
        :param route_section_list: list of aimsun section ids
        """
        succesful = True
        if len(route_section_list) == 0:    # set route to nothing -> vehicle will be deleted once it reaches the end of the current section
            self.route = []
        else:
            self.route = route_section_list
            # LOG.debug("new_route: {}".format(self.route))
            cen1, cen2 = self.aimsun_controler.get_not_touching_centroids(route_section_list)  # update the target centroid of the vehicle (not touching the route!)
            veh = AKIVehTrackedGetStaticInf(self.vid)
            veh.centroidOrigin = cen2
            veh.centroidDest = cen1
            if len(self.route) > 0:
                veh.idsectionExit = self.route[-1]  # update endsection (i think this is not really needed TODO)
            res = AKIVehTrackedSetStaticInf(self.vid, veh)  # update the information of the vehicle
            #if res < 0:
                # LOG.debug("Error in setting new veh static information {}! Code {} route {}".format(self.vid, res, self.route))

            succesful = self._set_next_section()    # set the next section (can be not succesful, if vehicle is currently on turn)
        return succesful

    def reroute(self, start_sec):
        """ returns a new shortest path to the target """
        LOG.warning("rerouting within aimsun needed! {}".format(self.vid))
        self.rerouting_needed = True
        target_sec = self.route[-1]
        sectionCostColumn = None
        nbSections = AKIInfNetGetShortestPathNbSections( start_sec, target_sec, sectionCostColumn )
        if nbSections > 0:
            path = intArray( nbSections )
            result = AKIInfNetGetShortestPath( start_sec, target_sec, sectionCostColumn, path )
            if result < 0:
                LOG.error(f"couldnt create new route from {start_sec} -> {target_sec} | {nbSections} {result}")
                #raise EnvironmentError
            else:
                new_route = [path[i] for i in range(nbSections)]
                self.route = new_route
                cen1, cen2 = self.aimsun_controler.get_not_touching_centroids(self.route)
                veh = AKIVehTrackedGetStaticInf(self.vid)
                veh.centroidOrigin = cen2
                veh.centroidDest = cen1
                if len(self.route) > 0:
                    veh.idsectionExit = self.route[-1]  # update endsection (i think this is not really needed TODO)
                res = AKIVehTrackedSetStaticInf(self.vid, veh)  # update the information of the vehicle
                #if res < 0:
                    # LOG.debug("Error in setting new veh static information {}! Code {} route {}".format(self.vid, res, self.route))

                #self._set_next_section()
