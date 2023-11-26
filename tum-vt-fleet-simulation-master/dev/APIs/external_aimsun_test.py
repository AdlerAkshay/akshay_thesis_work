# -------------------------------------------------------------------------------------------------------------------- #
# standard distribution imports
# -----------------------------
import sys
import traceback
import importlib

# additional module imports (> requirements)
# ------------------------------------------
import pandas as pd

import os
import socket
#import json
import pickle
import datetime
import time

TUM_FLEET_SIMULATION_ABS_PATH = r'C:\Users\ge37ser\Documents\Coding\TUM_VT_FleetSimulation\tum-vt-fleet-simulation'

if TUM_FLEET_SIMULATION_ABS_PATH not in sys.path:
    sys.path.append(TUM_FLEET_SIMULATION_ABS_PATH)

from src.misc.globals import *
from src.routing.NetworkBasic import Network

STAT_INT = 60
ENCODING = "utf-8"
LOG_COMMUNICATION = True
LARGE_INT = 100000
RECV_SIZE = 1024
HEADERSIZE = 20


def loadScenarioParameters(constant_config_csv, scenario_csv, n_parallel_sim=1, n_cpu_per_sim=1, evaluate=1, bugfix=False,
                  keep_old=False):
    """
    This function combines constant study parameters and scenario parameters.
    Then it sets up a pool of workers and starts a simulation for each scenario.
    The required parameters are stated in the documentation.

    :param constant_config_csv: this file contains all input parameters that remain constant for a study
    :type constant_config_csv: str
    :param scenario_csv: this file contain all input parameters that are varied for a study
    :type scenario_csv: str
    :param n_parallel_sim: number of parallel simulation processes
    :type n_parallel_sim: int
    :param n_cpu_per_sim: number of cpus for a single simulation
    :type n_cpu_per_sim: int
    :param evaluate: 0: no automatic evaluation / != 0 automatic simulation after each simulation
    :type evaluate: int
    :param bugfix: removes try,except structure for single simulation if True
    :type bugfix: bool
    :param keep_old: does not start new simulation if result files are already available in scenario output directory
    :type keep_old: bool
    """
    # combine inputs
    scenario_df = pd.read_csv(scenario_csv, squeeze=False)
    constant_series = pd.read_csv(constant_config_csv, index_col=0, squeeze=True)
    for k,v in constant_series.items():
        # remove constant entries
        if k.startswith("#"):
            continue
        # scenario entry has priority
        if k in scenario_df.columns:
            continue
        scenario_df[k] = v
    scenario_df["n_cpu_per_sim"] = n_cpu_per_sim
    scenario_df["evaluate"] = evaluate
    scenario_df["bugfix"] = bugfix
    if keep_old:
        scenario_df["keep_old"] = keep_old
    # prepare for single/multi processing
    list_scenarios = [x[1] for x in scenario_df.iterrows()]
    return list_scenarios[0]

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
    gtfs_name = scenario_parameters.get(G_GTFS_NAME, None)
    #
    dirs = {}
    dirs[G_DIR_MAIN] = TUM_FLEET_SIMULATION_ABS_PATH# os.path.abspath(os.path.join(__file__, os.path.pardir, os.path.pardir))
    dirs[G_DIR_DATA] = os.path.join(dirs[G_DIR_MAIN], "data")
    dirs[G_DIR_OUTPUT] = os.path.join(dirs[G_DIR_MAIN], "results", study_name, scenario_name)
    dirs[G_DIR_NETWORK] = os.path.join(dirs[G_DIR_DATA], "networks", network_name)
    dirs[G_DIR_VEH] = os.path.join(dirs[G_DIR_DATA], "vehicles")
    dirs[G_DIR_DEMAND] = os.path.join(dirs[G_DIR_DATA], "demand", demand_name, "matched", network_name)
    if zone_name is not None:
        dirs[G_DIR_ZONES] = os.path.join(dirs[G_DIR_DATA], "zones", zone_name, network_name)
        if fc_type:
            dirs[G_DIR_FC] = os.path.join(dirs[G_DIR_DATA], "demand", demand_name, "aggregated", zone_name, fc_type)
    if gtfs_name is not None:
        dirs[G_DIR_PT] = os.path.join(dirs[G_DIR_DATA], "pubtrans", gtfs_name)
    return dirs

class AimsunBlufferVehicle():
    def __init__(self, vehicle_id, routing_engine, position, route_to_drive, sim_time):
        self.aimsun_vid = vehicle_id
        self.routing_engine = routing_engine
        self.pos = position
        self.route_to_drive = route_to_drive
        self.driven_route_with_times = [(self.pos[0], sim_time)]

    def move(self, last_sim_time, new_sim_time):
        if len(self.route_to_drive) == 0:
            return False
        remaining_step_time = new_sim_time - last_sim_time
        (new_pos, driven_distance, arrival_in_time_step, passed_nodes, passed_node_times) = \
            self.routing_engine.move_along_route(self.route_to_drive, self.pos, remaining_step_time,
                                                    sim_vid_id=self.aimsun_vid,
                                                    new_sim_time=new_sim_time,
                                                    record_node_times=True)
        last_node = self.pos[0]
        self.pos = new_pos
        if passed_nodes:
            self.driven_route_with_times.extend(list(zip(passed_nodes, passed_node_times)))
        for node in passed_nodes:
            self.route_to_drive.remove(node)
        if arrival_in_time_step == -1:
            return True
        else:
            #self.driven_route_with_times.append( (self.pos[0], arrival_in_time_step) )
            return False

    def assign_new_route(self, new_route_to_drive):
        if len(new_route_to_drive) == 0:
            print("WARNING: empty route assigned for aimsun_vid {}".format(self.aimsun_vid))
        elif len(self.route_to_drive) == 0:
            print(f"WARNING: current route allready emtpy: vid {self.aimsun_vid} : pos {self.pos} -> {self.route_to_drive} -> {new_route_to_drive}")
        elif self.route_to_drive[0] != new_route_to_drive[0]:
            print(f"ERROR: inconsistent routes to assign: vid {self.aimsun_vid} : pos {self.pos} -> {self.route_to_drive} -> {new_route_to_drive}")
            raise EnvironmentError
        self.route_to_drive = new_route_to_drive

    def get_pos(self):
        return self.pos
        

class AimsunBluffer():
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
        self.routing_engine = None

        self.last_vid_id = 0
        self.active_aimsun_vids = {}    # aimsun_vid_id -> AimsunVehObj

        self.aimsun_vid_to_fs_opvid = {} # aimsun_vid -> (op_id, vid)
        self.fs_opvid_to_aimsun_vid = {} # (op_id, vid) -> aimsun_vid

        self.vehicles_reached_destination = {}

    def format_object_and_send_msg(self, obj):
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
        if LOG_COMMUNICATION and self.log_f:
            prt_str = f"run server mode\n" + "-" * 20 + "\n"
            with open(self.log_f, "a") as fhout:
                fhout.write(prt_str)
        new_msg = True
        full_msg = b""
        await_response = True
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
                print("new msg len:",byte_stream_msg[:HEADERSIZE])
                msglen = int(byte_stream_msg[:HEADERSIZE])
                new_msg = False

            full_msg += byte_stream_msg

            if len(full_msg)-HEADERSIZE == msglen:
                print("full msg recvd")
                print(full_msg[HEADERSIZE:])
                new_msg = True
                pickle_information = full_msg[HEADERSIZE:]
                full_msg = b""
                response_obj = pickle.loads(pickle_information)
                return response_obj


    def end(self):
        send_obj = {"type" : "end_of_simulation"}
        self.format_object_and_send_msg(send_obj)
        print("send {}".format(send_obj))
        self.server_connection.close()

    def external_init(self, scenario_parameters):
        self.scenario_parameters = scenario_parameters
        self.start_time = self.scenario_parameters[G_SIM_START_TIME]
        self.end_time = self.scenario_parameters[G_SIM_END_TIME]
        self.time_step = self.scenario_parameters[G_SIM_TIME_STEP]

        self.dir_names = get_directory_dict(scenario_parameters)
        self.routing_engine = Network(self.dir_names[G_DIR_NETWORK])

        self.log_f = os.path.join(self.dir_names[G_DIR_OUTPUT], "01_socket_com.txt")
        with open(self.log_f, "w") as fhout:
            fhout.write("")

    def get_vehicle_position_updates(self):
        # (op_id, vid) -> aimsun_pos
        vid_to_pos = {}
        for aimsun_vid, veh_obj in self.active_aimsun_vids.items():
            vid = self.aimsun_vid_to_fs_opvid[aimsun_vid]
            pos = veh_obj.get_pos()
            vid_to_pos[vid] = pos
        return vid_to_pos

    def start_new_routes(self, new_routes_to_start, sim_time):
        # (op_id, vid) -> (current_pos, node_list)
        for vid, pos_new_route in new_routes_to_start.items():
            aimsun_vid = self.fs_opvid_to_aimsun_vid.get(vid, None)
            pos, new_route = pos_new_route
            if aimsun_vid is not None:
                veh_obj = self.active_aimsun_vids[aimsun_vid]
                veh_obj.assign_new_route(new_route)
            else:
                new_aimsun_veh = AimsunBlufferVehicle(vid, self.routing_engine, pos, new_route, sim_time)
                self.active_aimsun_vids[vid] = new_aimsun_veh
                self.fs_opvid_to_aimsun_vid[vid] = vid
                self.aimsun_vid_to_fs_opvid[vid] = vid

    def move_vehicles(self, last_time, new_time):
        # vids_reached_destination: dict (opid, vid) -> list of (passed_aimsun_sections, time_of_passing)
        vids_reached_destination = {}
        for aimsun_vid, veh in self.active_aimsun_vids.items():
            still_moving = veh.move(last_time, new_time)
            if not still_moving:    # reached destination
                passed_nodes_with_times = veh.driven_route_with_times[:]
                vid = self.aimsun_vid_to_fs_opvid[aimsun_vid]
                vids_reached_destination[vid] = passed_nodes_with_times
        for vid in vids_reached_destination.keys():
            aimsun_vid = self.fs_opvid_to_aimsun_vid[vid]
            del self.active_aimsun_vids[aimsun_vid]
            del self.aimsun_vid_to_fs_opvid[aimsun_vid]
            del self.fs_opvid_to_aimsun_vid[vid]
        return vids_reached_destination

    def run(self):
        response_obj = self.await_message()
        if response_obj["type"] == "init":
            print("got init {}".format(response_obj))
            self.external_init(response_obj["scenario_parameters"])
        else:
            print("WRONG MESSAGE {}".format(response_obj))
            self.end()
        last_time = self.start_time
        try:
            for t in range(self.start_time, self.end_time, self.time_step):
                # move vehicles -> send vehicles reached destination
                vehicles_reached_destination = self.move_vehicles(last_time, t)
                last_time = t

                # new_network_statistics
                #   -> recieve acceptance

                # time trigger
                #   -> request for vehicle locations ?
                #       -> send vehicle locations
                #   -> new_routes to start, no offer requests to start
                #["vehicles_reached_destination"] # vids_reached_destination: dict (opid, vid) -> list of (passed_aimsun_sections, time_of_passing)
                send_obj = {"type" : "time_trigger", "sim_time" : t, "vehicles_reached_destination" : vehicles_reached_destination}
                vehicles_reached_destination = {}
                self.format_object_and_send_msg(send_obj)
                print("send {}".format(send_obj))
                wait_for_new_routes = True
                while wait_for_new_routes:
                    response_obj = self.await_message()
                    if response_obj["type"] == "request_veh_pos_updates":
                        print(response_obj)
                        #["opid_vid_to_pos_dict"] # (op_id, vid) -> aimsun_pos
                        opid_vid_to_pos_dict = self.get_vehicle_position_updates()
                        send_obj = {"type" : "veh_pos_updates", "opid_vid_to_pos_dict" : opid_vid_to_pos_dict}
                        self.format_object_and_send_msg(send_obj)
                    elif response_obj["type"] == "request_additional_time_trigger":
                        send_obj = {"type" : "time_trigger", "sim_time" : t, "vehicles_reached_destination" : vehicles_reached_destination}
                        self.format_object_and_send_msg(send_obj)
                    elif response_obj["type"] == "new_routes_to_start":
                        new_routes_to_start = response_obj["new_routes_to_start"]   # (op_id, vid) -> route
                        self.start_new_routes(new_routes_to_start, t)
                        wait_for_new_routes = False
                    else:
                        print("recivieved unknown message or error : {}".format(response_obj))
                        self.end()
                        break
        except:
            error_str = traceback.format_exc()
            print(error_str)
            self.end()
            raise EnvironmentError(error_str)

        self.end()


if __name__ == "__main__":
    cc = r"scenarios\TestBMWstudyRP\constant_config.csv"
    sc = r"scenarios\TestBMWstudyRP\scenarios_0.csv"
    n_cpu_per_sim = 1
    b = AimsunBluffer()
    b.run()