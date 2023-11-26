import os
import socket
#import json
import pickle
import traceback
import datetime
import pandas as pd
import geopandas as gpd

from src.misc.globals import *

STAT_INT = 60
ENCODING = "utf-8" # has to be the same on both sides of the socket!
LOG_COMMUNICATION = True
LARGE_INT = 100000
RECV_SIZE = 1024
HEADERSIZE = 20 # has to be the same on both sides of the socket!

# TODO # think about use of global variables

### following function used to translate between (op_id, vid) tuple keys to strings and back (json cant serialize tuple keys)
def translateTupleDictsToStringKeys(cor_dict):
    return {f"{k[0]}|{k[1]}" : v for k, v in cor_dict.items()}

def getTupleFromStr(s):
    a, b = s.split("|")
    return (int(a), int(b))

def translateStringDictsToTupleKeys(cor_dict):
    return {getTupleFromStr(k) : v for k, v in cor_dict.items()}

def translateStringListEntriesToTupleKeys(str_list):
    return [getTupleFromStr(x) for x in str_list]

class NetworkInterfaceBase():
    def __init__(self, nw_path_used, uncontracted_base_nw_path = None):
        pass
    def convert_aimsun_position_to_network_position(self, aimsun_position):
        return aimsun_position
    def convert_network_route_to_aimsun_section_list(self, route, start_position):
        return route, start_position
    def convert_aimsun_passed_section_list_with_time(self, passed_section_list_with_time):
        return passed_section_list_with_time

class AimsunNetworkInterface():
    def __init__(self, nw_path_used, uncontracted_base_nw_path = None):
        """ this class is used to translate aimsun routes und positions to the network definitions in the fleet simulator
        and the other way around. all information should be stored in the network folder used in the fleet simulator.
        :param nw_path_used: path to the network folder used in routing engine
        :param uncontracted_base_nw_path: needed if the network has been preprocessed and 1to1 relations between edges and aimsun secitons is no longer given
        """
        self.edges_to_aimsun_ids = {}
        self.edges_to_length = {}
        self.aimsun_id_to_edge = {}
        self.stop_node_id_to_aimsun_node = {}
        edges_all_info_gdf = gpd.read_file(os.path.join(nw_path_used, "base", "edges_all_infos.geojson"))
        for key, entries in edges_all_info_gdf.iterrows():
            node_from = int(entries["from_node"])
            node_to = int(entries["to_node"])
            distance = float(entries["distance"])
            source_edge_str = entries["source_edge_id"]
            edge_id = (node_from, node_to)
            if type(source_edge_str) == str:
                aimsun_ids = [int(x) for x in source_edge_str.split("-")]
                self.edges_to_aimsun_ids[edge_id] = aimsun_ids
                for aimsun_id in aimsun_ids:
                    self.aimsun_id_to_edge[aimsun_id] = edge_id
                roadtype = entries["road_type"]
                if roadtype == "Stop_Connector":    # look at the preprocessing file for aimsun networks
                    if len(aimsun_ids) > 1:
                        print("WARNING THIS MIGHT NOT BE AN AIMSUN NODE")
                    self.stop_node_id_to_aimsun_node[node_from] = aimsun_ids[0]
                    self.stop_node_id_to_aimsun_node[node_to] = aimsun_ids[-1] #one of these nodes is no stop node, but i dont care

            self.edges_to_length[edge_id] = distance

        if uncontracted_base_nw_path is None:
            uncontracted_base_nw_path = nw_path_used
        base_edges_all_infos = gpd.read_file(os.path.join(uncontracted_base_nw_path, "base", "edges_all_infos.geojson"))
        self.aimsun_obj_to_length = {}
        self.aimsun_obj_to_base_travel_time = {}
        self.aimsun_obj_is_section = {}
        self.aimsun_orgin_section_destination_section_turn_id = {} #aimsun origin section -> aimsun_destination section -> aimsun_turn_id
        to_node = {}
        from_node = {}
        for key, entries in base_edges_all_infos.iterrows():
            source_edge_str = entries["source_edge_id"]
            if not type(source_edge_str) == str:
                print(f"WARNING: {source_edge_str} is not of type str! {type(source_edge_str)}")
                raise EnvironmentError
            try:
                aimsun_id = int(source_edge_str)
            except:
                print(f"not convertable to int (really the uncontracked base network?) -> {source_edge_str}")
                raise EnvironmentError
            length = float(entries["distance"])
            base_travel_time = float(entries["travel_time"])
            roadtype = entries["road_type"]
            is_section = True
            if roadtype == "Turn" or roadtype == "Stop_Connector":
                is_section = False
            else:
                to_node[entries["to_node"]] = aimsun_id
                from_node[entries["from_node"]] = aimsun_id
            self.aimsun_obj_to_length[aimsun_id] = length
            self.aimsun_obj_is_section[aimsun_id] = is_section
            self.aimsun_obj_to_base_travel_time[aimsun_id] = base_travel_time

        for key, entries in base_edges_all_infos[base_edges_all_infos["road_type"] == "Turn"].iterrows():
            source_edge_str = entries["source_edge_id"]
            if not type(source_edge_str) == str:
                print(f"WARNING: {source_edge_str} is not of type str! {type(source_edge_str)}")
                raise EnvironmentError
            try:
                aimsun_id = int(source_edge_str)
            except:
                print(f"not convertable to int (really the uncontracked base network?) -> {source_edge_str}")
                raise EnvironmentError

            start_sec = to_node[entries["from_node"]]
            end_sec = from_node[entries["to_node"]]
            try:
                self.aimsun_orgin_section_destination_section_turn_id[start_sec][end_sec] = aimsun_id
            except:
                self.aimsun_orgin_section_destination_section_turn_id[start_sec] = {end_sec : aimsun_id}

    def convert_aimsun_position_to_network_position(self, aimsun_position):
        """ 
        :param aimsun_position: (sec_id, rel pos on sec_id)
        :return: network position
        """
        aimsun_id, aimsun_rel_pos = aimsun_position
        edge_id = self.aimsun_id_to_edge[aimsun_id]
        edge_length = self.edges_to_length[edge_id]
        par_length = self.aimsun_obj_to_length[aimsun_id] * aimsun_rel_pos
        for other_edge_aimsun_id in self.edges_to_aimsun_ids[edge_id]:
            if other_edge_aimsun_id == aimsun_id:
                break
            par_length += self.aimsun_obj_to_length[other_edge_aimsun_id]
        return (edge_id[0], edge_id[1], par_length/edge_length)

    def convert_network_route_to_aimsun_section_list(self, route, start_position):
        """
        :param route: list of node ids
        :param start_position: network position in route
        :return: list of aimsun section ids, aimsun_start_position
        """
        #print("convert network route to aimsun sec list: {} {}".format(route, start_position))
        sec_list = []
        if len(route) == 0:
            return sec_list, None
        if route[0] != start_position[0]:
            #print(f"inconsistent route and pos? {start_position} | {route}")
            route = [start_position[0]] + route[:]
        aimsun_start_position = None
        for i in range(1, len(route)):
            edge_id = (route[i-1], route[i])
            aimsun_ids = self.edges_to_aimsun_ids[edge_id]
            #print(f"edge {edge_id} -> aimsun {aimsun_ids}")
            if i == 1 and start_position[2] is not None:
                all_length = self.edges_to_length[edge_id]
                par_length = 0
                #print("aimsun start position {}".format(start_position))
                for aimsun_id in aimsun_ids:
                    par_length += self.aimsun_obj_to_length[aimsun_id] 
                    #print(f"aimsun id {aimsun_id} length {par_length} frac lengt {par_length/all_length}")
                    if par_length/all_length >= start_position[2]:
                        if self.aimsun_obj_is_section[aimsun_id]:
                            sec_list.append(aimsun_id)
                            if aimsun_start_position is None:
                                l = self.aimsun_obj_to_length[aimsun_id]
                                prev_par_length = par_length - l
                                if prev_par_length/all_length >= start_position[2]:
                                    aimsun_start_position = (aimsun_id, 0.0)
                                else:
                                    frac_before = prev_par_length/all_length
                                    delta_length = (start_position[2] - frac_before) * all_length
                                    aimsun_start_position = (aimsun_id, delta_length/l)
                            #print("aimsun pos: {}".format(aimsun_start_position))
            else:
                sec_list.extend([aimsun_id for aimsun_id in aimsun_ids if self.aimsun_obj_is_section[aimsun_id]])
                #print(" -> sec_list {}".format(sec_list))
        if len(sec_list) == 0:
            #print("allready reached!")
            return None, None #only a turn to travel (considered as already reached from now on)
        else:
            if aimsun_start_position is None:
                aimsun_start_position = (sec_list[0], 0.0)
            #print(f"created sec {sec_list} form node route {route} at start {start_position} <-> {aimsun_start_position}")
            return sec_list, aimsun_start_position

    def convert_aimsun_passed_section_list_with_time(self, passed_section_list_with_time):
        """
        :param passed_section_list_with_time: list (section_id, time of passing)
        :return: list of (node_id, time of reach)
        """
        #print("convert_aimsun_passed_secionts")
        if len(passed_section_list_with_time) == 0:
            return []
        filled_passed_section_list_with_time = [] # fill the turns
        if len(passed_section_list_with_time) == 1:
            filled_passed_section_list_with_time = passed_section_list_with_time
        else:
            filled_passed_section_list_with_time.append(passed_section_list_with_time[0])
            for i in range(1, len(passed_section_list_with_time)):
                prev_sec_with_time = passed_section_list_with_time[i-1]
                cur_sec_with_time = passed_section_list_with_time[i]
                try:
                    turn_with_time = (self.aimsun_orgin_section_destination_section_turn_id[prev_sec_with_time[0]][cur_sec_with_time[0]], prev_sec_with_time[1])
                    filled_passed_section_list_with_time.append(turn_with_time)
                except KeyError:
                    print("couldnt find turn {} -> {}".format(prev_sec_with_time, cur_sec_with_time))
                    print("IGNORE FOR NOW!")
                filled_passed_section_list_with_time.append(cur_sec_with_time)
        #print("a", filled_passed_section_list_with_time)
        prev_edge_id = self.aimsun_id_to_edge[filled_passed_section_list_with_time[0][0]]
        prev_time = filled_passed_section_list_with_time[0][1]
        node_list_with_times = [(prev_edge_id, prev_time)]
        for section_id, time in filled_passed_section_list_with_time:
            edge_id = self.aimsun_id_to_edge[section_id]
            #print("b", section_id, edge_id)
            if edge_id != prev_edge_id: # there might be multiple sections for same edge; only want latest item
                node_list_with_times.append( (edge_id[-1], time) )
            else:
                node_list_with_times[-1] = (edge_id[-1], time)
            prev_edge_id = edge_id
            #print("c", node_list_with_times)
        return node_list_with_times

    def get_new_edge_travel_times(self, new_section_data, new_turn_data):
        """ this function convertes section and turn travel time data from aimsun to the corresponding
        edge travel times which can be interpreted in the routing engine
        if no statistics are aquired by aimsun (tt < 0) the flee flow travel time is assumed
        :param new_section_data: dict aimsun section id -> avg travel time
        :param new_turn_data: dict aimsun turn id -> avg travel time
        :return: dict edge_id (tuple (start_node_index, end_node_index)) -> avg travel time
        """
        new_edge_travel_times = {}
        for section_id, tt in new_section_data.items():
            edge_id = self.aimsun_id_to_edge.get(section_id)
            if edge_id is None:
                continue
            if tt < 0:
                tt = self.aimsun_obj_to_base_travel_time[section_id]
            try:
                new_edge_travel_times[edge_id] += tt
            except KeyError:
                new_edge_travel_times[edge_id] = tt
        for turn_id, tt in new_turn_data.items():
            edge_id = self.aimsun_id_to_edge.get(turn_id)
            if edge_id is None:
                continue
            if tt < 0:
                tt = self.aimsun_obj_to_base_travel_time[turn_id]
            edge_id = self.aimsun_id_to_edge[turn_id]
            try:
                new_edge_travel_times[edge_id] += tt
            except KeyError:
                new_edge_travel_times[edge_id] = tt
        return new_edge_travel_times

    def convert_node_tuple_to_aimsun_node_tuple(self, node_tuple):
        return (self.stop_node_id_to_aimsun_node[node_tuple[0]], self.stop_node_id_to_aimsun_node[node_tuple[1]])

class AimsunSocket:
    def __init__(self, fs_obj, init_status=0):
        """This method initiates the communcation socket between aimsun and the python fleet simulation module.
        this class is used as the communication module of the fleet simulation part
        can be coupled with:
            - AimsunAPI_BMWproject.py
        first the fleet simulation has to be started to created the socket server. the aimsun simulation has to be started shortly after.
        :param fs_obj: instance of FleetSimulation; fs_obj.scenario_parameters contains all scenario parameters
        :param init_status: not necessary currently (TODO)
        """
        self.server_ip = "localhost"    # this has to be the same on both side of the socket!
        self.server_port = 12345       # this has to be the same on both side of the socket!
        self.fleet_sim = fs_obj
        self.server_connection = socket.socket()    # create the socket server
        # self.server_connection.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADRR, 1)
        self.server_connection.bind((self.server_ip, self.server_port))
        self.client_connection = None
        self.init_status = init_status
        self.last_stat_report_time = datetime.datetime.now()
        # create communication log
        self.log_f = os.path.join(fs_obj.dir_names[G_DIR_OUTPUT], "00_socket_com.txt")
        with open(self.log_f, "w") as fh_touch:
            fh_touch.write(f"{self.last_stat_report_time}: Opening socket communication ...\n")

        # network interface class to translate network information from aimsun tho the fleetsimulation network
        print(fs_obj.dir_names)
        print(fs_obj.scenario_parameters[G_UNPROCESSED_NETWORK_NAME])

        uncontracted_base_nw_path = None
        if fs_obj.scenario_parameters.get(G_UNPROCESSED_NETWORK_NAME) is not None:
            uncontracted_base_nw_path = os.path.join(fs_obj.dir_names[G_DIR_DATA], "networks", fs_obj.scenario_parameters[G_UNPROCESSED_NETWORK_NAME])
        self.aimsun_network_interface = AimsunNetworkInterface(fs_obj.dir_names[G_DIR_NETWORK], uncontracted_base_nw_path=uncontracted_base_nw_path)

    def format_object_and_send_msg(self, obj):
        """ sends python objects to the connection client (aimsun api)
        :param obj: dictionary with message and attributes as string keys and python objects as values
        """
        pickle_content = pickle.dumps(obj)
        byte_msg = bytes(f"{len(pickle_content):<{HEADERSIZE}}", ENCODING) + pickle_content
        if LOG_COMMUNICATION:
            prt_str = f"sending: {obj} to {self.client_connection}\n" + "-" * 20 + "\n"
            with open(self.log_f, "a") as fhout:
                fhout.write(prt_str)
        #byte_msg = msg.encode(ENCODING)
        self.client_connection.send(byte_msg)

    def keep_socket_alive(self):
        """ once the socket communcation is established, this function is runnung as long the aimsun simulation is running
        communication with aimsun is handled here and corresponding fleet simulation functions are triggered
        """
        if LOG_COMMUNICATION:
            prt_str = f"run server mode\n" + "-" * 20 + "\n"
            with open(self.log_f, "a") as fhout:
                fhout.write(prt_str)
        full_msg = None
        current_msg = ""
        stay_online = True
        while stay_online:
            self.establish_connection()
            
            current_simulation_time = None
            vehicles_reached_destination_prev = None
            vehicles_allready_reached = {}
            await_response = True

            full_msg = b''
            new_msg = True
            #print("await message")
            while await_response: # listen to server connection
                #print("await response")
                byte_stream_msg = self.client_connection.recv(RECV_SIZE)
                #print("await response. got {}".format(byte_stream_msg))
                time_now = datetime.datetime.now()
                if time_now - self.last_stat_report_time > datetime.timedelta(seconds=STAT_INT):
                    self.last_stat_report_time = time_now
                    if LOG_COMMUNICATION:
                        prt_str = f"time:{time_now}\ncurrent_msg:{current_msg}\nbyte_stream_msg:{byte_stream_msg}\n" \
                                  + "-" * 20 + "\n"
                        with open(self.log_f, "a") as fhout:
                            fhout.write(prt_str)
                if not byte_stream_msg:
                    continue

                #msglen = 0
                if new_msg:
                    #print("new msg len:",byte_stream_msg[:HEADERSIZE])
                    msglen = int(byte_stream_msg[:HEADERSIZE])
                    new_msg = False

                full_msg += byte_stream_msg

                if len(full_msg)-HEADERSIZE == msglen:  # the new message is complete
                    #print("full msg recvd")
                    #print(full_msg[HEADERSIZE:])
                    new_msg = True
                    pickle_information = full_msg[HEADERSIZE:]
                    full_msg = b""
                    response_obj = pickle.loads(pickle_information) # get the python dicitonary

                    # control structure depending on type of message
                    if response_obj["type"] == "time_trigger":
                        # sets new time -> update boardings, sends aimsun vehicles that reached destination, additional communication if opt-step is called, new routes to start are computed, customer interaction
                        # this control structure is called twice in case the fleet sim requests the update of vehicles positions!
                        # TODO depending on model -> declined customers that use PV instead
                        try:
                            sim_time = response_obj["sim_time"]     # float in aimsun! but currently only ints are sent here
                            new_travel_time_data_available = response_obj["new_travel_time_available"]
                            vehicles_reached_destination = response_obj["vehicles_reached_destination"] # vids_reached_destination: dict (opid, vid) -> list of (passed_aimsun_sections, time_of_passing)
                            vehicles_reached_destination = translateStringDictsToTupleKeys(vehicles_reached_destination)
                            vehicles_reached_destination = self.translate_vehicles_reached_destination(vehicles_reached_destination)
                            for key, val in vehicles_allready_reached.items():  # routes that only consist of a single turn
                                pos, node_list = val
                                node_list_with_time = [(n, sim_time) for n in node_list]
                                vehicles_reached_destination[key] = node_list_with_time
                            vehicles_allready_reached = {}

                            if new_travel_time_data_available:  # request new travel_time information first
                                answer_obj = {"type" : "request_new_travel_time_data"}
                                vehicles_reached_destination_prev = vehicles_reached_destination    # store vehicle reached destination
                                self.format_object_and_send_msg(answer_obj)
                            else:
                                if sim_time != current_simulation_time and self.fleet_sim.request_vehicle_position_update_from_aimsun(sim_time):
                                    if vehicles_reached_destination and vehicles_reached_destination_prev is None:  # if travel time information was updated a new time_trigger has been requested with empty vehicle_reached_destination
                                        vehicles_reached_destination_prev = vehicles_reached_destination
                                    answer_obj = {"type" : "request_veh_pos_updates"}   # TODO # add op_id for multiple ops later?
                                    current_simulation_time = sim_time
                                    self.format_object_and_send_msg(answer_obj)
                                else:   # if request_veh_pos is called, time_trigger is called again from aimsun -> vehicles reached destination is not sent again
                                    if not vehicles_reached_destination and vehicles_reached_destination_prev is not None: # if travel time information was updated and/or an position update was called a new time_trigger has been requested with empty vehicle_reached_destination
                                        vehicles_reached_destination = vehicles_reached_destination_prev
                                        vehicles_reached_destination_prev = None
                                    new_routes_to_start, list_pv_requests_to_start = self.fleet_sim.step(sim_time, vids_reached_destination = vehicles_reached_destination)
                                    # new_routes_to_start: (op_id, vid) -> (current_pos, node_list)
                                    new_routes_to_start, vehicles_allready_reached = self.translate_new_routes_to_start(new_routes_to_start)
                                    answer_obj = {"type":"new_routes_to_start", "new_routes_to_start" : translateTupleDictsToStringKeys(new_routes_to_start), "list_pv_requests_to_start" : self.convert_routes_to_aimsun_routes(list_pv_requests_to_start) }  # (tuples not feasible as keys for json serialization)
                                    current_simulation_time = sim_time
                                    self.format_object_and_send_msg(answer_obj)
                        except:
                            error_str = traceback.format_exc()
                            error_obj = {"type":"fs_error", "content":error_str}
                            self.format_object_and_send_msg(error_obj)
                            raise EnvironmentError(error_str)
                    elif response_obj["type"] == "veh_pos_updates": # new reoptimisation all vehicle positions
                        try:
                            opid_vid_to_pos_dict = response_obj["opid_vid_to_pos_dict"] # (op_id, vid) -> aimsun_pos # TODO # for eg charging states, additional infos might be added here
                            opid_vid_to_pos_dict = translateStringDictsToTupleKeys(opid_vid_to_pos_dict)
                            opid_vid_to_pos_dict = self.translate_vehicle_pos_dict(opid_vid_to_pos_dict)
                            veh_to_reroute = response_obj["veh_to_reroute"]
                            veh_to_reroute = translateStringListEntriesToTupleKeys(veh_to_reroute)
                            self.fleet_sim.update_vehicle_information(opid_vid_to_pos_dict, veh_to_reroute)
                            answer_obj = {"type" : "request_additional_time_trigger"}
                            self.format_object_and_send_msg(answer_obj)
                        except:
                            error_str = traceback.format_exc()
                            error_obj = {"type":"fs_error", "content":error_str}
                            self.format_object_and_send_msg(error_obj)
                            raise EnvironmentError(error_str)
                    elif response_obj["type"] == "new_travel_time_data": # update travel time data
                        # send_obj = {"type" : "new_travel_time_data", "new_section_data" : self.new_sec_travel_time_data, "new_turn_data" : self.new_turn_travel_time_data}
                        try:
                            # new_section_data = response_obj["new_section_data"]
                            # new_turn_data = response_obj["new_turn_data"]
                            print("READ NEW TRAVEL TIMES!")
                            tt_file = os.path.join(self.fleet_sim.dir_names[G_DIR_OUTPUT], "current_travel_times.csv")
                            tt_df = pd.read_csv(tt_file, index_col="aimsun_id")
                            section_df = tt_df[tt_df["is section"]]
                            turn_df = tt_df[tt_df["is section"] == False]
                            new_section_data = section_df["travel time"].to_dict()
                            new_turn_data = turn_df["travel time"].to_dict()
                            new_edge_travel_times = self.aimsun_network_interface.get_new_edge_travel_times(new_section_data, new_turn_data)
                            self.fleet_sim.update_network_travel_times(new_edge_travel_times)
                            answer_obj = {"type" : "request_additional_time_trigger"}
                            self.format_object_and_send_msg(answer_obj)
                        except:
                            error_str = traceback.format_exc()
                            error_obj = {"type":"fs_error", "content":error_str}
                            self.format_object_and_send_msg(error_obj)
                            raise EnvironmentError(error_str)                    
                    elif response_obj["type"] == "end_of_simulation":
                        # end of simulation
                        print("The aimsun simulation has ended!")
                        print("Either a follow up aimsun simulation with the current ending initial state can be started or the simulation can be ended if the complete ODM-simulation time is simulated")
                        print("If a follow up aimsun simulation is called. Start the aimsun simulation first before you answer the console!")
                        print("Ensure that the correct initial state and simulation time is adjusts in aimsun or weird things might happen!")
                        print("...")
                        answered = False
                        while not answered:
                            try:
                                console_response = input("The aimsun simulation has ended. If a follow up aimsun-simulation is coming to complete the ODM-simulation type [y]! Otherwise press any other key!")
                                answered = True
                            except:
                                print("\n")
                        if console_response != "y":
                            await_response = False
                            stay_online = False
                        else:
                            self.establish_connection()
                    elif response_obj["type"] == "aimsun_error":
                        # error in aimsun simulation
                        await_response = False
                        stay_online = False
                    else:
                        # raise Warning/Error for unknown message
                        pass
        # shut down server connection at the end of the simulation
        self.server_connection.close()

    def establish_connection(self):
        print("stay online")
        self.server_connection.listen()
        self.client_connection, client_address = self.server_connection.accept()
        print("connection accepted")
        if LOG_COMMUNICATION:
            prt_str = f"{datetime.datetime.now()}: connection from :{client_address}\n" + "-" * 20 + "\n"
            with open(self.log_f, "a") as fhout:
                fhout.write(prt_str)
        #send init values
        init_obj = {"type": "init", "status": self.init_status, "scenario_parameters" : self.fleet_sim.scenario_parameters}
        self.format_object_and_send_msg(init_obj)
        print("init sent: {}".format(init_obj))

    def translate_vehicles_reached_destination(self, vehicles_reached_destination):
        # vids_reached_destination: dict (opid, vid) -> list of (passed_aimsun_sections, time_of_passing)
        new_vehicles_reached_destination = {}
        for key, list_secs_with_time in vehicles_reached_destination.items():
            list_nodes_with_times = self.aimsun_network_interface.convert_aimsun_passed_section_list_with_time(list_secs_with_time)
            new_vehicles_reached_destination[key] = list_nodes_with_times
        return new_vehicles_reached_destination

    def translate_new_routes_to_start(self, new_routes_to_start):
        # new_routes_to_start: (op_id, vid) -> (current_pos, node_list)
        translated_new_routes_to_start = {}
        vehicles_allready_reached = {}
        for key, val in new_routes_to_start.items():
            current_pos, node_list = val
            #print("translate routes : {} -> {} | {}".format(key, current_pos, node_list))
            aimsun_sec_list, aimsun_pos = self.aimsun_network_interface.convert_network_route_to_aimsun_section_list(node_list, current_pos)
            if aimsun_sec_list is not None:
                translated_new_routes_to_start[key] = (aimsun_pos, aimsun_sec_list)
            else:
                vehicles_allready_reached[key] = val
        return translated_new_routes_to_start, vehicles_allready_reached

    def translate_vehicle_pos_dict(self, vid_to_aimsun_pos):
        # (op_id, vid) -> aimsun_po
        vid_to_nw_pos = {}
        for vid, aimsun_pos in vid_to_aimsun_pos.items():
            nw_pos = self.aimsun_network_interface.convert_aimsun_position_to_network_position(aimsun_pos)
            vid_to_nw_pos[vid] = nw_pos
        return vid_to_nw_pos

    def convert_routes_to_aimsun_routes(self, list_routes):
        return_list = []
        for route in list_routes:
            if len(route) == 0:
                continue
            start_pos = (route[0], None, None)
            aimsun_sec_list, _ = self.aimsun_network_interface.convert_network_route_to_aimsun_section_list(route, start_pos)
            return_list.append(aimsun_sec_list)
        return return_list
            

if __name__ == "__main__":
    test = AimsunSocket(1)