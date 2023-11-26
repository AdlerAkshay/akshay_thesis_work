import os
import socket
import json
import traceback
import datetime
import pandas as pd

from src.misc.globals import *

STAT_INT = 60
ENCODING = "utf-8"
LOG_COMMUNICATION = False
LARGE_INT = 100000

# TODO # think about use of global variables
class MobiToppSocket:
    def __init__(self, fs_obj, init_status):
        """This method initiates the communcation socket between mobitopp and the python fleet simulation module.

        :param fs_obj: instance of FleetSimulation; fs_obj.scenario_parameters contains all scenario parameters
        """
        self.server_ip = fs_obj.scenario_parameters.get("HOST", "localhost")
        self.server_port = fs_obj.scenario_parameters["socket"]
        self.fleet_sim = fs_obj
        self.server_connection = socket.socket()
        # self.server_connection.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADRR, 1)
        self.server_connection.bind((self.server_ip, self.server_port))
        self.client_connection = None
        self.init_status = init_status
        self.last_stat_report_time = datetime.datetime.now()
        # create communication log
        self.log_f = os.path.join(fs_obj.dir_names[G_DIR_OUTPUT], "00_socket_com.txt")
        with open(self.log_f, "w") as fh_touch:
            fh_touch.write(f"{self.last_stat_report_time}: Opening socket communication ...\n")
        # TODO # load network translation dictionary
        node_trafo_f = os.path.join(fs_obj.dir_names[G_DIR_NETWORK], "visum_node_id_to_node_index.csv")
        tmp = pd.read_csv(node_trafo_f)
        vis2fs = tmp.set_index("visum_node_id")
        self.vis2fs = vis2fs["node_index"].to_dict()
        fs2vid = tmp.set_index("node_index")
        self.fs2vid = fs2vid["visum_node_id"].to_dict()
        # TODO # for intermodal connection
        # problem: len(self.vis_e2fs_n) = 905 | len(self.fs_n2vis_e) = 578 -> use local dictionaries
        edge_trafo_f = os.path.join(fs_obj.dir_names[G_DIR_NETWORK], "visum_edge_id_to_edge_index.csv")
        edge_trafo_df = pd.read_csv(edge_trafo_f, index_col=0)
        #ens = edge_trafo_df.apply(lambda x: int(x["edge_id"].split(";")[0]), axis=1)
        ens = edge_trafo_df.apply(lambda x : (x['visum_start_node_id'],x['visum_end_node_id']), axis = 1)
        self.vis_edge_to_start_node_index = {k : self.vis2fs[v[0]] for k, v in ens.items()}
        self.start_node_index_to_vis_edge = {v : k for k, v in self.vis_edge_to_start_node_index.items()}
        #self.vis_e2fs_n = ens.to_dict()
        # self.fs_n2vis_e = dict((y,x) for x,y in self.vis_e2fs_n.items())

        self.delayed_arrivals = {}  # arrival times with egress time -> list answer_dicts
        self.open_agent_request = None

    # # assuming visum node id
    # def from_visum_to_fs_nw(self, visum_node_id):
    #     """This method translates visum node id to fleet sim node id.
    #
    #     :param visum_node_id
    #     :return: fs_node_id
    #     """
    #     fs_node_id = self.vis2fs[visum_node_id]
    #     return fs_node_id
    #
    # def from_fs_to_visum_nw(self, fs_node_id):
    #     """This method translates fleet sim node id to visum node id.
    #
    #     :param fs_node_id
    #     :return: visum_node_id
    #     """
    #     visum_node_id = self.fs2vid[fs_node_id]
    #     return visum_node_id

    # assuming visum edge id

    def log_com(self, msg):
        with open(self.log_f, "a") as fhout:
            fhout.write(msg)

    def from_visum_to_fs_nw(self, visum_edge_id):
        """This method translates visum edge id to fleet sim node id.

        :param visum_edge_id
        :return: fs_node_id
        """
        fs_node_id = self.vis_edge_to_start_node_index.get(visum_edge_id, -1)
        if fs_node_id < 0:
            if LOG_COMMUNICATION:
                err_str = "couldnt find fs_node for visum_edge {}\n".format(visum_edge_id)
                self.log_com(err_str)
        return fs_node_id

    def from_fs_to_visum_nw(self, fs_node_id, local_node_edge_dict):
        """This method translates fleet sim node id to visum node id.

        :param fs_node_id
        :return: visum_node_id
        """
        visum_node_id = local_node_edge_dict[fs_node_id]
        return visum_node_id

    def format_object_and_send_msg(self, obj):
        json_content = json.dumps(obj)
        msg = json_content + "\n"
        if LOG_COMMUNICATION:
            prt_str = f"sending: {msg} to {self.client_connection}\n" + "-" * 20 + "\n"
            self.log_com(prt_str)
        byte_msg = msg.encode(ENCODING)
        self.client_connection.send(byte_msg)

    def keep_socket_alive(self):
        if LOG_COMMUNICATION:
            prt_str = f"run server mode\n" + "-" * 20 + "\n"
            self.log_com(prt_str)
        full_msg = None
        current_msg = ""
        # currently, Java connects twice to socket without sending a shutdown signal
        retry = True
        stay_online = True
        while stay_online:
            self.server_connection.listen()
            self.client_connection, client_address = self.server_connection.accept()
            if LOG_COMMUNICATION:
                prt_str = f"{datetime.datetime.now()}: connection from :{client_address}\n" + "-" * 20 + "\n"
                self.log_com(prt_str)
            #
            init_obj = {"type": "message", "content": f"init status: {self.init_status}"}
            self.format_object_and_send_msg(init_obj)
            #
            if retry:
                retry = False
                continue
            # TODO # think about error status != 0 in init
            await_response = True
            while await_response:
                # listen to server connection
                byte_stream_msg = self.client_connection.recv(1024)
                time_now = datetime.datetime.now()
                if time_now - self.last_stat_report_time > datetime.timedelta(seconds=STAT_INT):
                    self.last_stat_report_time = time_now
                    if LOG_COMMUNICATION:
                        prt_str = f"time:{time_now}\ncurrent_msg:{current_msg}\nbyte_stream_msg:{byte_stream_msg}\n" \
                                  + "-" * 20 + "\n"
                        self.log_com(prt_str)
                if not byte_stream_msg:
                    continue
                c_stream_msg = byte_stream_msg.decode(ENCODING)
                if "\n" in c_stream_msg:
                    tmp = c_stream_msg.split("\n")
                    full_msg = current_msg + tmp[0]
                    current_msg = tmp[1]
                    if LOG_COMMUNICATION:
                        prt_str = f"full_msg:{full_msg}\ncurrent_msg:{current_msg}\n" + "-"*20 + "\n"
                        self.log_com(prt_str)
                else:
                    full_msg = None
                    current_msg += c_stream_msg
                # full_msg can be longer than msg-len!!!
                if full_msg is not None:
                    # full message received
                    response_obj = json.loads(full_msg)
                    # control structure depending on type of message
                    if response_obj["type"] == "request_offer":
                        # new agent wants offer -> blocking call with response
                        try:
                            # unpack JSON message
                            agent_id = response_obj["agent_id"]
                            if self.open_agent_request is not None and self.open_agent_request != agent_id:
                                self.fleet_sim.user_response(self.open_agent_request, 0)
                            self.open_agent_request = agent_id
                            visum_origin_node_index = response_obj["origin"]
                            o_node = self.from_visum_to_fs_nw(visum_origin_node_index)
                            visum_destination_node_index = response_obj["destination"]
                            d_node = self.from_visum_to_fs_nw(visum_destination_node_index)
                            earliest_pickup_time = response_obj["time"]
                            number_passengers  = response_obj["nr_pax"]
                            # call fleet sim function
                            offer = self.fleet_sim.create_RP_offer(agent_id, o_node, d_node, simulation_time, earliest_pickup_time,
                                                                         number_passengers)
                            # pack JSON message
                            send_obj = {"type":"offer"}
                            send_obj["agent_id"] = agent_id
                            # send very high values for wait, drive and fare if no offer is made
                            if not offer.service_declined():
                                send_obj["t_access"] = int(offer.get(G_OFFER_ACCESS_W, 0))
                                send_obj["t_wait"] = int(offer[G_OFFER_WAIT])
                                send_obj["t_drive"] = int(offer[G_OFFER_DRIVE])
                                send_obj["t_egress"] = int(offer.get(G_OFFER_EGRESS_W, 0))
                                send_obj["fare"] = int(offer[G_OFFER_FARE])  # int, in Cent
                                send_obj["offer_id"] = int(offer[G_OFFER_ID])
                            else:
                                send_obj["t_access"] = LARGE_INT
                                send_obj["t_wait"] = LARGE_INT
                                send_obj["t_drive"] = LARGE_INT
                                send_obj["t_egress"] = LARGE_INT
                                send_obj["fare"] = LARGE_INT
                                send_obj["offer_id"] = 0
                            self.format_object_and_send_msg(send_obj)
                        except:
                            error_str = traceback.format_exc()
                            error_obj = {"type":"fs_error", "content":error_str}
                            self.format_object_and_send_msg(error_obj)
                            raise EnvironmentError(error_str)
                    elif response_obj["type"] == "first_mile_request_offer":
                        # new agent wants first mile offer -> blocking call with response
                        try:
                            # unpack JSON message
                            agent_id = response_obj["agent_id"]
                            if self.open_agent_request is not None and self.open_agent_request != agent_id:
                                self.fleet_sim.user_response(self.open_agent_request, 0)
                            self.open_agent_request = agent_id
                            visum_origin_node_index = response_obj["origin"]
                            o_node = self.from_visum_to_fs_nw(visum_origin_node_index)
                            d_nodes_with_pt_tt = []
                            visum_nodes = {}
                            for intermodal_request in response_obj["destinations"]:
                                visum_destination_node_index = intermodal_request["transfer"]
                                d_node = self.from_visum_to_fs_nw(visum_destination_node_index)
                                visum_nodes[d_node] = visum_destination_node_index
                                pt_tt = intermodal_request["time"]
                                d_nodes_with_pt_tt.append( (d_node, pt_tt) )
                            earliest_pickup_time = response_obj["time"]
                            number_passengers  = response_obj["nr_pax"]
                            # call fleet sim function
                            first_mile_offers = self.fleet_sim.create_first_mile_RP_offer(agent_id, o_node, d_nodes_with_pt_tt,
                                                                         simulation_time, earliest_pickup_time, number_passengers)
                            # pack JSON message
                            send_obj = {"type":"first_mile_offers"}
                            send_obj["agent_id"] = agent_id
                            # send very high values for wait, drive and fare if no offer is made
                            list_first_mile_offers = []
                            for offer in first_mile_offers:
                                first_mile_offer = {}
                                first_mile_offer["t_access"] = int(offer.get(G_OFFER_ACCESS_W, 0))
                                first_mile_offer["t_wait"] = int(offer[G_OFFER_WAIT])
                                first_mile_offer["t_drive"] = int(offer[G_OFFER_DRIVE])
                                first_mile_offer["t_egress"] = int(offer.get(G_OFFER_EGRESS_W, 0))
                                first_mile_offer["fare"] = int(offer[G_OFFER_FARE])  # int, in Cent
                                first_mile_offer["offer_id"] = int(offer[G_OFFER_ID])
                                first_mile_offer["objective"] = 0
                                first_mile_offer["destination"] = visum_nodes[offer[G_RQ_DESTINATION]]
                                list_first_mile_offers.append(first_mile_offer)
                            if len(first_mile_offers) == 0:
                                first_mile_offer = {}
                                first_mile_offer["t_access"] = LARGE_INT
                                first_mile_offer["t_wait"] = LARGE_INT
                                first_mile_offer["t_drive"] = LARGE_INT
                                first_mile_offer["t_egress"] = LARGE_INT
                                first_mile_offer["fare"] = LARGE_INT
                                first_mile_offer["offer_id"] = 0
                                first_mile_offer["objective"] = 0
                                first_mile_offer["destination"] = -1
                                list_first_mile_offers.append(first_mile_offer)
                            send_obj["offers"] = list_first_mile_offers
                            self.format_object_and_send_msg(send_obj)
                        except:
                            error_str = traceback.format_exc()
                            error_obj = {"type":"fs_error", "content":error_str}
                            self.format_object_and_send_msg(error_obj)
                            raise EnvironmentError(error_str)
                    elif response_obj["type"] == "last_mile_request_offer":
                        # new agent wants last mile offer -> blocking call with response
                        try:
                            # unpack JSON message
                            agent_id = response_obj["agent_id"]
                            if self.open_agent_request is not None and self.open_agent_request != agent_id:
                                self.fleet_sim.user_response(self.open_agent_request, 0)
                            self.open_agent_request = agent_id
                            visum_destination_node_index = response_obj["destination"]
                            d_node = self.from_visum_to_fs_nw(visum_destination_node_index)
                            o_nodes_with_pt_tt = []
                            visum_nodes = {}
                            for intermodal_request in response_obj["origins"]:
                                visum_origin_node_index = intermodal_request["transfer"]
                                o_node = self.from_visum_to_fs_nw(visum_origin_node_index)
                                pt_tt = intermodal_request["time"]
                                o_nodes_with_pt_tt.append( (o_node, pt_tt) )
                                visum_nodes[o_node] = visum_origin_node_index
                            number_passengers  = response_obj["nr_pax"]
                            # call fleet sim function
                            last_mile_offers = self.fleet_sim.create_last_mile_RP_offer(agent_id, o_nodes_with_pt_tt, d_node,
                                                                         simulation_time, number_passengers)
                            # pack JSON message
                            send_obj = {"type":"last_mile_offers"}
                            send_obj["agent_id"] = agent_id
                            # send very high values for wait, drive and fare if no offer is made
                            list_last_mile_offers = []
                            for offer in last_mile_offers:
                                last_mile_offer = {}
                                last_mile_offer["t_access"] = int(offer.get(G_OFFER_ACCESS_W, 0))
                                last_mile_offer["t_wait"] = int(offer[G_OFFER_PU_DELAY])
                                last_mile_offer["t_drive"] = int(offer[G_OFFER_DRIVE])
                                last_mile_offer["t_egress"] = int(offer.get(G_OFFER_EGRESS_W, 0))
                                last_mile_offer["fare"] = int(offer[G_OFFER_FARE])  # int, in Cent
                                last_mile_offer["offer_id"] = int(offer[G_OFFER_ID])
                                last_mile_offer["objective"] = 0
                                last_mile_offer["origin"] = visum_nodes[offer[G_RQ_ORIGIN]]
                                list_last_mile_offers.append(last_mile_offer)
                            if len(last_mile_offers) == 0:
                                last_mile_offer = {}
                                last_mile_offer["t_access"] = LARGE_INT
                                last_mile_offer["t_wait"] = LARGE_INT
                                last_mile_offer["t_drive"] = LARGE_INT
                                last_mile_offer["t_egress"] = LARGE_INT
                                last_mile_offer["fare"] = LARGE_INT
                                last_mile_offer["offer_id"] = 0
                                last_mile_offer["objective"] = 0
                                last_mile_offer["origin"] = -1
                                list_last_mile_offers.append(last_mile_offer)
                            send_obj["offers"] = list_last_mile_offers
                            self.format_object_and_send_msg(send_obj)
                        except:
                            error_str = traceback.format_exc()
                            error_obj = {"type":"fs_error", "content":error_str}
                            self.format_object_and_send_msg(error_obj)
                            raise EnvironmentError(error_str)
                    elif response_obj["type"] == "book_offer":
                        # user response -> non-blocking call
                        try:
                            self.open_agent_request = None
                            # unpack JSON message
                            agent_id = response_obj["agent_id"]
                            chosen_offer_id = response_obj["confirms_offer"]
                            # call fleet sim function
                            self.fleet_sim.user_response(agent_id, chosen_offer_id)
                            # pack JSON message
                            send_obj = {"type":"confirm_booking"}
                            send_obj["agent_id"] = agent_id
                            send_obj["offer_id"] = chosen_offer_id
                            self.format_object_and_send_msg(send_obj)
                        except:
                            error_str = traceback.format_exc()
                            error_obj = {"type":"fs_error", "content":error_str}
                            self.format_object_and_send_msg(error_obj)
                            raise EnvironmentError(error_str)
                    elif response_obj["type"] == "fleet_control_and_update":
                        # end of time step -> blocking call
                        try:
                            # unpack JSON message
                            if self.open_agent_request is not None:
                                self.fleet_sim.user_response(self.open_agent_request, 0)
                            self.open_agent_request = None
                            next_simulation_time = response_obj["time"]
                            # optimize fleet control
                            self.fleet_sim.optimize_fleet()
                            # increase simulation time
                            prev_simulation_time = self.fleet_sim.fs_time
                            simulation_time = next_simulation_time
                            # check: update network
                            self.fleet_sim.update_network(simulation_time)
                            # update state
                            # return list of agents that end their trip in the new time step
                            # arrivals follow this specification: arrival = {} with following keys
                            # agent_id | int |
                            # t_access | int |
                            # t_wait | int |
                            # t_drive | int |
                            # t_egress | int |
                            # car_id | int |
                            list_arrivals_rq = self.fleet_sim.update_state(simulation_time)
                            list_arrivals_dict = []
                            for t in range(prev_simulation_time+1, simulation_time+1):
                                prev_arrival_dict_list = self.delayed_arrivals.get(t)
                                if prev_arrival_dict_list is not None:
                                    for entry in prev_arrival_dict_list:
                                        list_arrivals_dict.append(entry)
                                    del self.delayed_arrivals[t]
                            for rq_obj in list_arrivals_rq:
                                t_access = rq_obj.t_access
                                t_egress = rq_obj.t_egress
                                t_wait = rq_obj.pu_time - rq_obj.rq_time - t_access
                                agent_arrival = {"agent_id": int(rq_obj.rid), "t_drive": int(rq_obj.do_time - rq_obj.pu_time),
                                                 "car_id": rq_obj.service_vid, "t_wait": int(t_wait),
                                                 "t_access":int(t_access), "t_egress": int(t_egress)}
                                t_arrival = int(rq_obj.do_time + t_egress)
                                if t_arrival <= simulation_time:
                                    list_arrivals_dict.append(agent_arrival)
                                else:
                                    try:
                                        self.delayed_arrivals[t_arrival].append(agent_arrival)
                                    except:
                                        self.delayed_arrivals[t_arrival] = [agent_arrival]
                            send_obj = {"type":"customers_arriving"}
                            # v1: pack JSON message
                            send_obj["time"] = simulation_time
                            send_obj["list_arrivals"] = list_arrivals_dict
                            self.format_object_and_send_msg(send_obj)
                        except:
                            error_str = traceback.format_exc()
                            error_obj = {"type":"fs_error", "content":error_str}
                            self.format_object_and_send_msg(error_obj)
                            raise EnvironmentError(error_str)
                    elif response_obj["type"] == "end_of_simulation":
                        # end of simulation
                        await_response = False
                        stay_online = False
                    elif response_obj["type"] == "mT_error":
                        # error in Java simulation
                        await_response = False
                        stay_online = False
                    else:
                        # raise Warning/Error for unknown message
                        pass
        # shut down server connection at the end of the simulation
        self.server_connection.close()

