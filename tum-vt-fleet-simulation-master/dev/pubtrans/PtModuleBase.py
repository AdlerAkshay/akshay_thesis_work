# -------------------------------------------------------------------------------------------------------------------- #
# standard distribution imports
# -----------------------------
import logging
import os
from abc import abstractmethod

# additional module imports (> requirements)
# ------------------------------------------
import pandas as pd

# src imports
# -----------
from src.misc.gis import returnEuclidianDistance

# -------------------------------------------------------------------------------------------------------------------- #
# global variables
# ----------------
from src.misc.globals import *
LOG = logging.getLogger(__name__)
LARGE_INT = 100000


# -------------------------------------------------------------------------------------------------------------------- #
# functions
# ---------
def create_stations(columns):
    list_stops = [int(x) for x in str(columns["stop_ids"]).split(";")]
    list_lines = [int(x) for x in str(columns["route_ids"]).split(";")]
    return PTStation(columns["station_id"], columns["coordinate_1"], columns["coordinate_2"], columns["rail_based"],
                     list_lines, list_stops, columns["network_node_id"])


# -------------------------------------------------------------------------------------------------------------------- #
# class definitions
# -----------------
class PTStation:
    def __init__(self, station_id, pos_x, pos_y, is_rail_station, list_lines, list_stop_ids, street_network_node_id):
        self.station_id = station_id
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.street_network_node_id = street_network_node_id
        self.is_rail_station = is_rail_station
        self.list_stop_ids = list_stop_ids
        self.list_lines = list_lines


# -------------------------------------------------------------------------------------------------------------------- #
# main
# ----
class PublicTransportBase:
    def __init__(self, gtfs_data_dir, pt_stat_f, scenario_parameters, routing_engine, zone_system=None):
        """
        Loads the public transport module with the necessary data.
        :param gtfs_data_dir: directory containing all necessary public transport data (preprocessed, crowding, ...)
        :param pt_stat_f: stat file to record information of simulation
        :param scenario_parameters: can contain additional information (e.g. pricing)
        :param routing_engine: can be used to retrieve node coordinates or compute routes on the street network
        :param zone_system: contains possible zone definitions
        """
        self.pt_stat_f = pt_stat_f
        self.base_fare = scenario_parameters[G_PT_FARE_B]
        self.walking_speed = scenario_parameters[G_WALKING_SPEED]
        self.routing_engine = routing_engine
        self.zones = zone_system
        stations_f = os.path.join(gtfs_data_dir, "stations.csv")
        stations_df = pd.read_csv(stations_f)
        station_node_f = os.path.join(gtfs_data_dir, scenario_parameters[G_NETWORK_NAME], "station_nodes.csv")
        station_node_df = pd.read_csv(station_node_f)
        stations_df = stations_df.merge(station_node_df, left_on="station_id", right_on="station_id")
        # self.station_dict = {}  # station_id -> PTStation
        self.station_dict = stations_df.apply(create_stations, axis=1).to_dict()
        # creation of additional access options to pt-station objects
        self.st_nw_stations = {}   # street nw node id -> list station ids
        tmp_st_nw_stations = {}
        for station_obj in self.station_dict.values():
            try:
                tmp_st_nw_stations[station_obj.street_network_node_id].append(station_obj)
            except KeyError:
                tmp_st_nw_stations[station_obj.street_network_node_id] = [station_obj]
        # sort such that rail stations are first
        for k,v in tmp_st_nw_stations.items():
            self.st_nw_stations[k] = sorted(v, key=lambda x: x.is_rail_station, reverse=True)
        self.rail_stations = [x for x in self.station_dict.values() if x.is_rail_station]
        # get preprocessed data for nearest stations of street network stop nodes
        stop_node_station_f = os.path.join(gtfs_data_dir, scenario_parameters[G_NETWORK_NAME], "stop_only_stations.csv")
        self.stop_pt_stations_df = pd.read_csv(stop_node_station_f, index_col=0)
        # TODO # after ISTTT: init public transport network from gtfs (only in modules where GTFS is used!)
        # TODO # after ISTTT: think about a public transport time-card attribute for requests!

    def return_nearest_station_node(self, street_network_node, rail_only=False, method="StreetNetwork",
                                    direction="to_PT"):
        """
        This method returns the public transport station that can be accessed fastest from a street network node
        (according to Euclidian distance or routing on the street network).
        :param street_network_node: node_index
        :param rail_only: ignore bus stations for this search
        :param method: "Euclidian" (default) or "StreetNetwork" (only works if street_nw_node_id for stops is available)
        :return: (station_node_index, travel_distance, street_network_route)
        """
        street_network_pos = self.routing_engine.return_node_position(street_network_node)
        if method == "StreetNetwork":
            if street_network_node in self.stop_pt_stations_df.index:
                if rail_only:
                    station_index = self.stop_pt_stations_df["nearest_rail_station_id"].get(street_network_node)
                else:
                    station_index = self.stop_pt_stations_df["nearest_station_id"].get(street_network_node)
                station_st_nw_node_index = self.station_dict[station_index].street_network_node_id
                st_nw_pos = self.routing_engine.return_node_position(station_st_nw_node_index)
                if direction == "to_PT":
                    street_network_route = self.routing_engine.return_best_route_1to1(street_network_pos, st_nw_pos)
                else:
                    street_network_route = self.routing_engine.return_best_route_1to1(st_nw_pos, street_network_pos)
            else:
                if rail_only:
                    considered_stops = [self.routing_engine.return_node_position(x.street_network_node_id)\
                                        for x in self.rail_stations]
                else:
                    considered_stops = [self.routing_engine.return_node_position(x.street_network_node_id)\
                                        for x in self.station_dict.values()]
                if direction == "to_PT":
                    street_network_route = self.routing_engine.return_best_route_1toX(street_network_pos, considered_stops,
                                                                                      max_cost_value=LARGE_INT)
                    station_st_nw_node_index = street_network_route[-1]
                else:
                    street_network_route = self.routing_engine.return_best_route_Xto1(considered_stops, street_network_pos,
                                                                                      max_cost_value=LARGE_INT)
                    if len(street_network_route) == 0:
                        LOG.warning("no street network route from PT found to {} : {}".format(street_network_node, street_network_route))
                        #LOG.debug("considered stops : {}".format(considered_stops))
                        #self.routing_engine.checkNetwork(considered_stops, street_network_pos) # TODO # remove
                    station_st_nw_node_index = street_network_route[0]
                # due to sorting of self.st_nw_stations,
                # rail stations are preferred in case there are multiple pt stations
                # related to one street network node
                station_index = self.st_nw_stations[station_st_nw_node_index][0].station_id
            best_distance = self.routing_engine.return_route_infos(street_network_route, 0, 0)[1]
        else:
            # assume that all coordinates are given in same metric system!
            sn_x, sn_y = self.routing_engine.return_node_coordinates(street_network_node)
            best_distance = None
            station_index = None
            for station in self.station_dict.values():
                current_station_distance = returnEuclidianDistance(sn_x, sn_y, station.pos_x, station.pos_y)
                if not best_distance or current_station_distance < best_distance:
                    best_distance = current_station_distance
                    station_index = station.station_id
            street_network_route = []
        return station_index, best_distance, street_network_route

    def query(self, o_node, d_node, earliest_start):
        """
        This method simulates a query to a public transportation app. It returns a public transportation offer.
        It only assumes a route via the the nearest

        :param o_node: node in street network(!)
        :param d_node: node in street network(!)
        :param earliest_start: usually simulation_time, but can also be later for intermodal transport
        :return: pt offer {}: ...
        """
        LOG.debug(f"PT Query from {o_node} to {d_node}")
        o_station_id, access_distance, _ = self.return_nearest_station_node(o_node)
        d_station_id, egress_distance, _ = self.return_nearest_station_node(d_node)
        #
        o_walking_time = access_distance/self.walking_speed
        pt_station_start = earliest_start + o_walking_time
        d_walking_time = egress_distance/self.walking_speed
        #
        station_to_station_infos = self.query_station_to_station_nodes(o_station_id, d_station_id,
                                                                       pt_station_start)
        if station_to_station_infos:
            station_to_station_infos[G_OFFER_ACCESS_W] = o_walking_time
            station_to_station_infos[G_OFFER_EGRESS_W] = d_walking_time
            return station_to_station_infos
        else:
            return {}

    @abstractmethod
    def query_station_to_station_nodes(self, o_station_id, d_station_id, earliest_start):
        """
        This method simulates a query to a public transportation app that only accepts
        It returns a public transportation offer.
        :param o_station_id: node index of origin station
        :param d_station_id: node index of destination station
        :param earliest_start: usually simulation_time, but can also be later for intermodal transport
        :return: pt offer {}: key > value [definition of keys in src/misc/globals.py]
        G_OFFER_ACCESS_W > float | seconds | access walking time can be set to 0 for station to station
        G_OFFER_WAIT > float | seconds | waiting time between earliest start and trip start
        G_OFFER_DRIVE > float | seconds | time between entering the train at the o_station and leaving at d_station
        G_OFFER_EGRESS_W > float | seconds | egress walking time can be set to 0 for station to station
        G_OFFER_FARE > int | cents | price model probably has to be added additional to gtfs data
        G_OFFER_TRANSFERS > int | | number of transfers on the returned route
        """
        return {}

    @abstractmethod
    def assign_to_pt_network(self, pt_start_time, pt_end_time, route=[]):
        """
        This method is used to keep track of crowding in the PT system. It assigns a traveler to the public transport
        network; the assignment is primarily time-based, but can also be route based.
        :param pt_start_time: start time of the PT leg of a traveler's journey
        :param pt_end_time: end time of the PT leg of a traveler's journey
        :param route: optional parameter (for finer resolution of crowding)
        """
        pass

    @abstractmethod
    def update_pt_network(self, simulation_time):
        """
        This method updates the crowding factor of the PT system.
        :param simulation_time: simulation time in seconds
        """
        pass