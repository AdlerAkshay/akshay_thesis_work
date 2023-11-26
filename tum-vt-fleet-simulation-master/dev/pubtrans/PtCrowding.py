import os
import pandas as pd

from dev.pubtrans.PtModuleBase import PublicTransportBase, PTStation
from src.misc.globals import *

PER_HOUR_UPDATE_STEPS = 60


class PublicTransportWithCrowding(PublicTransportBase):
    def __init__(self, gtfs_data_dir, pt_stat_f, scenario_parameters, routing_engine, zone_system=None):
        """Initializes specific attributes to this model of public transportation.

        :param gtfs_data_dir: directory containing all necessary public transport data (preprocessed, crowding, ...)
        :param pt_stat_f: stat file to record information of simulation
        :param scenario_parameters: can contain additional information (e.g. pricing)
        :param routing_engine: can be used to retrieve node coordinates or compute routes on the street network
        :param zone_system: contains possible zone definitions
        """
        # TODO # after ISTTT: check definition
        super().__init__(gtfs_data_dir, pt_stat_f, scenario_parameters, routing_engine, zone_system)
        self.number_travelers_at_time = {}
        self.ts = scenario_parameters[G_SIM_TIME_STEP]
        self.interval_for_crowding_factor = scenario_parameters[G_NW_DENSITY_AVG_DURATION]
        self.crowding_factor = 1.0
        # define capacity for crowding of public transport system
        route_info_f = os.path.join(gtfs_data_dir, "add_route_information.csv")
        self.route_info_df = pd.read_csv(route_info_f)
        self.pt_total_nr_trips = 0
        self.pt_total_capacity = {}  # hour -> number of travelers that can be transported
        for i in range(24):
            nr_trips = self.route_info_df[f"nr_trips {i}-{i+1}"].sum()
            capacity_series = self.route_info_df[f"nr_trips {i}-{i+1}"] * self.route_info_df["trip_capacity"]
            # compute capacity on a per-minute level!!
            capacity = capacity_series.sum() / PER_HOUR_UPDATE_STEPS
            if i in scenario_parameters.get(G_PT_FRQ_HOURS, []):
                nr_trips = nr_trips * scenario_parameters.get(G_PT_FRQ_SCALE, 1.0)
                capacity = capacity * scenario_parameters.get(G_PT_FRQ_SCALE, 1.0)
            self.pt_total_nr_trips += nr_trips
            self.pt_total_capacity[i] = capacity
        self.pt_total_capacity[24] = self.pt_total_capacity[0]
        # TODO # after ISTTT define costs, emissions and fare system

    def query_station_to_station_nodes(self, o_station_node_id, d_station_node_id, earliest_start):
        """
        This method simulates a query to a public transportation app that only accepts
        It returns a public transportation offer.
        :param o_station_node_id: node index of origin station
        :param d_station_node_id: node index of destination station
        :param earliest_start: usually simulation_time, but can also be later for intermodal transport
        :return: pt offer {}: key > value [definition of keys in src/misc/globals.py]
        G_OFFER_ACCESS_W > float | seconds | access walking time can be set to 0 for station to station
        G_OFFER_WAIT > float | seconds | waiting time between earliest start and trip start
        G_OFFER_DRIVE > float | seconds | time between entering the train at the o_station and leaving at d_station
        G_OFFER_EGRESS_W > float | seconds | egress walking time can be set to 0 for station to station
        G_OFFER_COST > int | cents | price model probably has to be added additional to gtfs data
        G_OFFER_TRANSFERS > int | | number of transfers on the returned route
        """
        # TODO # after ISTTT: query_station_to_station_nodes()
        return {}

    def assign_to_pt_network(self, pt_start_time, pt_end_time, route=[]):
        """
        This method is used to keep track of crowding in the PT system. It assigns a traveler to the public transport
        network; the assignment is primarily time-based, but can also be route based.
        :param pt_start_time: start time of the PT leg of a traveler's journey
        :param pt_end_time: end time of the PT leg of a traveler's journey
        :param route: optional parameter (for finer resolution of crowding)
        """
        for t in range(pt_start_time, pt_end_time):
            try:
                self.number_travelers_at_time[t] += 1
            except KeyError:
                self.number_travelers_at_time[t] = 1

    def update_pt_network(self, simulation_time):
        """
        This method updates the crowding factor of the PT system.
        :param simulation_time: simulation time in seconds
        """
        sum = 0
        counter = 0
        for t in range(simulation_time-self.interval_for_crowding_factor, simulation_time, self.ts):
            sum += self.number_travelers_at_time.get(t, 0)
            counter += 1
        avg_pt_travelers = sum / counter
        sim_hour = simulation_time // 3600
        self.crowding_factor = avg_pt_travelers / self.pt_total_capacity[sim_hour]
