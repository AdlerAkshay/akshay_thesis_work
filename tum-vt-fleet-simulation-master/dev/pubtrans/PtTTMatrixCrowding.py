# -------------------------------------------------------------------------------------------------------------------- #
# standard distribution imports
# -----------------------------
import logging
import os

# additional module imports (> requirements)
# ------------------------------------------
import pandas as pd

# src imports
# -----------
from dev.pubtrans.PtModuleBase import PublicTransportBase

# -------------------------------------------------------------------------------------------------------------------- #
# global variables
# ----------------
from src.misc.globals import *
LOG = logging.getLogger(__name__)

LIMIT_FOR_OPERATION = 0.25  # PT is assumed to be not available below this relative capacity
CENTRAL_F = 3   # factor to increase crowding (in reality, travelers are not spread among whole PT network!)


# -------------------------------------------------------------------------------------------------------------------- #
# main class
class PublicTransportTravelTimeMatrixWithCrowding(PublicTransportBase):
    def __init__(self, gtfs_data_dir, pt_stat_f, scenario_parameters, routing_engine, zone_system=None):
        """Initializes specific attributes to this model of public transportation.

        :param gtfs_data_dir: directory containing all necessary public transport data (preprocessed, crowding, ...)
        :param pt_stat_f: stat file to record information of simulation
        :param scenario_parameters: can contain additional information (e.g. pricing)
        :param routing_engine: can be used to retrieve node coordinates or compute routes on the street network
        :param zone_system: contains possible zone definitions
        """
        super().__init__(gtfs_data_dir, pt_stat_f, scenario_parameters, routing_engine, zone_system)
        self.number_travelers_at_time = {}
        self.start_time = scenario_parameters[G_SIM_START_TIME]
        self.ts = scenario_parameters[G_SIM_TIME_STEP]
        self.bin_size_for_crowding_factor = scenario_parameters[G_NW_DENSITY_T_BIN_SIZE]
        self.interval_for_crowding_factor = scenario_parameters[G_NW_DENSITY_AVG_DURATION]
        self.crowding_factor = 1.0
        # load station-to-station travel information matrix with columns walk_distance, travel_time, number_transfers
        tt_f = os.path.join(gtfs_data_dir, "traveltime_matrix.csv")
        self.s2s_df = pd.read_csv(tt_f, index_col=[0,1])
        # define capacity for crowding of public transport system
        route_info_f = os.path.join(gtfs_data_dir, "add_route_information.csv")
        self.route_info_df = pd.read_csv(route_info_f)
        self.pt_total_nr_trips = 0
        self.pt_total_capacity = {}  # hour -> number of travelers that can be transported
        self.operating_hours = {}  # hour -> True/False | False if less than 25% of max capacity is available
        for i in range(24):
            nr_trips = self.route_info_df[f"{G_PT_R_NR} {i}-{i+1}"].sum()
            # compute capacity per line:
            #   -> number of trips per hour * average trip duration / 60 minutes * capacity per traing
            capacity_series = self.route_info_df[f"{G_PT_R_NR} {i}-{i+1}"] * self.route_info_df[G_PT_R_CAP]\
                              * self.route_info_df[G_PT_AVG_DUR] / 60
            capacity = capacity_series.sum()
            if i in scenario_parameters.get(G_PT_FRQ_HOURS, []):
                nr_trips = nr_trips * scenario_parameters.get(G_PT_FRQ_SCALE, 1.0)
                capacity = capacity * scenario_parameters.get(G_PT_FRQ_SCALE, 1.0)
            self.pt_total_nr_trips += nr_trips
            self.pt_total_capacity[i] = capacity
        self.pt_total_capacity[24] = self.pt_total_capacity[0]
        max_capacity = max(self.pt_total_capacity)
        for i in range(24):
            if self.pt_total_capacity[i]/max_capacity >= LIMIT_FOR_OPERATION:
                self.operating_hours[i] = True
        # load background travelers (commuters from/to outside of study area)
        # TODO # change directory
        self.bg_trav = {}    # hour -> number concurrent travelers
        bg_trav_f = os.path.join(gtfs_data_dir, "pt_background_travelers.csv")
        if os.path.isfile(bg_trav_f):
            bg_trav_df = pd.read_csv(bg_trav_f, index_col=0)
            if "all" in bg_trav_df.index:
                for hour in range(24):
                    self.bg_trav[hour] = bg_trav_df.loc["all", f"concurrent_bg_travelers {hour}-{hour+1}"]
            else:
                # TODO # after ISTTT:
                raise EnvironmentError("Route fine PT background travelers not yet implemented!")
        # record PT stats
        with open(self.pt_stat_f, "w") as fhout:
            fhout.write(f"{G_SIM_TIME},{G_PT_ROUTE},{G_PT_TRAVELERS},{G_PT_MOV_AVG},{G_PT_BG_T},{G_PT_CAP},"
                        f"{G_PT_CROWD}\n")

    def query_station_to_station_nodes(self, o_station_id, d_station_id, earliest_start):
        """This method simulates a query to a public transportation app that only accepts stations as input.
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
        G_OFFER_CROWD > float | | share [0,1] of full pt_total_capacity that is currently occupied
        """
        offer_dict = {}
        # only offer PT trip if origin station and destination station are not the same
        if o_station_id == d_station_id:
            return offer_dict
        hour = earliest_start // 3600
        if not self.operating_hours.get(hour, False):
            return offer_dict
        try:
            s2s_info = self.s2s_df.loc[(o_station_id, d_station_id)]
        except:
            LOG.warning("key error in station-station-query from {} to {}; no pt offer is returned".format(o_station_id, d_station_id))
            return offer_dict
        # transfer walk distance is already included in travel time; hence the values are not used
        # no wait time modeled for matrices
        offer_dict[G_OFFER_DRIVE] = s2s_info[G_PT_TT_TT]
        offer_dict[G_OFFER_TRANSFERS] = s2s_info[G_PT_TT_NT]
        # number transfers is -1 if best PT option is to walk
        if offer_dict[G_OFFER_TRANSFERS] == -1:
            # including walking between stations is better than no alternative
            # use self.base_fare as fare for every trip
            offer_dict[G_OFFER_FARE] = 0
            # use current step crowding factor
            offer_dict[G_OFFER_CROWD] = 0
        else:
            # use self.base_fare as fare for every trip
            offer_dict[G_OFFER_FARE] = self.base_fare
            # use current step crowding factor
            offer_dict[G_OFFER_CROWD] = self.crowding_factor
        return offer_dict

    def assign_to_pt_network(self, pt_start_time, pt_end_time, route=[]):
        """
        This method is used to keep track of crowding in the PT system. It assigns a traveler to the public transport
        network; the assignment is primarily time-based, but can also be route based.
        :param pt_start_time: start time of the PT leg of a traveler's journey
        :param pt_end_time: end time of the PT leg of a traveler's journey
        :param route: optional parameter (for finer resolution of crowding)
        """
        LOG.debug("assign to pt network {} {}".format(pt_start_time, pt_end_time))
        pt_start_bin = int(pt_start_time / self.bin_size_for_crowding_factor)
        pt_end_bin = max(pt_start_bin + 1, int(pt_end_time / self.bin_size_for_crowding_factor))
        for t in range(pt_start_bin, pt_end_bin):
            try:
                self.number_travelers_at_time[t*self.bin_size_for_crowding_factor] += 1
            except KeyError:
                self.number_travelers_at_time[t*self.bin_size_for_crowding_factor] = 1

    def update_pt_network(self, simulation_time):
        """
        This method updates the crowding factor of the PT system.
        :param simulation_time: simulation time in seconds
        """
        s = 0
        counter = 0
        LOG.debug("update_pt: number travelers: {}".format(self.number_travelers_at_time))
        LOG.debug("update pt : bins to check {}".format([t for t in range(simulation_time-self.interval_for_crowding_factor, simulation_time, self.bin_size_for_crowding_factor)]))
        for t in range(simulation_time-self.interval_for_crowding_factor, simulation_time,
                       self.bin_size_for_crowding_factor):
            if t >= self.start_time:
                s += self.number_travelers_at_time.get(t, 0)
                counter += 1
        try:
            avg_pt_travelers = s / counter
            LOG.debug("avg pt travelers: {}".format(avg_pt_travelers))
        except:
            avg_pt_travelers = 0
        try:
            del self.number_travelers_at_time[simulation_time - self.interval_for_crowding_factor]
        except:
            pass
        # TODO # after ISTTT: think about consistency -> always seconds as internal time unit? fix bin size?
        sim_hour = simulation_time // 3600
        # TODO # is it necessary to consider a factor for uneven distribution? some parts of the PT network can be at
        # the limit, while others are not.
        bg_travelers = self.bg_trav.get(sim_hour, 0)
        self.crowding_factor = min(CENTRAL_F * (avg_pt_travelers + bg_travelers) / self.pt_total_capacity[sim_hour], 1)
        with open(self.pt_stat_f, "a") as fhout:
            fhout.write(f"{simulation_time},total,{self.number_travelers_at_time.get(simulation_time, 0)},"
                        f"{avg_pt_travelers},{bg_travelers},{self.pt_total_capacity[sim_hour]},"
                        f"{self.crowding_factor }\n")
