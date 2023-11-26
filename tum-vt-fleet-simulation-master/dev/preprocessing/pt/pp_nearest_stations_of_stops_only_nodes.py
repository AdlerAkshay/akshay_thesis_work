import os
import sys
import pandas as pd

MAIN = os.path.join(os.path.abspath(os.path.dirname(__file__)), os.path.pardir, os.path.pardir, os.path.pardir)
sys.path.append(MAIN)

from dev.pubtrans.PtModuleBase import PublicTransportBase
# TODO # after ISTTT: change to base network class later on
from src.routing.NetworkBasic import NetworkBasic

from src.misc.globals import *


def pp_nearest_stop_only_stations(pt_nw_dir):
    """This function generates the nearest station for the stop-only nodes within the network. GTFS and network data
    directories are derived from the pt_nw_dir. The script generates the 'stop_only_stations.csv' file in the
    pt_nw_dir.

    :param pt_nw_dir: network-matched sub-directory of public transport data directory.
    :type pt_nw_dir: str
    """
    # derive directories
    nw_base_name = os.path.basename(pt_nw_dir)
    gtfs_dir = os.path.join(pt_nw_dir, os.path.pardir)
    nw_name_dir = os.path.join(os.path.abspath(gtfs_dir), os.path.pardir, os.path.pardir, "networks", nw_base_name)
    # load network and pt module
    # TODO # after ISTTT: use base network class to create list of stop_nodes
    nw = NetworkBasic(nw_name_dir)
    # TODO # after ISTTT:  -> no need for clusters!!
    zone_nw_dir = r"C:\Users\ne53qez\Data_and_Simulation\tum-vt-fleet-simulation\data\zones\A99-iB2R\MUNbene_withBPs_300_1_LHMArea_OVstations_reduced_Flo"
    from src.infra.Zoning import ZoneSystem
    zones = ZoneSystem(zone_nw_dir)
    nw.add_init_data(start_time=0, time_step=1, moving_average_temporal_bin_size=1, moving_average_duration=1,
                      zone_system_obj=zones, network_stat_f=os.path.join(pt_nw_dir, "delete.csv"))
    scenario_par_dummy = {G_PT_FARE_B:0, G_WALKING_SPEED:1.33, G_NETWORK_NAME:nw_base_name}
    pt = PublicTransportBase(gtfs_dir, "nofile", scenario_par_dummy, nw)
    # create output
    output_list = []
    total_nodes = nw.get_number_network_nodes()
    counter_nodes = 0
    counter_stops = 0
    for node_index, node_obj in nw.nodes.items():
        counter_nodes += 1
        if counter_nodes % 1000 == 0:
            print(f"\t{counter_nodes}/{total_nodes}: currently found stop nodes: {counter_stops}")
        if node_obj.must_stop():
            counter_stops += 1
            if counter_stops % 10 == 0:
                print(f"\t\t currently found stop nodes: {counter_stops}")
            # all pt stations
            pt_all_station_index, _, _ = pt.return_nearest_station_node(node_index, rail_only=False,
                                                                        method="StreetNetwork", direction="to_PT")
            # rail only pt stations
            pt_rail_station_index, _, _ = pt.return_nearest_station_node(node_index, rail_only=True,
                                                                         method="StreetNetwork", direction="to_PT")
            output_list.append([node_index, pt_all_station_index, pt_rail_station_index])
    print(f"{counter_nodes}/{total_nodes}: found stop nodes: {counter_stops}")
    df = pd.DataFrame(output_list, columns=["network_stop_id", "nearest_station_id", "nearest_rail_station_id"])
    output_f = os.path.join(pt_nw_dir, "stop_only_stations.csv")
    df.to_csv(output_f, index=False)


if __name__ == "__main__":
    pp_nearest_stop_only_stations(sys.argv[1])
