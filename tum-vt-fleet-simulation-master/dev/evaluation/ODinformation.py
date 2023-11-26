import os
import sys
import glob
import pandas as pd
import numpy as np

MAIN_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(MAIN_DIR)
from src.infra.Zoning import ZoneSystem
from src.misc.globals import *


# -------------------------------------------------------------------------------------------------------------------- #
# help functions
def extract_offer_information(row, nr_mod_op):
    """This function extracts following information from the offers and sets nan if information is not available.
    - private vehicle driving time
    - private vehicle distance
    - private vehicle toll costs
    - private vehicle park costs
    - public transport driving time
    - public transport wait time
    - public transport access time
    - public transport egress time
    - public transport transfers
    - public transport fare
    - public transport crowding
    for each mod operator:
    - mod driving time
    - mod wait time
    - mod fare
    MIND: the number of output arguments is dependent on the number of operators!

    :param row: user-stat row
    :param nr_mod_op: number of operators to evaluate
    :return: tuple of above mentioned information
    """
    offers = row[G_RQ_OFFERS]
    pv_tt = np.nan
    pv_d = np.nan
    pv_tc = np.nan
    pv_pc = np.nan
    pt_tt = np.nan
    pt_wt = np.nan
    pt_at = np.nan
    pt_et = np.nan
    pt_transfers = np.nan
    pt_fare = np.nan
    pt_crowd = np.nan
    unsorted_mod_results = {}
    for i in range(nr_mod_op):
        mod_offer_made = 0
        mod_tt = np.nan
        mod_wt = np.nan
        mod_fare = np.nan
        unsorted_mod_results[i] = [mod_offer_made, mod_tt, mod_wt, mod_fare]
    #
    if not pd.isnull(offers):
        try:
            offers.split("|")
        except:
            print(offers)
            raise AssertionError
        for offers_by_mode in offers.split("|"):
            mode_str  = offers_by_mode.split(":")[0]
            offer_str = offers_by_mode[len(mode_str)+1:]
            mode = int(mode_str)
            offer_dict = {}
            for par_str in offer_str.split(";"):
                k, v = par_str.split(":")
                offer_dict[k] = float(v)
            # private vehicle
            if mode == G_MC_DEC_PV:
                if offer_dict.get(G_OFFER_DRIVE):
                    pv_tt = float(offer_dict.get(G_OFFER_DRIVE))
                if offer_dict.get(G_OFFER_DIST):
                    pv_d = float(offer_dict.get(G_OFFER_DIST))
                if offer_dict.get(G_OFFER_TOLL):
                    pv_tc = float(offer_dict.get(G_OFFER_TOLL))
                if offer_dict.get(G_OFFER_PARK):
                    pv_pc = float(offer_dict.get(G_OFFER_PARK))
            # public transport
            elif mode == G_MC_DEC_PT:
                if offer_dict.get(G_OFFER_DRIVE):
                    pt_tt = float(offer_dict.get(G_OFFER_DRIVE))
                if offer_dict.get(G_OFFER_WAIT):
                    pt_wt = float(offer_dict.get(G_OFFER_WAIT))
                if offer_dict.get(G_OFFER_ACCESS_W):
                    pt_at = float(offer_dict.get(G_OFFER_ACCESS_W))
                if offer_dict.get(G_OFFER_EGRESS_W):
                    pt_et = float(offer_dict.get(G_OFFER_EGRESS_W))
                if offer_dict.get(G_OFFER_TRANSFERS):
                    pt_transfers = float(offer_dict.get(G_OFFER_TRANSFERS))
                if offer_dict.get(G_OFFER_FARE):
                    pt_fare = float(offer_dict.get(G_OFFER_FARE))
                if offer_dict.get(G_OFFER_CROWD):
                    pt_crowd = float(offer_dict.get(G_OFFER_CROWD))
            # MOD
            elif mode >= 0:
                mod_offer_made = 1
                if offer_dict.get(G_OFFER_DRIVE):
                    mod_tt = float(offer_dict.get(G_OFFER_DRIVE))
                else:
                    mod_tt = np.nan
                if offer_dict.get(G_OFFER_WAIT):
                    mod_wt = float(offer_dict.get(G_OFFER_WAIT))
                else:
                    mod_wt = np.nan
                if offer_dict.get(G_OFFER_FARE):
                    mod_fare = float(offer_dict.get(G_OFFER_FARE))
                else:
                    mod_fare = np.nan
                unsorted_mod_results[mode] = [mod_offer_made, mod_tt, mod_wt, mod_fare]
    # sort mod_results by operator_id
    mod_results = []
    for op_id in sorted(unsorted_mod_results.keys()):
        mod_results.extend(unsorted_mod_results[op_id])
    return (pv_tt, pv_d, pv_tc, pv_pc, pt_tt, pt_wt, pt_at, pt_et, pt_transfers, pt_fare, pt_crowd, *mod_results)


def set_zones(od_row, node_zone_dict):
    """This function returns the origin and destination zones for a given node_zone_dictionary.

    :param od_row: aggregated od row on node level
    :param node_zone_dict: node -> zone
    :return: o_zone, d_zone
    """
    o_zone = node_zone_dict.get(od_row["origin_node"])
    d_zone = node_zone_dict.get(od_row["destination_node"])
    return o_zone, d_zone


def set_zones_nr_rq_veh_f(veh_f_row, node_zone_dict):
    """This function returns the origin and destination zones for a given node_zone_dictionary in a vehicle stat file.

    :param veh_f_row: row in vehicle stat file
    :param node_zone_dict:node -> zone
    :return: o_zone, d_zone
    """
    o_pos_str = veh_f_row["start_pos"]
    o_node = int(o_pos_str.split(";")[0])
    d_pos_str = veh_f_row["end_pos"]
    d_node = int(d_pos_str.split(";")[0])
    o_zone = node_zone_dict.get(o_node)
    d_zone = node_zone_dict.get(d_node)
    rq_str = str(veh_f_row["rq_on_board"])
    nr_rq = len(rq_str.split(";"))
    return o_zone, d_zone, nr_rq


# -------------------------------------------------------------------------------------------------------------------- #
# main function
def od_eval(output_dir, create_od_plots=False, external_zone_system_network_dir=None, print_progress=False):
    """This function evaluates OD specific information (mode choice, share offers, offered quantities) and also
    aggregates these results on zone level if a zone system is used in the simulation. The optional parameter
    'external_zone_system_network_dir' can be used to evaluate on a certain zone system even though it was not used
    in the simulation.

    :param output_dir: simulation scenario to evaluate
    :param create_od_plots: script will generate OD pictures if True
    :param external_zone_system_network_dir: script will use this zone system if it is not None
    :param print_progress: print progress if True
    :return:
    """
    # 1) load information
    # -------------------
    geometry_available = False
    scenario_parameters, list_operator_attributes, _ = load_scenario_inputs(output_dir)
    dir_names = get_directory_dict(scenario_parameters)
    if print_progress:
        print(f"Loading information for scenario {scenario_parameters.get(G_SCENARIO_NAME)}...")
    nr_mod_operators = scenario_parameters[G_NR_OPERATORS]
    zone_system_name = dir_names.get(G_DIR_ZONES, None)
    if external_zone_system_network_dir is not None:
        zone_system_name = external_zone_system_network_dir
    if zone_system_name:
        zone_system = ZoneSystem(zone_system_name, scenario_parameters, dir_names)
        # TODO # load geometry if available
    else:
        zone_system = None
    user_stats_f = os.path.join(output_dir, "1_user-stats.csv")
    us_df = pd.read_csv(user_stats_f)
    offer_columns = ["pv travel_time", "pv distance", "pv toll_cost", "pv park_cost", "pt travel_time", "pt wait_time",
                     "pt access_time", "pt egress_time", "pt transfers", "pt fare", "pt crowding"]
    for i in range(nr_mod_operators):
        offer_columns.extend([f"mod-{i} offer_made", f"mod-{i} travel_time", f"mod-{i} wait time", f"mod-{i} fare"])
    # load node-to-node results if available
    node_od_eval_f = os.path.join(output_dir, "od_node_eval.csv")
    od_cols = ["origin_node", "destination_node", "nr_travelers"]
    od_ms_cols = ["pv mode_share", "pt mode_share"]
    od_value_cols = ["pv travel_time", "pv distance", "pv toll_cost", "pv park_cost", "pt travel_time", "pt wait_time",
                     "pt access_time", "pt egress_time", "pt transfers", "pt fare", "pt crowding"]
    for i in range(nr_mod_operators):
        od_ms_cols.append(f"mod-{i} mode_share")
        od_value_cols.extend([f"mod-{i} offer_made", f"mod-{i} travel_time", f"mod-{i} wait time", f"mod-{i} fare"])
    if os.path.isfile(node_od_eval_f):
        print("Loading existing OD evaluation on node-level ...")
        od_info_df = pd.read_csv(node_od_eval_f)
    else:
        # 2) extract offer information
        # ----------------------------
        if print_progress:
            print("Extracting offer details ...")
        us_df[offer_columns] = us_df.apply(extract_offer_information, axis=1, args=(nr_mod_operators,),
                                           result_type="expand")

        # 3) evaluate OD information on node-level
        # ----------------------------------------
        if print_progress:
            print("Evaluating OD information on node level ...")
        all_od_cols = od_cols + od_ms_cols + od_value_cols
        list_od_info = []
        counter = 0
        max_number_od = len(us_df[G_RQ_ORIGIN].unique()) * len(us_df[G_RQ_DESTINATION].unique())
        for od_label, od_df in us_df.groupby([G_RQ_ORIGIN, G_RQ_DESTINATION]):
            counter += 1
            if print_progress and counter % 10000 == 0:
                print(f"\t\t... od-group {counter} of maximum {max_number_od}")
            single_od_infos = []
            o_pos, d_pos = od_label
            o_node = int(o_pos.split(";")[0])
            d_node = int(d_pos.split(";")[0])
            tot_travelers = od_df.shape[0]
            ms = od_df[G_RQ_OP_ID].value_counts() / tot_travelers
            pv_ms = ms.get(G_MC_DEC_PV, 0)
            pt_ms = ms.get(G_MC_DEC_PT, 0)
            single_od_infos.extend([o_node, d_node, tot_travelers, pv_ms, pt_ms])
            for i in range(nr_mod_operators):
                single_od_infos.append(ms.get(i, 0))
            for cat in od_value_cols:
                single_od_infos.append(od_df[cat].mean())
            list_od_info.append(single_od_infos)
        od_info_df = pd.DataFrame(list_od_info, columns=all_od_cols)
        od_info_df.to_csv(node_od_eval_f, index=False)
        if print_progress:
            print(f"\t... saved results to {node_od_eval_f}")

    # 4) cluster OD information on node-level for relevant node-pairs
    # ---------------------------------------------------------------
    sorted_df = od_info_df.sort_values("nr_travelers", ascending=False)
    nr_tot_travelers = sorted_df["nr_travelers"].sum()
    nr_rows = sorted_df.shape[0]
    # # only pick highest quartile
    # top_od_trip_quartile = sorted_df.head(nr_rows // 4).copy()
    # use all trips
    top_od_trip_quartile = sorted_df
    top_od_trip_quartile_trips = top_od_trip_quartile["nr_travelers"].sum()
    share_trips = 100 * (top_od_trip_quartile_trips / nr_tot_travelers)
    min_trip_number_per_od = top_od_trip_quartile["nr_travelers"].min()
    if print_progress:
        print(f"Analyzing top trip-quartile of OD node connections (covering {share_trips} % of all trips and"
              f" containing at least {min_trip_number_per_od} trips)")
    dom_cols = ["pv dominant", "pt dominant"]
    top_od_trip_quartile["pv dominant"] = top_od_trip_quartile["pv mode_share"] > 0.5
    top_od_trip_quartile["pt dominant"] = top_od_trip_quartile["pt mode_share"] > 0.5
    for op_id in range(nr_mod_operators):
        dom_cols.append(f"mod-{op_id} dominant")
        top_od_trip_quartile[f"mod-{op_id} dominant"] = top_od_trip_quartile[f"mod-{op_id} mode_share"] > 0.5
    all_od_cluster_cols = dom_cols + ["nr_travelers", "nr_od_pairs"] + od_ms_cols + od_value_cols
    list_result_clusters = []
    for dom_label, dom_df in top_od_trip_quartile.groupby(dom_cols):
        tot_travelers = dom_df["nr_travelers"].sum()
        tot_od_pairs = dom_df.shape[0]
        single_od_infos = [*dom_label, tot_travelers, tot_od_pairs]
        for cat in od_ms_cols + od_value_cols:
            weighed_sum = dom_df["nr_travelers"] * dom_df[cat]
            single_od_infos.append(weighed_sum.sum() / tot_travelers)
        list_result_clusters.append(single_od_infos)
    od_cluster_res_df = pd.DataFrame(list_result_clusters, columns=all_od_cluster_cols)
    for cat in od_value_cols:
        cat_max = od_cluster_res_df[cat].max()
        # max scale
        od_cluster_res_df[f"scaled {cat}"] = od_cluster_res_df[cat] / cat_max
        # # min-max scale
        # cat_min = od_cluster_res_df[cat].min()
        # od_cluster_res_df[f"scaled {cat}"] = (od_cluster_res_df[cat] - cat_min) / (cat_max - cat_min)
    cluster_od_eval_f = os.path.join(output_dir, "od_result_cluster_eval.csv")
    od_cluster_res_df.to_csv(cluster_od_eval_f, index=False)
    if print_progress:
        print(f"\t... saved results to {cluster_od_eval_f}")

    # 5) evaluate OD information on zone-level
    # ----------------------------------------
    if zone_system:
        if print_progress:
            print(f"Evaluating OD information on zone level for zone system {zone_system_name} ...")
        node_zone_dict = zone_system.node_zone_df[G_ZONE_ZID].to_dict()
        zone_cols = ["origin_zone", "destination_zone"]
        od_info_df[zone_cols] = od_info_df.apply(set_zones, axis=1, args=(node_zone_dict,), result_type="expand")
        all_od_zone_cols = zone_cols + ["nr_travelers"] + od_ms_cols + od_value_cols
        list_od_zone_info = []
        for od_zone_label, od_zone_df in od_info_df.groupby(zone_cols):
            tot_travelers = od_zone_df["nr_travelers"].sum()
            single_od_infos = [od_zone_label[0], od_zone_label[1], tot_travelers]
            for cat in od_ms_cols + od_value_cols:
                weighed_sum = od_zone_df["nr_travelers"] * od_zone_df[cat]
                single_od_infos.append(weighed_sum.sum() / tot_travelers)
            list_od_zone_info.append(single_od_infos)
        od_zone_info_df = pd.DataFrame(list_od_zone_info, columns=all_od_zone_cols)
        zone_od_eval_f = os.path.join(output_dir, "od_zone_eval.csv")
        od_zone_info_df.to_csv(zone_od_eval_f, index=False)
        if print_progress:
            print(f"\t... saved results to {zone_od_eval_f}")

    # 7) create plots if possible
    # ---------------------------
    if create_od_plots and geometry_available:
        # TODO # create plots if all zones have a geometry
        pass


def get_zone_to_zone_sharing_rates(output_dir, external_zone_system_network_dir=None, print_progress=True):
    """This function follows trajectories of vehicles and determines, which OD connections (on zone level) have shared
    trips.
    It also returns a sharing rate per zone.

    :param output_dir: scenario output directory
    :param external_zone_system_network_dir: external zone system that does not necessarily have to be the one used
            in the simulation
    :param print_progress: output shell messages
    :return: None
    """
    # load zone system
    scenario_parameters, list_operator_attributes, _ = load_scenario_inputs(output_dir)
    dir_names = get_directory_dict(scenario_parameters)
    zone_system_nw_dir = dir_names.get(G_DIR_ZONES, None)
    if external_zone_system_network_dir is not None:
        zone_system_nw_dir = external_zone_system_network_dir
    if zone_system_nw_dir:
        zone_system = ZoneSystem(zone_system_nw_dir, scenario_parameters, dir_names)
        zone_system_name = os.path.basename(os.path.dirname(zone_system_nw_dir))
    else:
        raise IOError("This script requires zone information!")
    node_zone_dict = zone_system.node_zone_df[G_ZONE_ZID].to_dict()
    sc_name = os.path.basename(output_dir)
    if print_progress:
        print(f"get_zone_to_zone_sharing_rates({sc_name},{zone_system_name})")
    # load vehicle stat files
    glob_str = os.path.join(output_dir, "2-*_op-stats.csv")
    list_veh_stat_f = glob.glob(glob_str)
    combined_results_pax = None
    combined_results_rq = None
    for f in list_veh_stat_f:
        if print_progress:
            bn = os.path.basename(f)
            print(f"\t file {bn} preparing data frame")
        df = pd.read_csv(f)
        # filter customer trips
        fdf = df[(df["status"] == "route") & (df["occupancy"] > 0)]
        fdf[["o_zone", "d_zone", "nr_rq"]] = fdf.apply(set_zones_nr_rq_veh_f, axis=1, args=(node_zone_dict,),
                                                       result_type="expand")
        #
        if print_progress:
            print(f"\t aggregating data")
        list_odo = []
        for odo_tuple, tmp_df in fdf.groupby(["o_zone", "d_zone", "occupancy"]):
            o_zone, d_zone, occupancy = odo_tuple
            list_odo.append([o_zone, d_zone, occupancy, tmp_df.shape[0]])
        occ_df = pd.DataFrame(list_odo, columns=["o_zone", "d_zone", "occupancy", "number_trips"])
        occ_df.set_index(["o_zone", "d_zone", "occupancy"], inplace=True)
        if combined_results_pax is None:
            combined_results_pax = occ_df
        else:
            occ_df.rename({"number_trips": "add_trips"}, axis=1, inplace=True)
            combined_results_pax = combined_results_pax.merge(occ_df, left_index=True, right_index=True)
            combined_results_pax["number_trips"] += combined_results_pax["add_trips"]
            combined_results_pax.drop("add_trips", axis=1, inplace=True)
        #
        list_odr = []
        for odr_tuple, tmp_df in fdf.groupby(["o_zone", "d_zone", "nr_rq"]):
            o_zone, d_zone, nr_rq = odr_tuple
            list_odr.append([o_zone, d_zone, nr_rq, tmp_df.shape[0]])
        occ_df = pd.DataFrame(list_odr, columns=["o_zone", "d_zone", "nr_rq", "number_trips"])
        occ_df.set_index(["o_zone", "d_zone", "nr_rq"], inplace=True)
        if combined_results_rq is None:
            combined_results_rq = occ_df
        else:
            occ_df.rename({"number_trips": "add_trips"}, axis=1, inplace=True)
            combined_results_rq = combined_results_pax.merge(occ_df, left_index=True, right_index=True)
            combined_results_rq["number_trips"] += combined_results_rq["add_trips"]
            combined_results_rq.drop("add_trips", axis=1, inplace=True)
    #
    if print_progress:
        print("\t Writing output files")
    out_f_pax = os.path.join(output_dir, f"od_sharing_occ_{zone_system_name}.csv")
    combined_results_pax.to_csv(out_f_pax)
    out_f_rq = os.path.join(output_dir, f"od_sharing_rq_{zone_system_name}.csv")
    combined_results_rq.to_csv(out_f_rq)
    return combined_results_pax, combined_results_rq


if __name__ == "__main__":
    # output_dir = sys.argv[1]
    output_dir = r"C:\Users\ne53qez\Data_and_Simulation\tum-vt-fleet-simulation\studies\Flo_PhD_Operations\results\s1_5_1250_PoolIRSBatch_-_-_-_1"
    external_zone_system = r"C:\Users\ne53qez\Data_and_Simulation\tum-vt-fleet-simulation\data\zones\muc_a99_max2km_4lvl_mivoev\MUNbene_withBPs_300_1_LHMArea_OVstations_reduced_Flo"
    if os.path.isdir(output_dir):
        # od_eval(output_dir, create_od_plots=True, print_progress=True)
        get_zone_to_zone_sharing_rates(output_dir, external_zone_system_network_dir=external_zone_system)