import os
import sys
import glob
import datetime
import pandas as pd
import geopandas as gpd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm

MAIN_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(MAIN_DIR)
from src.misc.plotting import plot_network_bg, plot_od_arrows
from src.misc.globals import *

BOUND = 1000
TI_SIZE = 15*60
X_OFF = 1
DEF_BANDWIDTH = 1
DEF_METHOD = "V2"
# SCATTER_M_SIZE = 8
SCATTER_M_SIZE = 24


def _mark_vid_active(columns, inactive_dict, start_ti, max_ti):
    first_node_id = int(float(columns["start_pos"].split(";")[0]))
    if columns["first"] is True:
        for ti in range(start_ti, columns["ti"]):
            inactive_dict[ti] = first_node_id
    #
    last_node_id = int(float(columns["end_pos"].split(";")[0]))
    if columns["status"] == VRL_STATES.IDLE.display_name:
        pick_col = "ti"
    else:
        pick_col = "end_ti_top"
        # remove idle vehicle for time of activeness
        for ti in range(columns["ti"], columns[pick_col]):
            try:
                del inactive_dict[ti]
            except:
                pass
    range_bot = int(columns[pick_col])
    for ti in range(range_bot, max_ti):
        inactive_dict[ti] = last_node_id


def create_idle_veh_and_unserved_rq_df(output_dir):
    """This function evaluates idle vehicles and unserved requests per time interval on a node level

    :param output_dir: result directory
    :return: data frame with unserved and idle per ti for each node index with entries
    :rtype: DataFrame
    """
    idle_unserved_f = os.path.join(output_dir, "idle_and_unserved.csv")
    scenario_parameters, list_operator_attributes, _ = load_scenario_inputs(output_dir)
    dir_names = get_directory_dict(scenario_parameters)
    #
    sim_start_ti = int(np.floor(scenario_parameters[G_SIM_START_TIME] / TI_SIZE))
    sim_end_ti = int(np.ceil(scenario_parameters[G_SIM_END_TIME] / TI_SIZE))
    list_ti = range(sim_start_ti, sim_end_ti)
    nw_main_dir = dir_names[G_DIR_NETWORK]
    nw_base_dir = os.path.join(nw_main_dir, "base")
    scenario_name = scenario_parameters[G_SCENARIO_NAME]
    node_f = os.path.join(nw_base_dir, "nodes.csv")
    node_df = pd.read_csv(node_f, index_col=0)
    # Loading Simulation User Data
    # ----------------------------
    print("\t processing idle and unserved data")
    # a) unserved request data
    print("\t\tunserved request data ...")
    user_f = os.path.join(output_dir, f"1_user-stats.csv")
    user_df = pd.read_csv(user_f)
    list_operators = list(range(scenario_parameters[G_NR_OPERATORS]))
    not_served_df = user_df[~user_df["operator_id"].isin(list_operators)]
    not_served_df["ti"] = np.floor(not_served_df["rq_time"] / TI_SIZE).astype(np.int64)
    not_served_df["node_id"] = not_served_df["start"].apply(lambda x: x.split(";")[0])
    for ti_node_label, ti_node_df in not_served_df.groupby(["ti", "node_id"]):
        (ti_float, node_id_float) = ti_node_label
        ti = int(ti_float)
        if type(node_df.index.tolist()[0]) == int:
            node_id = int(node_id_float)
        else:
            node_id = str(int(node_id_float))
        node_df.loc[node_id, f"not served {ti}"] = ti_node_df.shape[0]
    # b) get idle
    print("\t\tget idle vehicle data ...")
    for op_id in range(scenario_parameters[G_NR_OPERATORS]):
        veh_f = os.path.join(output_dir, f"2-{op_id}_op-stats.csv")
        veh_df = pd.read_csv(veh_f)
        idle_at_nodes_full_ti = {}
        for ti in list_ti:
            idle_at_nodes_full_ti[ti] = {}
        vid_counter = 0
        for vid, tmp_df in veh_df.groupby("vehicle_id"):
            vid_counter += 1
            if vid_counter % 100 == 0:
                print(f"\t\t\tvid {vid_counter}")
            vid_df = tmp_df.sort_values("start_time").copy()
            # remove dict entry during active ti for current task
            # always write last position to all ti after current task
            vehicle_inactive = {}  # ti -> last position (node_id)
            vid_df["ti"] = np.floor(veh_df["start_time"] / TI_SIZE).astype(np.int64)
            vid_df["end_ti_top"] = np.ceil(veh_df["end_time"] / TI_SIZE).astype(np.int64)
            vid_df["first"] = [True] + [False] * (len(vid_df) - 1)
            vid_df.apply(_mark_vid_active, axis=1, args=(vehicle_inactive, sim_start_ti, sim_end_ti))
            for ti in list_ti:
                idle_node_id = vehicle_inactive.get(ti)
                if idle_node_id is not None:
                    try:
                        idle_at_nodes_full_ti[ti][idle_node_id] += 1
                    except:
                        idle_at_nodes_full_ti[ti][idle_node_id] = 1
        for ti in list_ti:
            col_name = f"op{op_id} idle {ti}"
            for k, v in idle_at_nodes_full_ti.get(ti, {}).items():
                node_df.loc[k, col_name] = v
    # saving data frame
    node_df.fillna(0, inplace=True)
    content_cols = [col for col in node_df.columns if (col.startswith("not served") or col.startswith("op"))]
    node_df["content"] = node_df[content_cols].sum(axis=1)
    node_df.reset_index(inplace=True)
    r_node_df = node_df[node_df["content"] != 0]
    r_node_df.to_csv(idle_unserved_f, index=False)
    return r_node_df


def create_idle_and_unserved_zone_lvl(output_dir, external_zone_system_network_dir=None, print_progress=True):
    """

    :param output_dir: result directory of scenario
    :param external_zone_system_network_dir: use external zone system for plot
    :param print_progress: print progress to shell
    :return: geo-data frame with number of idle and unserved requests per zone
    :rtype: GeoDataFrame
    """
    # load zone system
    scenario_parameters, list_operator_attributes, _ = load_scenario_inputs(output_dir)
    dir_names = get_directory_dict(scenario_parameters)
    #
    sim_start_ti = int(np.floor(scenario_parameters[G_SIM_START_TIME] / TI_SIZE))
    sim_end_ti = int(np.ceil(scenario_parameters[G_SIM_END_TIME] / TI_SIZE))
    list_ti = range(sim_start_ti, sim_end_ti)
    nw_main_dir = dir_names[G_DIR_NETWORK]
    nw_base_dir = os.path.join(nw_main_dir, "base")
    scenario_name = scenario_parameters[G_SCENARIO_NAME]
    if print_progress:
        print(f"plot_idle_and_unserved({scenario_name})")
    #
    idle_unserved_f = os.path.join(output_dir, "idle_and_unserved.csv")
    if os.path.isfile(idle_unserved_f):
        print("\t loading idle and unserved data")
        r_node_df = pd.read_csv(idle_unserved_f)
    else:
        r_node_df = create_idle_veh_and_unserved_rq_df(output_dir)
    zone_system_nw_dir = dir_names.get(G_DIR_ZONES)
    if external_zone_system_network_dir is not None:
        zone_system_nw_dir = external_zone_system_network_dir
    if not zone_system_nw_dir:
        raise IOError("This script requires zone information!")
    zone_system_main_dir = os.path.dirname(zone_system_nw_dir)
    zone_system_name = os.path.basename(os.path.dirname(zone_system_nw_dir))
    #
    n_z_f = os.path.join(zone_system_nw_dir, "node_zone_info.csv")
    n_z_df = pd.read_csv(n_z_f, index_col=0, squeeze=True)
    r_node_df["zone_id"] = r_node_df.apply(lambda x: n_z_df.loc[x["node_index"], "zone_id"], axis=1)
    #
    zs_pf = os.path.join(zone_system_main_dir, "polygon_definition.geojson")
    z_gdf = gpd.read_file(zs_pf)
    for zone_id, zr_id_us_df in r_node_df.groupby("zone_id"):
        for ti in list_ti:
            ns_col = f"not served {ti}"
            if ns_col in zr_id_us_df.columns:
                z_gdf.loc[zone_id, ns_col] = zr_id_us_df[ns_col].sum()
            else:
                z_gdf.loc[zone_id, ns_col] = 0
            for op_id in range(scenario_parameters[G_NR_OPERATORS]):
                idle_col = f"op{op_id} idle {ti}"
                if idle_col in zr_id_us_df.columns:
                    z_gdf.loc[zone_id, idle_col] = zr_id_us_df[idle_col].sum()
                else:
                    z_gdf.loc[zone_id, idle_col] = 0
    z_out_f = os.path.join(output_dir, f"idle_and_unserved_zones_{zone_system_name}.geojson")
    z_gdf.to_file(z_out_f, driver="GeoJSON")
    z_df = z_gdf.drop("geometry", axis=1)
    zdf_out_f = os.path.join(output_dir, f"idle_and_unserved_zones_{zone_system_name}.csv")
    z_df.to_csv(zdf_out_f)
    return z_gdf


def create_repo_od_zone_lvl(output_dir, external_zone_system_network_dir=None):
    """This function filters the vehicle stat files for repositioning trips and creates OD matrix entries by time
    interval.

    :param output_dir: result directory of scenario
    :param external_zone_system_network_dir: can be zone system which was not used for simulation
    :return: repo_df
    :rtype: DataFrame
    """
    # load zone system
    scenario_parameters, list_operator_attributes, _ = load_scenario_inputs(output_dir)
    dir_names = get_directory_dict(scenario_parameters)
    #
    sim_start_ti = int(np.floor(scenario_parameters[G_SIM_START_TIME] / TI_SIZE))
    sim_end_ti = int(np.ceil(scenario_parameters[G_SIM_END_TIME] / TI_SIZE))
    list_ti = range(sim_start_ti, sim_end_ti)
    nw_main_dir = dir_names[G_DIR_NETWORK]
    nw_base_dir = os.path.join(nw_main_dir, "base")
    scenario_name = scenario_parameters[G_SCENARIO_NAME]
    zone_system_nw_dir = dir_names.get(G_DIR_ZONES)
    if external_zone_system_network_dir is not None:
        zone_system_nw_dir = external_zone_system_network_dir
    if not zone_system_nw_dir:
        raise IOError("This script requires zone information!")
    zone_system_main_dir = os.path.dirname(zone_system_nw_dir)
    zone_system_name = os.path.basename(os.path.dirname(zone_system_nw_dir))
    #
    n_z_f = os.path.join(zone_system_nw_dir, "node_zone_info.csv")
    n_z_df = pd.read_csv(n_z_f, index_col=0, squeeze=True)
    #
    return_list_df = []
    list_veh_stat_f = glob.glob(f"{output_dir}/2-*_op-stats.csv")
    for veh_stat_f in list_veh_stat_f:
        op_id = os.path.basename(veh_stat_f).split("_")[0].split("-")[1]
        df = pd.read_csv(veh_stat_f)
        repo_df = df[df["status"] == VRL_STATES.REPOSITION.display_name]
        repo_df["ti"] = np.floor(repo_df[G_VR_LEG_START_TIME] / TI_SIZE).astype(np.int64)
        if len(repo_df) > 0:
            repo_df["o_zone"] = repo_df.apply(lambda x: n_z_df.loc[int(x["start_pos"].split(";")[0]), "zone_id"], axis=1)
            repo_df["d_zone"] = repo_df.apply(lambda x: n_z_df.loc[int(x["end_pos"].split(";")[0]), "zone_id"], axis=1)
        else:
            repo_df["o_zone"] = []
            repo_df["d_zone"] = []
        list_tod = []
        for tod_tuple, odt_df in repo_df.groupby(["ti", "o_zone", "d_zone"]):
            ti, o_zone, d_zone = tod_tuple
            odt_df["duration"] = odt_df[G_VR_LEG_END_TIME] - odt_df[G_VR_LEG_START_TIME]
            list_tod.append([ti, o_zone, d_zone, odt_df.shape[0], odt_df[G_VR_LEG_DISTANCE].mean(),
                             odt_df["duration"].mean()])
        repo_od_df = pd.DataFrame(list_tod, columns=["ti", "o_zone", "d_zone", "number_repo_trips", "mean_distance",
                                                     "mean_duration"])
        repo_out_f = os.path.join(output_dir, f"repositioning_od_op{op_id}_{zone_system_name}.csv")
        repo_od_df.to_csv(repo_out_f)
        return_list_df.append(repo_od_df)
    return return_list_df


def plot_idle_and_unserved_node_lvl(output_dir, external_nw_base_dir=None, print_progress=True):
    """This function creates a scatter plot of idle vehicles and unserved customers. The plots are created on a
    node level. Idle vehicles are slightly moved to the left and unserved customers slightly moved to the right to show
    both at once.
    
    :param output_dir: result directory of scenario
    :param external_nw_base_dir: external network base dir for background plot
    :param print_progress: print progress to shell
    :return: 
    """
    scenario_parameters, list_operator_attributes, _ = load_scenario_inputs(output_dir)
    dir_names = get_directory_dict(scenario_parameters)
    #
    sim_start_ti = int(np.floor(scenario_parameters[G_SIM_START_TIME] / TI_SIZE))
    sim_end_ti = int(np.ceil(scenario_parameters[G_SIM_END_TIME] / TI_SIZE))
    list_ti = range(sim_start_ti, sim_end_ti)
    nw_main_dir = dir_names[G_DIR_NETWORK]
    nw_base_dir = os.path.join(nw_main_dir, "base")
    scenario_name = scenario_parameters[G_SCENARIO_NAME]
    if print_progress:
        print(f"plot_idle_and_unserved({scenario_name})")
    #
    idle_unserved_f = os.path.join(output_dir, "idle_and_unserved.csv")
    if os.path.isfile(idle_unserved_f):
        print("\t loading idle and unserved data")
        r_node_df = pd.read_csv(idle_unserved_f)
    else:
        r_node_df = create_idle_veh_and_unserved_rq_df(output_dir)
    # create plots
    print("\t creating plots")
    list_unserved_cols = [col for col in r_node_df.columns if col.startswith("not served")]
    max_unserved = r_node_df[list_unserved_cols].max().max()
    norm_unserved = cm.colors.Normalize(vmax=max_unserved, vmin=0)
    cmap_unserved = "autumn_r"
    mappable_unserved = plt.cm.ScalarMappable(norm_unserved, cmap_unserved)
    list_idle_cols = [col for col in r_node_df.columns if col.startswith("op")]
    max_idle = r_node_df[list_idle_cols].max().max()
    norm_idle = cm.colors.Normalize(vmax=max_idle, vmin=0)
    cmap_idle = "cool"
    mappable_idle = plt.cm.ScalarMappable(norm_idle, cmap_idle)
    #
    n_gdf = gpd.GeoDataFrame(r_node_df, geometry=gpd.points_from_xy(r_node_df["pos_x"], r_node_df["pos_y"]))
    minx, miny, maxx, maxy = n_gdf.geometry.total_bounds
    # delta_x = min(X_OFF * (maxx - minx) / 100, 5)
    idle_out_dir = os.path.join(output_dir, "dir_idle_and_unserved")
    if not os.path.isdir(idle_out_dir):
        os.mkdir(idle_out_dir)
    number_ti = len(list_ti)
    counter = 0
    for ti in list_ti:
        plot = False
        counter += 1
        print(f"\t\t plotting time interval {ti} (number {counter}/{number_ti})")
        fig, ax1 = plt.subplots(figsize=(16,8))
        if external_nw_base_dir is not None:
            ax1 = plot_network_bg(external_nw_base_dir, ax1, print_progress=False)
        else:
            ax1 = plot_network_bg(nw_base_dir, ax1, print_progress=False, filter_fast_roads=True)
        ax1.set_xlim(xmin=minx-BOUND, xmax=maxx+BOUND)
        ax1.set_ylim(ymin=miny-BOUND, ymax=maxy+BOUND)
        # offset = lambda p: transforms.ScaledTranslation(p / 72., 0, plt.gcf().dpi_scale_trans)
        # trans = plt.gca().transData
        # TODO # plot calls
        ns_ti = f"not served {ti}"
        if ns_ti in n_gdf.columns:
            # n_gdf.plot(ns_ti, ax=ax1, transform=trans+offset(-X_OFF), cmap=cmap_unserved, norm=norm_unserved)
            tmp_gdf = n_gdf[n_gdf[ns_ti] > 0]
            tmp_gdf.plot(ns_ti, ax=ax1, marker=8, cmap=cmap_unserved, norm=norm_unserved, markersize=SCATTER_M_SIZE)
            plot = True
        # only operator 0
        i0_ti = f"op0 idle {ti}"
        if i0_ti in n_gdf.columns:
            tmp_gdf = n_gdf[n_gdf[i0_ti] > 0]
            tmp_gdf.plot(i0_ti, ax=ax1, marker=9, cmap=cmap_idle, norm=norm_idle, markersize=SCATTER_M_SIZE)
            plot = True
        if plot:
            t0 = datetime.timedelta(seconds=ti * TI_SIZE)
            t1 = datetime.timedelta(seconds=(ti+1) * TI_SIZE)
            time_str = f"{t0} - {t1}"
            ax1.set_title(f"Unserved Requests and Idle Vehicles {time_str}", fontsize=16)
            cbar_unserved = fig.colorbar(mappable_unserved, ax=ax1, pad=-0.05)
            # cbar_unserved.set_label("unserved requests", size=14, labelpad=-40, y=1.05, rotation=0)
            cbar_unserved.set_label("unserved requests", size=14)
            cbar_idle = fig.colorbar(mappable_idle, ax=ax1, pad=0.01)
            cbar_idle.set_label("idle vehicles", size=14)
            # cbar_idle.set_label("idle vehicles", size=14, labelpad=-40, y=1.05, rotation=0)
            out_f = os.path.join(idle_out_dir, f"idle_and_unserved_{ti}.png")
            fig.savefig(out_f, bbox_inches="tight")
        plt.close(fig)


def plot_idle_and_unserved_density(output_dir, external_zone_system_network_dir=None, external_nw_base_dir=None,
                                   print_progress=True):
    """This function can be used to visualize idle vehicles and unserved demand on a zone level and repositioning trips
    between zones.

    # visualization
    # -------------
    # color of area -> unserved request density
    # radius of circle in centroid -> number idle vehicles
    # OD plot of repositioning trips starting in this time frame

    :param output_dir:
    :param external_zone_system_network_dir:
    :param external_nw_base_dir:
    :param print_progress:
    :return:
    """
    scenario_parameters, list_operator_attributes, _ = load_scenario_inputs(output_dir)
    dir_names = get_directory_dict(scenario_parameters)
    nw_main_dir = dir_names[G_DIR_NETWORK]
    nw_base_dir = os.path.join(nw_main_dir, "base")
    sim_start_ti = int(np.floor(scenario_parameters[G_SIM_START_TIME] / TI_SIZE))
    sim_end_ti = int(np.ceil(scenario_parameters[G_SIM_END_TIME] / TI_SIZE))
    list_ti = range(sim_start_ti, sim_end_ti)
    zone_system_nw_dir = dir_names.get(G_DIR_ZONES)
    if external_zone_system_network_dir is not None:
        zone_system_nw_dir = external_zone_system_network_dir
    if not zone_system_nw_dir:
        raise IOError("This script requires zone information!")
    zone_system_dir = os.path.dirname(zone_system_nw_dir)
    zone_system_name = os.path.basename(zone_system_dir)
    # read zone-lvl idle and unserved
    z_out_f = os.path.join(output_dir, f"idle_and_unserved_zones_{zone_system_name}.geojson")
    if os.path.isfile(z_out_f):
        z_iv_ur_gdf = gpd.read_file(z_out_f)
    else:
        z_iv_ur_gdf = create_idle_and_unserved_zone_lvl(output_dir, external_zone_system_network_dir)
    # read zone-to-zone OD matrix for operator 0
    op_id = 0
    repo_out_f = os.path.join(output_dir, f"repositioning_od_op{op_id}_{zone_system_name}.csv")
    if os.path.isfile(repo_out_f):
        repo_od_df = pd.read_csv(repo_out_f)
    else:
        tmp = create_repo_od_zone_lvl(output_dir, external_zone_system_network_dir=external_zone_system_network_dir)
        repo_od_df = tmp[0]
    #
    zlvl_output_dir = os.path.join(output_dir, f"dir_{zone_system_name}_idle_unserved_repos")
    if not os.path.isdir(zlvl_output_dir):
        os.mkdir(zlvl_output_dir)
    print("\t creating plots")
    minx, miny, maxx, maxy = z_iv_ur_gdf.geometry.total_bounds
    if pd.isnull(minx):
        minx = z_iv_ur_gdf.geometry.bounds.minx.min()
        miny = z_iv_ur_gdf.geometry.bounds.miny.min()
        maxx = z_iv_ur_gdf.geometry.bounds.maxx.min()
        maxy = z_iv_ur_gdf.geometry.bounds.maxy.min()
    #
    z_iv_ur_gdf["area"] = z_iv_ur_gdf["geometry"].area / 10**6  # area in km^2
    list_unserved_cols = [col for col in z_iv_ur_gdf.columns if col.startswith("not served")]
    us_per_area_df = z_iv_ur_gdf[["geometry"]]
    for col in list_unserved_cols:
        z_iv_ur_gdf[col].fillna(0, inplace=True)
        us_per_area_df[col] = z_iv_ur_gdf[col] / z_iv_ur_gdf["area"]
    max_unserved = us_per_area_df[list_unserved_cols].max().max()
    norm_unserved = cm.colors.Normalize(vmax=max_unserved, vmin=0)
    cmap_unserved = "Reds"
    mappable_unserved = plt.cm.ScalarMappable(norm_unserved, cmap_unserved)
    #
    list_idle_cols = [col for col in z_iv_ur_gdf.columns if col.startswith("op")]
    max_idle = z_iv_ur_gdf[list_idle_cols].max().max()
    norm_idle = cm.colors.Normalize(vmax=max_idle, vmin=0)
    cmap_idle = "cool"
    mappable_idle = plt.cm.ScalarMappable(norm_idle, cmap_idle)
    #
    centroid_series = z_iv_ur_gdf["geometry"].centroid
    max_repo = repo_od_df["number_repo_trips"].max()
    norm_repo = cm.colors.Normalize(vmax=max_repo, vmin=0)
    cmap_repo = "Greys"
    mappable_repo = plt.cm.ScalarMappable(norm_repo, cmap_repo)
    dict_repo_gdf = {}
    for ti, ti_repo_od_df in repo_od_df.groupby("ti"):
        dict_repo_gdf[ti] = ti_repo_od_df
    #
    number_ti = len(list_ti)
    counter = 0
    for ti in list_ti:
        counter += 1
        print(f"\t\t plotting time interval {ti} (number {counter}/{number_ti})")
        fig, ax1 = plt.subplots(figsize=(16,8))
        # if external_nw_base_dir is not None:
        #     ax1 = plot_network_bg(external_nw_base_dir, ax1, print_progress=False)
        # else:
        #     ax1 = plot_network_bg(nw_base_dir, ax1, print_progress=False, filter_fast_roads=True)
        # ax1.set_xlim(xmin=minx-BOUND, xmax=maxx+BOUND)
        # ax1.set_ylim(ymin=miny-BOUND, ymax=maxy+BOUND)
        ns_ti = f"not served {ti}"
        if ns_ti not in us_per_area_df.columns:
            us_per_area_df[ns_ti] = 0
        us_per_area_df.plot(column=ns_ti, ax=ax1, cmap=cmap_unserved, norm=norm_unserved, alpha=0.5, edgecolor='black')
        #
        # only operator 0
        i0_ti = f"op0 idle {ti}"
        if i0_ti in z_iv_ur_gdf.columns:
            tmp_gdf = z_iv_ur_gdf[z_iv_ur_gdf[i0_ti] > 0].copy()
            tmp_gdf["centroid"] = tmp_gdf["geometry"].centroid
            tmp_gdf.rename({"geometry":"polygon"}, axis=1, inplace=True)
            tmp_gdf.rename({"centroid":"geometry"}, axis=1, inplace=True)
            tmp_gdf.plot(column=i0_ti, ax=ax1, marker="o", cmap=cmap_idle, norm=norm_idle, markersize=16)
        #
        o_kw = "o_zone"
        d_kw = "d_zone"
        v_kw = "number_repo_trips"
        repo_ti_gdf = dict_repo_gdf.get(ti)
        if repo_ti_gdf is not None:
            ax1 = plot_od_arrows(ax1, repo_ti_gdf, centroid_series, o_kw, d_kw, v_kw, mappable=mappable_repo)
        #
        t0 = datetime.timedelta(seconds=ti * TI_SIZE)
        t1 = datetime.timedelta(seconds=(ti + 1) * TI_SIZE)
        time_str = f"{t0} - {t1}"
        ax1.set_title(f"Unserved Requests and Idle Vehicles {time_str}", fontsize=16)
        cbar_unserved = fig.colorbar(mappable_unserved, ax=ax1, pad=-0.05)
        cbar_unserved.set_label("unserved requests per square km", size=14)
        cbar_idle = fig.colorbar(mappable_idle, ax=ax1, pad=0.01)
        cbar_idle.set_label("idle vehicles", size=14)
        #
        out_f = os.path.join(zlvl_output_dir, f"ns_idle_repo_{ti}.png")
        fig.savefig(out_f, bbox_inches="tight")
        plt.close(fig)


if __name__ == "__main__":
    if len(sys.argv) in [2, 3]:
        res_dir = sys.argv[1]
        if len(sys.argv) == 3:
            ext_nw_base_dir = sys.argv[2]
        else:
            ext_nw_base_dir = None
        print(f"evaluation of {res_dir}")
    else:
        res_dir = r"C:\Users\ne53qez\Data_and_Simulation\tum-vt-fleet-simulation\studies\Flo_PhD_Operations\results\s1_5_1750_HailIRS_-_-_-_1"
        ext_nw_base_dir = r"C:\Users\ne53qez\Data_and_Simulation\tum-vt-fleet-simulation\data\networks\plot_muc\base"
        ext_zone_system_network_dir = r"C:\Users\ne53qez\Data_and_Simulation\tum-vt-fleet-simulation\data\zones\muc_a99_max4km_4lvl_mivoev\MUNbene_withBPs_300_1_LHMArea_OVstations_reduced_Flo"
    plot_idle_and_unserved_node_lvl(res_dir, external_nw_base_dir=ext_nw_base_dir)
    plot_idle_and_unserved_density(res_dir, external_zone_system_network_dir=ext_zone_system_network_dir,
                                   external_nw_base_dir=ext_nw_base_dir)
