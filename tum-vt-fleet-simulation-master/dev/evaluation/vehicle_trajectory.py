import os
import sys
import numpy as np
import pandas as pd
import geopandas as gpd
import matplotlib.pyplot as plt
from shapely.geometry import Point

MAIN_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(MAIN_DIR)
from src.misc.plotting import plot_network_bg
from src.misc.globals import *
FIGSIZE = (12,12)
LIST_COLORS = [x for x in plt.rcParams['axes.prop_cycle'].by_key()['color']]
N_COLORS = len(LIST_COLORS)
LIST_RQ_COLORS = LIST_COLORS.copy()     # TODO # create larger list of colors
N_RQ_COLORS = len(LIST_RQ_COLORS)
EXTRA_BOUNDS = 0.25


def plot_vehicle_trajectory(output_dir, list_veh_id, operator_id=0, time_start=None, time_end=None, cut_bounds=True):
    """This function plots the trajectories of certain vehicles. It adds icons for request pick-ups and drop-offs
    along its trajectory.

    :param output_dir: result directory of simulation
    :param list_veh_id: list of vehicles that should be plotted
    :param operator_id: operator id of fleet vehicles
    :param time_start: all vehicle tasks before this time are ignored
    :param time_end: all vehicle tasks after this time are ignored
    :param cut_bounds: cut figure near boarding points instead of based on network
    :return: fig, ax1
    """
    # read scenario data
    scenario_parameters, list_operator_attributes, _ = load_scenario_inputs(output_dir)
    dir_names = get_directory_dict(scenario_parameters)
    if time_start is None:
        time_start = scenario_parameters[G_SIM_START_TIME]
    if time_end is None:
        time_end = scenario_parameters[G_SIM_END_TIME]
    # load network data
    nw_dir_name = dir_names[G_DIR_NETWORK]
    nw_base_dir = os.path.join(nw_dir_name, "base")
    nw_node_f = os.path.join(nw_base_dir, "nodes.csv")
    nw_node_df = pd.read_csv(nw_node_f, index_col=0)
    geojson_f = os.path.join(nw_base_dir, "edges_all_infos.geojson")
    if os.path.isfile(geojson_f):
        gdf = gpd.read_file(geojson_f)
        gdf["from_node"] = gdf["from_node"].astype(np.int64)
        gdf["to_node"] = gdf["to_node"].astype(np.int64)
        gdf.set_index(["from_node", "to_node"], inplace=True)
    # plot background map
    fig, ax1 = plt.subplots(figsize=FIGSIZE)
    plot_network_bg(nw_base_dir, ax1, filter_fast_roads=False, print_progress=True)
    # read and filter vehicle data
    veh_f = os.path.join(output_dir, f"2-{operator_id}_op-stats.csv")
    veh_df = pd.read_csv(veh_f)
    if G_VR_REPLAY_ROUTE not in veh_df.columns and G_VR_NODE_LIST not in veh_df.columns:
        raise AssertionError("Route and trajectory were not output in simulation!!")
    elif G_VR_NODE_LIST in veh_df.columns :
        use_trajectory = False
    else:
        use_trajectory = True
    f_df = veh_df[(veh_df[G_RQ_VID].isin(list_veh_id)) & (veh_df[G_VR_LEG_END_TIME] >= time_start) &
                  (veh_df[G_VR_LEG_START_TIME] <= time_end)].copy()
    # process data
    print("processing vehicle trajectories ...")
    counter = 0
    rq_counter = 0
    min_x = None
    max_x = None
    min_y = None
    max_y = None
    rq_cols = {}
    rq_index = {}
    rq_c = 1
    for vid, vid_df in f_df.groupby(G_RQ_VID):
        print(f"\t ... vid {vid}")
        c = LIST_COLORS[counter % N_COLORS]
        # 1) trajectory
        list_complete_nodes = []
        if use_trajectory:
            for traj_str in vid_df[G_VR_REPLAY_ROUTE].tolist():
                if pd.isnull(traj_str):
                    continue
                for entry in traj_str.split(";"):
                    node_id = int(entry.split(":")[0])
                    list_complete_nodes.append(node_id)
        else:
            for node_str in vid_df[G_VR_NODE_LIST].tolist():
                if pd.isnull(node_str):
                    continue
                for entry in node_str.split(";"):
                    node_id = int(entry)
                    list_complete_nodes.append(node_id)
        for i in range(len(list_complete_nodes)-1):
            o = list_complete_nodes[i]
            d = list_complete_nodes[i+1]
            try:
                tmp_gdf = gdf.iloc[gdf.index.get_locs([o,d])]
                if len(tmp_gdf) > 0:
                    tmp_gdf.plot(ax=ax1, color=c, lw=5)
                    bounds = tmp_gdf.bounds
                    if max_x is None or bounds["maxx"].max() > max_x:
                        max_x = bounds["maxx"].max()
                    if min_x is None or bounds["minx"].min() < min_x:
                        min_x = bounds["minx"].min()
                    if max_y is None or bounds["maxy"].max() > max_y:
                        max_y = bounds["maxy"].max()
                    if min_y is None or bounds["miny"].min() < min_y:
                        min_y = bounds["miny"].min()
            except:
                pass
        counter += 1
        # 2) boarding and disembarking requests
        b_vid_df = vid_df[vid_df[G_VR_STATUS] == "boarding"].copy()
        for position, pos_df in b_vid_df.groupby(G_VR_LEG_START_POS):
            rq_stop_counter = 0
            for _, row in pos_df.iterrows():
                node_id = int(row[G_VR_LEG_START_POS].split(";")[0])
                node_info = nw_node_df.loc[node_id]
                node_pos_x = node_info[G_NODE_X]
                if max_x is None or node_pos_x > max_x:
                    max_x = node_pos_x
                if min_x is None or node_pos_x < min_x:
                    min_x = node_pos_x
                node_pos_y = node_info[G_NODE_Y]
                if max_y is None or node_pos_y > max_y:
                    max_y = node_pos_y
                if min_y is None or node_pos_y < min_y:
                    min_y = node_pos_y
                # boarding
                b_rqs = row[G_VR_BOARDING_RID]
                if not pd.isnull(b_rqs):
                    list_rq = b_rqs.split(";")
                    for rq in list_rq:
                        if not rq_cols.get(rq):
                            # print(f"did not find color for rq {rq} in {rq_cols}")
                            rq_cols[rq] = LIST_RQ_COLORS[rq_counter % N_RQ_COLORS]
                            rq_counter += 1
                        if rq_index.get(rq, None) is None:
                            rq_index[rq] = rq_c
                            rq_c += 1
                        ax1.scatter([node_pos_x],[node_pos_y], c=rq_cols[rq], marker="o", s=200)
                        ax1.annotate(rq_index[rq], [node_pos_x, node_pos_y], xytext=(15,-5-15*rq_stop_counter),
                                     textcoords='offset points', size=18,color=rq_cols[rq], backgroundcolor="white")
                        rq_counter += 1
                        rq_stop_counter += 1
                # disembarking / alighting
                a_rqs = row[G_VR_ALIGHTING_RID]
                if not pd.isnull(a_rqs):
                    list_rq = a_rqs.split(";")
                    for k, rq in enumerate(list_rq):
                        if not rq_cols.get(rq):
                            # print(f"did not find color for rq {rq} in {rq_cols}")
                            rq_cols[rq] = LIST_RQ_COLORS[rq_counter % N_RQ_COLORS]
                            rq_counter += 1
                        if rq_index.get(rq, None) is None:
                            rq_index[rq] = rq_c
                            rq_c += 1
                        ax1.scatter([node_pos_x],[node_pos_y], c=rq_cols[rq], marker="x", s=200)
                        ax1.annotate(rq_index[rq], [node_pos_x, node_pos_y], xytext=(15,-5-15*rq_stop_counter),
                                     textcoords='offset points', size=18, color=rq_cols[rq], backgroundcolor="white")
                        rq_stop_counter += 1
    # cut figure to boarding point boundaries
    if cut_bounds:
        x_dim = max_x - min_x
        ax1.set_xlim(min_x - EXTRA_BOUNDS * x_dim, max_x + EXTRA_BOUNDS * x_dim)
        y_dim = max_y - min_y
        ax1.set_ylim(min_y - EXTRA_BOUNDS * y_dim, max_y + EXTRA_BOUNDS * y_dim)
    # saving figure
    vid_str = "-".join([str(x) for x in list_vid[:4]])
    if len(list_vid) <= 4:
        vid_str += "-ff"
    out_f = os.path.join(output_dir, f"routes_{vid_str}.png")
    fig.savefig(out_f, dpi=300, bbox_inches="tight")


if __name__ == "__main__":
    result_dir = r"C:\Users\ge37ser\Documents\tum-vt-fleet-simulation\studies\MOIA_ITSWC\results\SQ"
    list_vid = [72]
    list_list_vid = [[18], [21]]
    for list_vid in list_list_vid:
        rec_time_start = 496800
        rec_time_end = 498600
        plot_vehicle_trajectory(result_dir, list_vid, operator_id=0, time_start=rec_time_start, time_end=rec_time_end,
                                cut_bounds=True)
