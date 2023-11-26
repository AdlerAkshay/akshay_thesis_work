import os
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

MAIN_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(MAIN_DIR)
from src.misc.globals import *
FIG_SIZE = (9,6)
plt.style.use("seaborn-whitegrid")
WT_BIN_SIZE = 1
WT_BIN_MAX = 20
RDT_BIN_SIZE = 5
RDT_BIN_MAX = 20
TL_BIN_SIZE = 1
TL_BIN_MAX = 25
OP_ID = 0


def create_user_stat_distributions(output_dir):
    scenario_parameters, list_operator_attributes, _ = load_scenario_inputs(output_dir)
    operator_attributes = list_operator_attributes[OP_ID]
    boarding_time = operator_attributes[G_OP_CONST_BT]
    #
    user_f = os.path.join(output_dir, "1_user-stats.csv")
    user_df = pd.read_csv(user_f)
    #
    served_df = user_df[user_df["operator_id"] == 0].copy()
    served_df["waiting time"] = (served_df[G_RQ_PU] - served_df[G_RQ_TIME]) / 60
    served_df["relative detour"] = np.maximum(0, 100 * (served_df[G_RQ_DO] - served_df[G_RQ_PU] - boarding_time
                                                        - served_df[G_RQ_DRT]) / served_df[G_RQ_DRT])
    #
    print(served_df["waiting time"].describe())
    wt_min_bin = 0
    wt_max_bin = min(int(served_df["waiting time"].max() / WT_BIN_SIZE) + 1, WT_BIN_MAX)
    wt_bins = [WT_BIN_SIZE * x for x in range(wt_min_bin, wt_max_bin)]
    fig, ax1 = plt.subplots(figsize=FIG_SIZE)
    served_df["waiting time"].plot(kind="hist", bins=wt_bins, density=True, ax=ax1, rwidth=0.8)
    ax1.set_xlabel("Waiting Time [min]")
    fig.savefig("waiting_time_distribution.png")
    wt_dist_df = pd.DataFrame(np.histogram(served_df["waiting time"], bins=wt_bins, density=True))
    wt_dist_df.index = ["Frequency", "Bins"]
    tmp = wt_dist_df.loc["Frequency"].sum()
    if tmp != 1:
        wt_dist_df.loc["Frequency"] = wt_dist_df.loc["Frequency"] / tmp
    wt_dist_df.to_csv("waiting_time_distribution.csv")
    #
    print(served_df["relative detour"].describe())
    rdt_min_bin = 0
    rdt_max_bin = min(int(served_df["relative detour"].max() / RDT_BIN_SIZE) + 1, RDT_BIN_MAX)
    rdt_bins = [RDT_BIN_SIZE * x for x in range(rdt_min_bin, rdt_max_bin)]
    fig, ax1 = plt.subplots(figsize=FIG_SIZE)
    served_df["relative detour"].plot(kind="hist", bins=rdt_bins, density=True, ax=ax1, rwidth=0.8)
    ax1.set_xlabel("Relative Detour Time [%]")
    fig.savefig("relative_detour_distribution.png")
    rdt_dist_df = pd.DataFrame(np.histogram(served_df["relative detour"], bins=rdt_bins, density=True))
    rdt_dist_df.index = ["Frequency", "Bins"]
    tmp = rdt_dist_df.loc["Frequency"].sum()
    if tmp != 1:
        rdt_dist_df.loc["Frequency"] = rdt_dist_df.loc["Frequency"] / tmp
    rdt_dist_df.to_csv("relative_detour_distribution.csv")


def create_user_travel_distance_from_veh_stats(output_dir):
    veh_f = os.path.join(output_dir, f"2-{OP_ID}_op-stats.csv")
    veh_df = pd.read_csv(veh_f)
    veh_d_df = veh_df[veh_df["status"] == "route"].copy().reset_index()
    max_row_id = len(veh_d_df)
    rq_dist_dict = {}
    for row_id, row in veh_d_df.iterrows():
        if row_id % 10000 == 0:
            print(f"row {row_id}/{max_row_id}")
        rq_ob_str = row[G_VR_OB_RID]
        if not pd.isnull(rq_ob_str):
            vid = row[G_V_VID]
            dist = row[G_VR_LEG_DISTANCE] / 1000
            list_rq_ob = rq_ob_str.split(";")
            for rq in list_rq_ob:
                rq_vid_str = f"{rq};{vid}"
                try:
                    rq_dist_dict[rq_vid_str] += dist
                except KeyError:
                    rq_dist_dict[rq_vid_str] = dist
    trip_length_s = pd.Series(rq_dist_dict)
    trip_length_s.name = "Trip length [km]"
    trip_length_s.index.name = "Request ID;Vehicle ID"
    trip_length_s.to_csv("trip_length_full_data.csv")
    #
    print(trip_length_s.describe())
    tl_min_bin = 0
    tl_max_bin = min(int(trip_length_s.max() / TL_BIN_SIZE) + 1, TL_BIN_MAX)
    tl_bins = [TL_BIN_SIZE * x for x in range(tl_min_bin, tl_max_bin)]
    fig, ax1 = plt.subplots(figsize=FIG_SIZE)
    trip_length_s.plot(kind="hist", bins=tl_bins, density=True, ax=ax1, rwidth=0.8)
    ax1.set_xlabel("User Trip Length [km]")
    fig.savefig("trip_length_distribution.png")
    tl_dist_df = pd.DataFrame(np.histogram(trip_length_s, bins=tl_bins, density=True))
    tl_dist_df.index = ["Frequency", "Bins"]
    tmp = tl_dist_df.loc["Frequency"].sum()
    if tmp != 1:
        tl_dist_df.loc["Frequency"] = tl_dist_df.loc["Frequency"] / tmp
    tl_dist_df.to_csv("trip_length_distribution.csv")


if __name__ == "__main__":
    if len(sys.argv) == 2:
        result_dir = sys.argv[1]
        create_user_stat_distributions(result_dir)
        create_user_travel_distance_from_veh_stats(result_dir)
