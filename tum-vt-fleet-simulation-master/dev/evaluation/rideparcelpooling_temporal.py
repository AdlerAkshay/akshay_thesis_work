import os
import sys
import pandas as pd
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

MAIN_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(MAIN_DIR)
from src.evaluation.standard import decode_offer_str, load_scenario_inputs, get_directory_dict,\
                                    read_op_output_file, read_user_output_file
from src.evaluation.temporal import _load_op_stats_and_infer_idle_states
from src.misc.globals import *

# plt.style.use("seaborn-whitegrid")
# matplotlib.rcParams['ps.useafm'] = True
# matplotlib.rcParams['pdf.use14corefonts'] = True
#matplotlib.rcParams['text.usetex'] = True

# assuming temporal resolution of seconds in output files
MIN = 60
HOUR = 3600
DAY = 24 * 3600
DEF_TEMPORAL_RESOLUTION = 15*MIN
DEF_SMALLER_TEMPORAL_RESOLUTION = 2*MIN # for stacked plots

# styling figures
FIG_SIZE = (6,4)
LABEL_FONT_SIZE = 14
LIST_COLORS = [x for x in plt.rcParams['axes.prop_cycle'].by_key()['color']]
#LIST_COLORS = ['#DDDBD3', '#E6AA33', '#BC8820', '#4365FF', '#0532FF', '#8C9296', '#000C15'] # MOIA colors
N_COLORS = len(LIST_COLORS)

# other
SMALL_VALUE = 0.000001

def _create_occ_parcel_plot_lists(op_df, max_occupancy):
    """
    possible states: inactive, empty driving, with person(s), with person(s) and parcels, only with parcels
    """
    non_action_states = ["out_of_service", "charging", "idle"]
    times = {}
    for t in op_df[G_VR_LEG_START_TIME].values:
        times[t] = 1
    for t in op_df[G_VR_LEG_END_TIME].values:
        times[t] = 1
    times = sorted([t for t in times.keys()])
    new_times = []
    for i, t in enumerate(times):
        if i == 0:
            new_times.append(t)
            continue
        while t - new_times[-1] > DEF_SMALLER_TEMPORAL_RESOLUTION:
            new_times.append(new_times[-1] + DEF_SMALLER_TEMPORAL_RESOLUTION)
        new_times.append(t)
    times = new_times
    state_changes = [[], [], [], [], []]
    for vid, vid_df in op_df.groupby(G_V_VID):
        for start_time, end_time, vrl_state, rq_boarding, rq_alighting, rq_on_board in \
                    zip(vid_df["start_time"].values, vid_df["end_time"].values, vid_df["status"].values, vid_df["rq_boarding"].values, vid_df["rq_alighting"].values, vid_df["rq_on_board"].values):
            if vrl_state in non_action_states:
                state = 4
            else:
                if rq_on_board != rq_on_board:
                    ob_list = []
                else:
                    ob_list = str(rq_on_board).split(";")
                if rq_boarding != rq_boarding:
                    bd_list = []
                else:
                    bd_list = str(rq_boarding).split(";")
                if len(ob_list) == 0 and len(bd_list) == 0:
                    state = 0
                else:
                    parcel_ob = False
                    cust_ob = False
                    for rq_id in ob_list + bd_list:
                        if rq_id.startswith("p"):
                            parcel_ob = True
                        else:
                            cust_ob = True
                    if cust_ob:
                        if parcel_ob:
                            state = 2
                        else:
                            state = 1
                    else:
                        if parcel_ob:
                            state = 3
                        else:
                            state = 2
            state_changes[state].append( (start_time, +1) )
            state_changes[state].append( (end_time, -1) )
       
    plotvalues = {}
    state_to_plot_str = {0 : "empty", 1 : "users onboard", 2 : "users + parcels onboard", 3 : "parcels onboard", 4 : "inactive"}     
    for state, state_list in enumerate(state_changes):
        together = sorted(state_list, key = lambda x:x[0])
        y = []
        n = 0
        together_index = 0
        for t in times:
            while together_index < len(together) and t >= together[together_index][0]:
                n += together[together_index][1]
                together_index += 1
            y.append(n)
        plotvalues[state_to_plot_str[state]] = y
                    
    return times, list(plotvalues.values()), list(plotvalues.keys())

def parcel_occ_over_time(output_dir, scenario_parameters, operator_attributes, dir_names, op_id, show=False,
                        bin_size=DEF_TEMPORAL_RESOLUTION, evaluation_start_time=None, evaluation_end_time=None):
    """ this function creates a plot of the different vehicle occupancies over time
    :param ouput_dir: directory of scenario results
    :param scenario_parameters: scenario parameter dictionary
    :param operator_attributes: op atts dictionary
    :param dir_names: dir name dictionary
    :param op_id: operator id
    :param show: if True, plot is shown but not stored; if false otherwise
    :param bin_size: bin size of plot
    :param evaluation_start_time: start time of the evaluation time interval
    :param evaluation_end_time: end time of the evaluation time interval
    """
    # load operator stats and infor idle times
    op_df = _load_op_stats_and_infer_idle_states(output_dir, scenario_parameters, op_id,
                                                 evaluation_start_time=evaluation_start_time,
                                                 evaluation_end_time=evaluation_end_time)
    # create number of occupancy stats at different time stamps
    max_occupancy = op_df[op_df[G_VR_STATUS] == "route"][G_VR_NR_PAX].max()
    times, values, occupancy = _create_occ_parcel_plot_lists(op_df, max_occupancy)
    # additional binning
    if bin_size is not None:
        new_times = []
        new_indices = []
        last_time = None
        for i, t in enumerate(times):
            if i == 0:
                last_time = t
                new_times.append(t)
                new_indices.append(i)
            else:
                if t - last_time >= bin_size:
                    if t - times[i-1] >= bin_size:
                        new_times.append(times[i-1])
                        new_indices.append(i-1)
                    new_times.append(t)
                    new_indices.append(i)
                    last_time = t
        times = new_times
        for j, value in enumerate(values):
            new_value = []
            for i in new_indices:
                new_value.append(value[i])
            values[j] = new_value

    # create plots
    number_vehicles = len(op_df[G_V_VID].unique())
    start_time = scenario_parameters[G_SIM_START_TIME]
    plt.figure(figsize=(10,10))
    plt.stackplot(times, values, labels=occupancy, colors=LIST_COLORS)
    plt.legend(title='Occupancy States', bbox_to_anchor=(1.04,0.5), loc="center left", borderaxespad=0)
    plt.xlabel("Time [h]")
    plt.ylabel("Accumulated Number of Vehicles")
    # labels=[f"2019/{i+18}/11" for i in range(int(times[-1]/3600/24))]
    plt.xticks([i for i in range(start_time, int(times[-1]), 3600*12)],
               labels=[f"{int(i/3600)}" for i in range(start_time, int(times[-1]), 3600*12)])
    if evaluation_start_time is None and evaluation_end_time is None:
        plt.xlim(start_time, times[-1])
    else:
        s = start_time
        if evaluation_start_time is not None:
            s = evaluation_start_time
        e = times[-1]
        if evaluation_end_time is not None:
            e = evaluation_end_time
        plt.xlim(s, e)
    plt.ylim(0, number_vehicles)
    # plt.tight_layout(rect=[0,0,0.75,1])
    if show:
        plt.show()
    else:
        plt.savefig(os.path.join(output_dir, "fleet_rpp_states_time_op_{}.png".format(op_id)), bbox_inches="tight")
        plt.close()
    return times, values

def _create_occ_only_parcel_plot_lists(op_df, max_parcel_occupancy, parcel_size_dict = {}):
    """
    parcel_size_dict: rq_id -> parcel size
    possible states: inactive, empty driving, parcel_occupancies, only_persons
    """
    non_action_states = ["out_of_service", "charging", "idle"]
    times = {}
    for t in op_df[G_VR_LEG_START_TIME].values:
        times[t] = 1
    for t in op_df[G_VR_LEG_END_TIME].values:
        times[t] = 1
    times = sorted([t for t in times.keys()])
    new_times = []
    for i, t in enumerate(times):
        if i == 0:
            new_times.append(t)
            continue
        while t - new_times[-1] > DEF_SMALLER_TEMPORAL_RESOLUTION:
            new_times.append(new_times[-1] + DEF_SMALLER_TEMPORAL_RESOLUTION)
        new_times.append(t)
    times = new_times
    state_changes = [[] for i in range(max_parcel_occupancy + 2)]
    for vid, vid_df in op_df.groupby(G_V_VID):
        for start_time, end_time, vrl_state, rq_boarding, rq_alighting, rq_on_board in \
                    zip(vid_df["start_time"].values, vid_df["end_time"].values, vid_df["status"].values, vid_df["rq_boarding"].values, vid_df["rq_alighting"].values, vid_df["rq_on_board"].values):
            if vrl_state in non_action_states:
                state = -1
            else:
                if rq_on_board != rq_on_board:
                    ob_list = []
                else:
                    ob_list = str(rq_on_board).split(";")
                if rq_boarding != rq_boarding:
                    bd_list = []
                else:
                    bd_list = str(rq_boarding).split(";")
                if len(ob_list) == 0 and len(bd_list) == 0:
                    state = 0
                else:
                    parcel_obs = [x for x in set(ob_list + bd_list) if x.startswith("p")]
                    n_parcel_obs = sum([parcel_size_dict.get(pid, 1) for pid in parcel_obs])
                    if n_parcel_obs == 0:
                        state = -2
                    else:
                        state = n_parcel_obs
            if state >= max_parcel_occupancy:
                print(state)
                state = max_parcel_occupancy - 1
            state_changes[state].append( (start_time, +1) )
            state_changes[state].append( (end_time, -1) )
       
    plotvalues = {}
    sp_tuple = [(i , str(i)) for i in range(max_parcel_occupancy)]
    sp_tuple += [(max_parcel_occupancy, "only passengers"), (max_parcel_occupancy + 1, "idle")]
    state_to_plot_str = {k : v for k, v in sp_tuple}     
    for state, state_list in enumerate(state_changes):
        together = sorted(state_list, key = lambda x:x[0])
        y = []
        n = 0
        together_index = 0
        for t in times:
            while together_index < len(together) and t >= together[together_index][0]:
                n += together[together_index][1]
                together_index += 1
            y.append(n)
        plotvalues[state_to_plot_str[state]] = y
                    
    return times, list(plotvalues.values()), list(plotvalues.keys())

def parcel_occ_states_over_time(output_dir, scenario_parameters, operator_attributes, dir_names, op_id, show=False,
                        bin_size=DEF_TEMPORAL_RESOLUTION, evaluation_start_time=None, evaluation_end_time=None):
    """ this function creates a plot of the different vehicle occupancies over time
    :param ouput_dir: directory of scenario results
    :param scenario_parameters: scenario parameter dictionary
    :param operator_attributes: op atts dictionary
    :param dir_names: dir name dictionary
    :param op_id: operator id
    :param show: if True, plot is shown but not stored; if false otherwise
    :param bin_size: bin size of plot
    :param evaluation_start_time: start time of the evaluation time interval
    :param evaluation_end_time: end time of the evaluation time interval
    """
    # load operator stats and infor idle times
    op_df = _load_op_stats_and_infer_idle_states(output_dir, scenario_parameters, op_id,
                                                 evaluation_start_time=evaluation_start_time,
                                                 evaluation_end_time=evaluation_end_time)
    # create number of occupancy stats at different time stamps
    max_parcel_occupancy = 9
    times, values, occupancy = _create_occ_only_parcel_plot_lists(op_df, max_parcel_occupancy)
    # additional binning
    if bin_size is not None:
        new_times = []
        new_indices = []
        last_time = None
        for i, t in enumerate(times):
            if i == 0:
                last_time = t
                new_times.append(t)
                new_indices.append(i)
            else:
                if t - last_time >= bin_size:
                    if t - times[i-1] >= bin_size:
                        new_times.append(times[i-1])
                        new_indices.append(i-1)
                    new_times.append(t)
                    new_indices.append(i)
                    last_time = t
        times = new_times
        for j, value in enumerate(values):
            new_value = []
            for i in new_indices:
                new_value.append(value[i])
            values[j] = new_value

    # create plots
    number_vehicles = len(op_df[G_V_VID].unique())
    start_time = scenario_parameters[G_SIM_START_TIME]
    plt.figure(figsize=(10,10))
    plt.stackplot(times, values, labels=occupancy, colors=LIST_COLORS)
    plt.legend(title='Parcel Occupancy States', bbox_to_anchor=(1.04,0.5), loc="center left", borderaxespad=0)
    plt.xlabel("Time [h]")
    plt.ylabel("Accumulated Number of Vehicles")
    # labels=[f"2019/{i+18}/11" for i in range(int(times[-1]/3600/24))]
    plt.xticks([i for i in range(start_time, int(times[-1]), 3600*12)],
               labels=[f"{int(i/3600)}" for i in range(start_time, int(times[-1]), 3600*12)])
    if evaluation_start_time is None and evaluation_end_time is None:
        plt.xlim(start_time, times[-1])
    else:
        s = start_time
        if evaluation_start_time is not None:
            s = evaluation_start_time
        e = times[-1]
        if evaluation_end_time is not None:
            e = evaluation_end_time
        plt.xlim(s, e)
    plt.ylim(0, number_vehicles)
    # plt.tight_layout(rect=[0,0,0.75,1])
    if show:
        plt.show()
    else:
        plt.savefig(os.path.join(output_dir, "fleet_rpp_parcel_occ_states_time_op_{}.png".format(op_id)), bbox_inches="tight")
        plt.close()
    return times, values

# -------------------------------------------------------------------------------------------------------------------- #
# main script call
def run_complete_rpp_temporal_evaluation(output_dir, evaluation_start_time=None, evaluation_end_time=None, show=False):
    """This method creates all plots for fleet, user, network and pt KPIs for a given scenario and saves them in
    the respective output directory. Furthermore, it creates a file 'temporal_eval.csv' containing the time series data.
    These can be used for scenario comparisons.

    :param output_dir: output directory of a scenario
    :type output_dir: str
    :param evaluation_start_time: start time of evaluation
    :param evaluation_end_time: end time of evaluation
    """
    scenario_parameters, list_operator_attributes, _ = load_scenario_inputs(output_dir)
    dir_names = get_directory_dict(scenario_parameters)
    for op_id, op_attributes in enumerate(list_operator_attributes):
        parcel_occ_states_over_time(output_dir, scenario_parameters, op_attributes, dir_names, op_id, evaluation_start_time=evaluation_start_time, evaluation_end_time=evaluation_end_time, show=show)
        parcel_occ_over_time(output_dir, scenario_parameters, op_attributes, dir_names, op_id, evaluation_start_time=evaluation_start_time, evaluation_end_time=evaluation_end_time, show=show)
