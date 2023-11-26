import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from src.evaluation.standard import *

def standard_rideparcelpooling_evaluation(output_dir, evaluation_start_time = None,
                                          evaluation_end_time = None, print_comments=False, dir_names_in = {}):
    """This function runs a standard evaluation over a scenario output directory.

    :param output_dir: scenario output directory
    :param start_time: start time of evaluation interval in s (if None, then evaluation of all data from sim start)
    :param end_time: end time of evaluation interval in s   (if None, then evalation of all data until sim end)
    :param print_comments: print some comments about status in between
    """
    if not os.path.isdir(output_dir):
        raise IOError(f"Could not find result directory {output_dir}!")
    
    scenario_parameters, list_operator_attributes, _ = load_scenario_inputs(output_dir)
    dir_names = get_directory_dict(scenario_parameters)
    if dir_names_in:
        dir_names = dir_names_in

    # evaluation interval
    if evaluation_start_time is None and scenario_parameters.get(G_EVAL_INT_START) is not None:
        evaluation_start_time = int(scenario_parameters[G_EVAL_INT_START])
    if evaluation_end_time is None and scenario_parameters.get(G_EVAL_INT_END) is not None:
        evaluation_end_time = int(scenario_parameters[G_EVAL_INT_END])

    # vehicle type data
    # veh_type_db = create_vehicle_type_db(dir_names[G_DIR_VEH])
    # veh_type_stats = pd.read_csv(os.path.join(output_dir, "2_vehicle_types.csv"))

    if print_comments:
        print(f"Evaluating {scenario_parameters[G_SCENARIO_NAME]}\nReading user stats ...")
    user_stats = read_user_output_file(output_dir, evaluation_start_time=evaluation_start_time, evaluation_end_time=evaluation_end_time)
    if print_comments:
        print(f"\t shape of user stats: {user_stats.shape}")
    if scenario_parameters.get(G_PA_RQ_FILE):
        parcel_demand_df = pd.read_csv(os.path.join(dir_names[G_DIR_PARCEL_DEMAND], scenario_parameters[G_PA_RQ_FILE]))
        def add_p(row):
            return f"p_{row['request_id']}"
        parcel_demand_df["request_id"] = parcel_demand_df.apply(add_p, axis=1)
        parcel_demand_df.set_index("request_id", inplace=True)
    else:
        parcel_demand_df = pd.DataFrame(columns=["start","end","parcel_size","request_id","rq_time"])
    if not "parcel_size" in parcel_demand_df.columns:
        parcel_demand_df["parcel_size"] = 1
    parcel_sizes = parcel_demand_df["parcel_size"].to_dict()

    # add passengers columns where necessary
    if G_RQ_ID not in user_stats.columns:
        user_stats[G_RQ_ID] = 1

    row_id_to_offer_dict = {}    # user_stats_row_id -> op_id -> offer
    op_id_to_offer_dict = {}    # op_id -> user_stats_row_id -> offer
    active_offer_parameters = {}
    for key, entries in user_stats.iterrows():
        row_id_to_offer_dict[key] = entries[G_RQ_PAX]
        offer_entry = entries[G_RQ_OFFERS]
        offer = decode_offer_str(offer_entry)
        row_id_to_offer_dict[key] = offer
        for op_id, op_offer in offer.items():
            try:
                op_id_to_offer_dict[op_id][key] = op_offer
            except KeyError:
                op_id_to_offer_dict[op_id] = {key : op_offer}
            for offer_param in op_offer.keys():
                active_offer_parameters[offer_param] = 1

    rpp_eval_series = pd.Series(name="MoD_0")
    
    parcel_users = user_stats[user_stats["rq_type"] == "BasicParcelRequest"]
    person_users = user_stats[user_stats["rq_type"] == "BasicRequest"]
    number_parcels = parcel_users.shape[0]
    
    served_parcels = parcel_users[parcel_users["dropoff_time"].notna()]
    pu_parcels = parcel_users[parcel_users["operator_id"] == 0]
    n_pu_parcels = pu_parcels.shape[0]
    n_served_parcels = served_parcels.shape[0]
    n_unserved_parcels = number_parcels - n_served_parcels
    
    n_abs_parcels = sum([parcel_sizes[pid] for pid in parcel_users["request_id"].values])
    n_abs_pu_parcels = sum([parcel_sizes[pid] for pid in pu_parcels["request_id"].values])
    n_abs_served_parcels = sum([parcel_sizes[pid] for pid in served_parcels["request_id"].values])
    n_abs_unserved_parcels = n_abs_parcels - n_abs_served_parcels
    
    served_persons = person_users[person_users["operator_id"] == 0]
    unserved_persons = person_users[person_users["operator_id"] != 0]
    n_served_persons = served_persons.shape[0]
    n_unserved_persons = person_users.shape[0] - n_served_persons
    
    rpp_eval_series["served parcels"] = n_served_parcels
    rpp_eval_series["unserved parcels"] = n_unserved_parcels
    rpp_eval_series["picked up parcels"] = n_pu_parcels
    
    rpp_eval_series["abs served parcels"] = n_abs_served_parcels
    rpp_eval_series["abs unserved parcels"] = n_abs_unserved_parcels
    rpp_eval_series["abs picked up parcels"] = n_abs_pu_parcels
    
    rpp_eval_series["served persons"] = n_served_persons
    rpp_eval_series["unserved persons"] = n_unserved_persons
    
    rpp_eval_series["waiting time [min]"] = (served_persons[G_RQ_PU].mean() - served_persons[G_RQ_TIME].mean())/60.0
    rpp_eval_series["travel time [min]"] = (served_persons[G_RQ_DO].mean() - served_persons[G_RQ_PU].mean())/60.0
    
    eval_df = pd.DataFrame([rpp_eval_series]).transpose()
    eval_df.to_csv(os.path.join(output_dir, "rpp_eval.csv"))
    
    # print(rpp_eval_series)
    # unserved_persons.hist("rq_time")
    # plt.xlabel("rq_time [s]")
    # plt.ylabel("Number persons")
    # plt.title("request time of unserved persons")
    # plt.savefig(os.path.join(output_dir, "time_unserved_persons.png"))
    # plt.close()
    # served_persons.hist("pickup_time")
    # plt.xlabel("pickup_time [s]")
    # plt.ylabel("Number persons")
    # plt.title("pickup time of served persons")
    # plt.savefig(os.path.join(output_dir, "time_served_persons.png"))
    # plt.close()
    # parcel_users.hist("pickup_time")
    # plt.xlabel("pickup_time [s]")
    # plt.ylabel("Number parcels")
    # plt.title("pickup time of served parcels")
    # plt.savefig(os.path.join(output_dir, "time_served_parcels.png"))
    # plt.close()
    
def evaluate_folder(path, evaluation_start_time = None, evaluation_end_time = None, print_comments = False):
    """ this function calls standard_valuation on all simulation results found in the given path 
    :param evaluation_start_time: start time of evaluation interval in s (if None, then evaluation from sim start)
    :param evaluation_end_time: end time of evaluation interval in s   (if None, then evalation ountil sim end)
    :param print_comments: print comments
    """
    for f in os.listdir(path):
        sc_path = os.path.join(path, f)
        if os.path.isdir(sc_path):
            if os.path.isfile(os.path.join(sc_path, "1_user-stats.csv")):
                standard_rideparcelpooling_evaluation(sc_path, evaluation_start_time=evaluation_start_time, evaluation_end_time=evaluation_end_time, print_comments=print_comments)

if __name__ == "__main__":
    # rpp_folder = r'C:\Users\ge37ser\Documents\Coding\TUM_VT_FleetSimulation\tum-vt-fleet-simulation\FleetPy\studies\RPP_Simulation\results'
    # evaluate_folder(rpp_folder)
    res_folder = r'C:\Users\ge37ser\Documents\Coding\TUM_VT_FleetSimulation\tum-vt-fleet-simulation\FleetPy\studies\RPP_Simulation\results\agg_calib_scs_600\agg_calib_sc_600_0'
    standard_rideparcelpooling_evaluation(res_folder)