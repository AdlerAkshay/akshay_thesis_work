import os
import sys
import glob
import numpy as np
import pandas as pd

MAIN_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(MAIN_DIR)
FLEETPY_DIR = os.path.join(MAIN_DIR, "FleetPy")
sys.path.append(MAIN_DIR)
sys.path.append(FLEETPY_DIR)
from dev.evaluation.publictransport import public_transport_evaluation
from src.misc.globals import *

EURO_PER_TON_OF_CO2 = 145 # from BVWP2030 Modulhandbuch (page 113)
EMISSION_CPG = 145 * 100 / 1000**2
ENERGY_EMISSIONS = 112 # g/kWh from https://www.swm.de/dam/swm/dokumente/geschaeftskunden/broschuere-strom-erdgas-gk.pdf
PV_G_CO2_KM = 130 # g/km from https://www.ris-muenchen.de/RII/RII/DOK/ANTRAG/2337762.pdf with 60:38 benzin vs diesel


# TODO # use src.misc.config.decode_input_str() instead of this function
def offerEntriesToDict(offer_entry):
    offer_dict = {}
    try:
        offer_strs = offer_entry.split("|")
    except:
        return {}
    for offer_str in offer_strs:
        x = offer_str.split(":")
        key = x[0]
        vals = ":".join(x[1:])
        try:
            key = int(key)
        except:
            pass
        offer_dict[key] = {}
        for offer_entries in vals.split(";"):
            try:
                k2, v2 = offer_entries.split(":")
            except:
                continue
            try:
                v2 = float(v2)
            except:
                pass
            offer_dict[key][k2] = v2
    return offer_dict


def create_vehicle_type_db(vehicle_data_dir):
    list_veh_data_f = glob.glob(f"{vehicle_data_dir}/*csv")
    veh_type_db = {}    # veh_type -> veh_type_data
    for f in list_veh_data_f:
        veh_type_name = os.path.basename(f)[:-4]
        veh_type_data = pd.read_csv(f, index_col=0, squeeze=True)
        veh_type_db[veh_type_name] = {}
        for k, v in veh_type_data.items():
            try:
                veh_type_db[veh_type_name][k] = float(v)
            except:
                veh_type_db[veh_type_name][k] = v
        veh_type_db[veh_type_name][G_VTYPE_NAME] = veh_type_data.name
    print(veh_type_db)
    return veh_type_db


# TODO # after ISTTT: use globals instead of strings where possible
def standard_evaluation(output_dir, print_comments=False):
    """This function runs a standard evaluation over a scenario output directory.

    :param output_dir: scenario output directory
    :param print_comments: print some comments about status in between
    """
    scenario_parameters, list_operator_attributes, dir_names = load_scenario_inputs(output_dir)
    # vehicle type data
    veh_type_db = create_vehicle_type_db(dir_names[G_DIR_VEH])
    veh_type_stats = pd.read_csv(os.path.join(output_dir, "2_vehicle_types.csv"))

    if print_comments:
        print(f"Evaluating {scenario_parameters[G_SCENARIO_NAME]}\nReading user stats ...")
    user_stats = pd.read_csv(os.path.join(output_dir, "1_user-stats.csv"))
    if print_comments:
        print(f"\t shape of user stats: {user_stats.shape}")
    # print(user_stats.head())
    # print(list_operator_attributes)
    result_dict_list = []
    operator_names = []

    offer_dict = {}
    for key, entries in user_stats.iterrows():
        rq_id = entries['request_id']
        offer_entry = entries['offers']
        offer = offerEntriesToDict(offer_entry)
        offer_dict[rq_id] = offer
        #print(offer)
    # TODO # standard_evaluation()
    # for all zones combined
    # 1) AMoD operators:
    if print_comments:
        print("\t ... processing AMoD user data")
    # - revenue, number served users, share of accepted users/modal split, wait/travel/detour time, utility (user_stats)
    # - costs, emissions, % fleet utilization, occupancy rates, total vkm, % empty vkm, % repositioning vkm,
    #   toll (op_stats)
    # - profit (derived)
    op_attributes = {}
    op_users = {}
    op_vehicle_stats = {}
    for op_id, attributes in enumerate(list_operator_attributes):
        boarding_time = attributes["op_const_boarding_time"]
        # user stats
        # ----------
        op_attributes[op_id] = attributes
        op_users[op_id] = user_stats[user_stats["operator_id"] == op_id]
        # TODO # user groupby instead of 4 times "== construction"?
        if print_comments:
            print("Loading AMoD vehicle data ...")
        try:
            op_vehicle_df = pd.read_csv(os.path.join(output_dir, f"2-{op_id}_op-stats.csv"))
        except FileNotFoundError:
            op_vehicle_df = pd.DataFrame([], columns=[G_V_OP_ID, G_V_VID, G_VR_STATUS, G_VR_LOCKED, G_VR_LEG_START_TIME,
                                                      G_VR_LEG_END_TIME, G_VR_LEG_START_POS, G_VR_LEG_END_POS,
                                                      G_VR_LEG_DISTANCE, G_VR_LEG_START_SOC, G_VR_LEG_END_SOC,
                                                      G_VR_TOLL, G_VR_OB_RID, G_VR_BOARDING_RID, G_VR_ALIGHTING_RID,
                                                      G_VR_NODE_LIST, G_VR_REPLAY_ROUTE])
        if print_comments:
            print(f"\t shape of vehicle stats: {op_vehicle_df.shape}\n\t ... processing AMoD vehicle data")
        op_vehicle_stats[op_id] = op_vehicle_df
        op_number_users = op_users[op_id].shape[0]
        op_modal_split = float(op_number_users)/user_stats.shape[0]
        sum_travel_time = op_users[op_id]["dropoff_time"].sum() - op_users[op_id]["pickup_time"].sum()
        op_revenue = op_users[op_id][G_RQ_FARE].sum()
        op_created_offers = sum((1 for rq_id in user_stats['request_id']
                                 if offer_dict[rq_id].get(op_id, None) is not None)) / user_stats.shape[0] * 100.0

        def get_rel_detour(row):
            direct_tt = offer_dict[row["request_id"]][G_MC_DEC_PV][G_OFFER_DRIVE] + boarding_time
            return 100 * (row["dropoff_time"] - row["pickup_time"] - boarding_time - direct_tt)/direct_tt
        if op_number_users > 0:
            op_utility = op_users[op_id][G_RQ_C_UTIL].sum()/op_number_users
            op_wait_time = (op_users[op_id]["pickup_time"].sum() - op_users[op_id]["rq_time"].sum()) / op_number_users
            op_travel_time = sum_travel_time / op_number_users
            op_detour_time = (sum_travel_time - sum([offer_dict[rq_id][G_MC_DEC_PV][G_OFFER_DRIVE] for rq_id in
                                                     op_users[op_id]['request_id']]))/op_number_users - 2*boarding_time
            op_rel_detour = op_users[op_id].apply(get_rel_detour, axis=1).sum() / op_number_users
        else:
            op_utility = 0
            op_wait_time = np.nan
            op_travel_time = np.nan
            op_detour_time = np.nan
            op_rel_detour = np.nan

        # vehicle stats
        # -------------
        n_vehicles = sum([x for x in attributes[G_OP_FLEET].values()])
        simulation_time = scenario_parameters["end_time"] - scenario_parameters["start_time"]
        try:
            op_fleet_utilization = 100 * (op_vehicle_df[G_VR_LEG_END_TIME].sum() -
                                          op_vehicle_df[G_VR_LEG_START_TIME].sum()) / (n_vehicles * simulation_time)
        except ZeroDivisionError:
            op_fleet_utilization = np.nan
        op_total_km = op_vehicle_df[G_VR_LEG_DISTANCE].sum()/1000.0

        def weightOB(entries):
            try:
                return len(entries[G_VR_OB_RID].split(";")) * entries[G_VR_LEG_DISTANCE]
            except:
                return 0.0
        try:
            op_vehicle_df["weighted_ob"] = op_vehicle_df.apply(weightOB, axis = 1)
            op_distance_avg_occupancy = op_vehicle_df["weighted_ob"].sum()/op_vehicle_df[G_VR_LEG_DISTANCE].sum()
            empty_df = op_vehicle_df[op_vehicle_df[G_VR_OB_RID].isnull()]
            op_empty_vkm = empty_df[G_VR_LEG_DISTANCE].sum()/1000.0/op_total_km*100.0
            op_repositioning_vkm = empty_df[empty_df[G_VR_STATUS] == "reposition"][G_VR_LEG_DISTANCE].sum()/op_total_km*100.0/1000.0
            op_toll = op_vehicle_df[G_VR_TOLL].sum()
        except ZeroDivisionError:
            op_vehicle_df["weighted_ob"] = 0
            op_distance_avg_occupancy = 0
            op_empty_vkm = 0
            op_repositioning_vkm = 0
            op_toll = 0

        # by vehicle stats
        # ----------------
        op_veh_types = veh_type_stats[veh_type_stats[G_V_OP_ID] == op_id]
        op_veh_types.set_index(G_V_VID, inplace=True)
        all_vid_dict = {}
        for vid, vid_vtype_row in op_veh_types.iterrows():
            vtype_data = veh_type_db[vid_vtype_row[G_V_TYPE]]
            op_vid_vehicle_df = op_vehicle_df[op_vehicle_df[G_V_VID] == vid]
            veh_km = op_vid_vehicle_df[G_VR_LEG_DISTANCE].sum() / 1000
            veh_kWh = veh_km * vtype_data[G_VTYPE_BATTERY_SIZE] / vtype_data[G_VTYPE_RANGE]
            co2_per_kWh = scenario_parameters.get(G_ENERGY_EMISSIONS, ENERGY_EMISSIONS)
            veh_co2 = co2_per_kWh * veh_kWh
            veh_fix_costs = np.rint(scenario_parameters.get(G_OP_SHARE_FC, 1.0) * vtype_data[G_VTYPE_FIX_COST])
            veh_var_costs = np.rint(vtype_data[G_VTYPE_DIST_COST] * veh_km)
            # TODO # after ISTTT: idle times
            all_vid_dict[vid] = {"type":vtype_data[G_VTYPE_NAME], "total km":veh_km, "total kWh": veh_kWh,
                                 "total CO2 [g]": veh_co2, "fix costs": veh_fix_costs,
                                 "total variable costs": veh_var_costs}
        all_vid_df = pd.DataFrame.from_dict(all_vid_dict, orient="index")
        all_vid_df.to_csv(os.path.join(output_dir, f"standard_mod-{op_id}_veh_eval.csv"))

        # aggregated specific by vehicle stats
        # ------------------------------------
        try:
            op_co2 = all_vid_df["total CO2 [g]"].sum()
            op_ext_em_costs = np.rint(EMISSION_CPG * op_co2)
            op_fix_costs = all_vid_df["fix costs"].sum()
            op_var_costs = all_vid_df["total variable costs"].sum()
        except:
            op_co2 = 0
            op_ext_em_costs = 0
            op_fix_costs = 0
            op_var_costs = 0
        # output
        # ------
        result_dict = {"operator_id": op_id, "number users": op_number_users, "modal split": op_modal_split,
                       "travel time": op_travel_time, "utility": op_utility}
        result_dict["waiting time"] = op_wait_time
        result_dict["detour time"] = op_detour_time
        result_dict["rel detour"] = op_rel_detour
        result_dict[r"% fleet utilization"] = op_fleet_utilization
        result_dict["total vkm"] = op_total_km
        result_dict["occupancy"] = op_distance_avg_occupancy
        result_dict[r"% empty vkm"] = op_empty_vkm
        result_dict[r"% repositioning vkm"] = op_repositioning_vkm
        result_dict[r'% created offers'] = op_created_offers
        result_dict["total toll"] = op_toll
        result_dict["mod revenue"] = op_revenue
        result_dict["mod fix costs"] = op_fix_costs
        result_dict["mod var costs"] = op_var_costs
        result_dict["total CO2 emissions [t]"] = op_co2 / 10**6
        result_dict["total external emission costs"] = op_ext_em_costs

        result_dict_list.append(result_dict)
        operator_names.append("MoD_{}".format(op_id))

    # 2) public transportation: -> evaluation/publictransport.py
    # - revenue, number served users, modal split, travel time, utility (user_stats)
    # - costs, emissions, utilization (publictransport evaluation)
    if print_comments:
        print("\t ... processing PT user data")
    pt_user_stats = user_stats[user_stats['operator_id'] == G_MC_DEC_PT]
    if len(pt_user_stats) > 0:
        pt_number_users = pt_user_stats.shape[0]
        pt_modal_split = float(pt_number_users)/user_stats.shape[0]
        pt_travel_time = sum([offer_dict[rq_id][G_MC_DEC_PT][G_OFFER_DRIVE]
                              for rq_id in pt_user_stats['request_id']])/pt_number_users
        pt_revenue = pt_user_stats[G_RQ_FARE].sum()
        pt_utility = pt_user_stats[G_RQ_C_UTIL].sum()/pt_number_users
        pt_created_offers = sum( (1 for rq_id in user_stats['request_id']
                                  if offer_dict[rq_id].get(G_MC_DEC_PT, None) is not None))/user_stats.shape[0]*100.0
        pt_extra_eval_dict = public_transport_evaluation(output_dir)
        result_dict = {"operator_id": G_MC_DEC_PT, "number users": pt_number_users,
                                  "modal split": pt_modal_split, "travel time" : pt_travel_time, "utility": pt_utility,
                                  r'% created offers': pt_created_offers, "pt revenue":pt_revenue}
        result_dict.update(pt_extra_eval_dict)
        #print("stats pt: ", pt_number_users, pt_modal_split, pt_travel_time, pt_utility)
        result_dict_list.append(result_dict)
        operator_names.append("PT")

    # 3) private vehicle:
    # - number PV users, modal split, travel time, parking costs, toll, utility (user_stats)
    # - costs, emissions
    if print_comments:
        print("\t ... processing PV user data")
    pv_user_stats = user_stats[user_stats['operator_id'] == G_MC_DEC_PV]
    pv_number_users = pv_user_stats.shape[0]
    pv_modal_split = float(pv_number_users)/user_stats.shape[0]
    pv_travel_time = sum([offer_dict[rq_id][G_MC_DEC_PV][G_OFFER_DRIVE] for rq_id in pv_user_stats['request_id']])/pv_number_users
    pv_distance = sum([offer_dict[rq_id][G_MC_DEC_PV][G_OFFER_DIST] for rq_id in pv_user_stats['request_id']])/pv_number_users
    pv_toll = pv_user_stats['included_toll'].sum()/pv_number_users
    pv_parking_cost = pv_user_stats['included_park_costs'].sum()/pv_number_users
    pv_utility = pv_user_stats[G_RQ_C_UTIL].sum()/pv_number_users
    pv_created_offers = sum( (1 for rq_id in user_stats['request_id'] if offer_dict[rq_id].get(G_MC_DEC_PV, None) is not None))/user_stats.shape[0]*100.0
    # total values
    g_co2_per_km = scenario_parameters.get(G_PV_G_CO2_KM)
    if not g_co2_per_km:
        g_co2_per_km = PV_G_CO2_KM
    g_co2 = pv_number_users * pv_distance/1000 *g_co2_per_km
    total_emissions = g_co2 / 10**6
    total_ext_emission_costs = np.rint(EMISSION_CPG * g_co2)
    #print("stats pv: ",pv_number_users, pv_modal_split, pv_travel_time, pv_toll, pv_parking_cost, pv_utility)
    result_dict_list.append( {"operator_id": G_MC_DEC_PV, "number users": pv_number_users,
                              "modal split": pv_modal_split, "travel time": pv_travel_time,
                              "travel distance": pv_distance,  "parking cost": pv_parking_cost, "toll": pv_toll,
                              "utility": pv_utility, r'% created offers': pv_created_offers,
                              "total CO2 emissions [t]": total_emissions,
                              "total external emission costs": total_ext_emission_costs})
    operator_names.append("PV")

    # 4) intermodal AMoD: AMoD fare already treated by sub-request -> only evaluate PT fare and subsidies (user_stats)
    if print_comments:
        print("\t ... processing intermodal user data")
    for op_id in op_vehicle_stats.keys():
        im_op_id = -1*(op_id - G_MC_DEC_IM)
        im_users = user_stats[user_stats["operator_id"] == im_op_id]
        im_number_users = im_users.shape[0]
        im_modal_split = im_number_users/user_stats.shape[0] # TODO # modal split adaption of PT?
        im_utility = im_users[G_RQ_C_UTIL].sum()/im_number_users
        im_pt_revenue = im_users[G_RQ_IM_PT_FARE].sum()
        im_subsidy = im_users[G_RQ_SUB].sum()

        im_created_offers = sum( (1 for rq_id in user_stats['request_id'] if offer_dict[rq_id].get(im_op_id, None) is not None))/user_stats.shape[0]*100.0

        result_dict = {"operator_id": im_op_id, "number users": im_number_users, "modal split": im_modal_split,
                       "utility": im_utility, r'% created offers': im_created_offers, "pt revenue":im_pt_revenue,
                       "total intermodal MoD subsidy":im_subsidy}
        result_dict_list.append(result_dict)
        operator_names.append("IM_MoD_{}".format(op_id))

    # combine and save
    result_df = pd.DataFrame(result_dict_list, index=operator_names)
    print(result_df)
    result_df.to_csv(os.path.join(output_dir, "standard_eval.csv"))


if __name__ == "__main__":
    import sys
    standard_evaluation(sys.argv[1], print_comments=True)
