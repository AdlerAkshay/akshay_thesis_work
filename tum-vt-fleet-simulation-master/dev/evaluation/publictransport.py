import os
import sys
import numpy as np
import pandas as pd

MAIN_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(MAIN_DIR)
from src.misc.globals import *

ENERGY_EMISSIONS = 112 # g/kWh from https://www.swm.de/dam/swm/dokumente/geschaeftskunden/broschuere-strom-erdgas-gk.pdf
PT_COST_PER_KM = 326 # cent/km from
# https://prof.beuth-hochschule.de/fileadmin/prof/jschlaich/200811_Fr_JS_Kostenmodelle_NAHVERKEHR.pdf
EURO_PER_TON_OF_CO2 = 145 # from BVWP2030 Modulhandbuch (page 113)
EMISSION_CPG = 145 * 100 / 1000**2


def compute_daily_trips_per_line(columns, trip_cols):
    """This function computes the daily trip number of a given line. Due to scaling, the inputs could be floats."""
    tmp_sum = sum([columns[col] for col in trip_cols])
    return np.rint(tmp_sum)


def public_transport_evaluation(output_dir):
    """This function returns scalar public transportation information.

    :param output_dir: output directory of scenario
    :return: result_dict
    """
    scenario_parameters, list_operator_attributes, dir_names = load_scenario_inputs(output_dir)
    result_dict = {}
    # 0) evaluation period
    start_hour = scenario_parameters[G_SIM_START_TIME] // 3600
    end_hour = scenario_parameters[G_SIM_END_TIME] // 3600

    # 1) costs and emissions
    # ----------------------
    # get information from data
    gtfs_dir = dir_names[G_DIR_PT]
    route_f = os.path.join(gtfs_dir, "add_route_information.csv")
    route_df = pd.read_csv(route_f)
    # get frequency information from input
    scale_factor = scenario_parameters.get(G_PT_FRQ_SCALE, 1)
    if scale_factor != 1:
        frq_scale_hours = scenario_parameters.get(G_PT_FRQ_HOURS, [])
        for hour in frq_scale_hours:
            route_df[f"{G_PT_R_NR} {hour}-{hour+1}"] *= scale_factor
    # compute total number of trips per line per day
    # trip_cols = [col for col in route_df.columns if col.startswith(G_PT_R_NR)]
    trip_cols = []
    for hour in range(start_hour, end_hour):
        trip_cols.append(f"{G_PT_R_NR} {hour}-{hour+1}")
    route_df["daily_trips"] = route_df.apply(compute_daily_trips_per_line, axis=1, args=(trip_cols,))
    # compute monetary costs
    result_dict["pt_total_operating_costs"] = sum(route_df["daily_trips"] * route_df[G_PT_R_L]
                                                  * route_df[G_PT_COST_PER_KM])
    # compute emissions
    # TODO # check that units are matching!
    co2_per_kWh = scenario_parameters.get(G_ENERGY_EMISSIONS)
    if not co2_per_kWh:
        co2_per_kWh = ENERGY_EMISSIONS
    tot_co2_g = co2_per_kWh * sum(route_df["daily_trips"] * route_df[G_PT_R_EPT])
    result_dict["total CO2 emissions [t]"] = tot_co2_g / 10**6
    result_dict["total external emission costs"] = np.rint(EMISSION_CPG * tot_co2_g)

    # 2) pt utilization
    # -----------------
    pt_stat_f =  os.path.join(output_dir, "4_pt_stats.csv")
    if os.path.isfile(pt_stat_f):
        pt_stat_df = pd.read_csv(pt_stat_f)
        result_dict["max concurrent travelers"] = pt_stat_df[G_PT_TRAVELERS].max()
        result_dict["max crowding [%]"] = 100 * pt_stat_df[G_PT_CROWD].max()
        result_dict["avg concurrent travelers"] = pt_stat_df[G_PT_TRAVELERS].mean()
        result_dict["avg crowding [%]"] = 100 * result_dict["avg concurrent travelers"] / pt_stat_df[G_PT_CAP].mean()

    return result_dict


if __name__ == "__main__":
    public_transport_evaluation(sys.argv[1])
