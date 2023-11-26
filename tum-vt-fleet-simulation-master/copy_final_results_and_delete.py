       
import sys
import os
import pandas as pd
from shutil import copy, rmtree
from src.misc.globals import *

MAIN_DIR = os.path.dirname(__file__)

scriptname = os.path.basename(__file__)
__doc__ = """

this script only collects the necesssary files for each scenario completed and found
in a given FleetPy/study/results folder and can be used in case only the aggregated results are needed for evaluation
these files are copied in a folder FleetPy/study/results_finished/scenario/...

Call:
-----
python {} study_name
""".format(scriptname)

def create_agg_results_folder(study_name):
    relevant_files = ["00_config.json", "standard_eval.csv",
                      "1_user-stats.csv", "2_vehicle_types.csv", "2-0_op-stats.csv", "3-0_op-dyn_atts.csv",
                      "4-0_repositioning_info.csv", "final_state.csv", "standard_mod-0.0_veh_eval.csv"]
    study_folder = os.path.join(MAIN_DIR, "FleetPy", "studies", study_name)
    results_folder = os.path.join(study_folder, "results")
    new_results_folder = os.path.join(study_folder, "results_finished")
    if not os.path.isdir(new_results_folder):
        os.mkdir(new_results_folder)
    for sc in os.listdir(results_folder):
        if os.path.isdir(os.path.join(results_folder, sc)):
            print("found scenario", sc)
            if not "standard_eval.csv" in os.listdir(os.path.join(results_folder, sc)):
                print(" -> not finished")
                continue
            if not os.path.isdir(os.path.join(new_results_folder, sc)):
                os.mkdir(os.path.join(new_results_folder, sc))
            print(" -> copy files for scenario ", sc)
            for f in relevant_files:
                if f in os.listdir(os.path.join(results_folder, sc)):
                    copy(os.path.join(results_folder, sc, f), os.path.join(new_results_folder, sc, f))
            print(" -> remove results folder")
            rmtree(os.path.join(results_folder, sc))

if __name__ == "__main__":
    if len(sys.argv) > 1:
        # constant_config_file, scenario_file, n_parallel_sim=1, n_cpu_per_sim=1
        x = sys.argv[1:]
        study_name = x[0]
        print("copy files for study", study_name)
        create_agg_results_folder(study_name)