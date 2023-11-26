import sys
import os
import pandas as pd
from shutil import copy
from src.misc.globals import *

MAIN_DIR = os.path.dirname(__file__)

scriptname = os.path.basename(__file__)
__doc__ = """

this script only collects the files "00_config.json" and "standard_eval.csv" for each scenario compted and found
in a given FleetPy/study/results folder and can be used in case only the aggregated results are needed for evaluation
these files are copied in a folder FleetPy/study/results_agg_only/scenario/...

Call:
-----
python {} study_name
""".format(scriptname)

def create_agg_results_folder(study_name):
    relevant_files = ["00_config.json", "standard_eval.csv"]
    study_folder = os.path.join(MAIN_DIR, "FleetPy", "studies", study_name)
    results_folder = os.path.join(study_folder, "results")
    new_results_folder = os.path.join(study_folder, "results_agg_only")
    if not os.path.isdir(new_results_folder):
        os.mkdir(new_results_folder)
    for sc in os.listdir(results_folder):
        if os.path.isdir(os.path.join(results_folder, sc)):
            all_in = True
            for f in relevant_files:
                if not f in os.listdir(os.path.join(results_folder, sc)):
                    all_in = False
                    break
            if all_in:
                if not os.path.isdir(os.path.join(new_results_folder, sc)):
                    os.mkdir(os.path.join(new_results_folder, sc))
                print("copy file for scenario ", sc)
                for f in relevant_files:
                    copy(os.path.join(results_folder, sc, f), os.path.join(new_results_folder, sc, f))

if __name__ == "__main__":
    if len(sys.argv) > 1:
        # constant_config_file, scenario_file, n_parallel_sim=1, n_cpu_per_sim=1
        x = sys.argv[1:]
        study_name = x[0]
        print("copy files for study", study_name)
        create_agg_results_folder(study_name)
