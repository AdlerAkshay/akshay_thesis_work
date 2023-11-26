# -------------------------------------------------------------------------------------------------------------------- #
# standard distribution imports
# -----------------------------
import sys
import os
import multiprocessing as mp
import traceback
from time import sleep
from pathlib import Path
import importlib

# additional module imports (> requirements)
# ------------------------------------------
from tqdm import tqdm

# Add path of FleetyPy submodule
# ------------------------------------------
sys.path.append(str(Path().joinpath("FleetPy")))

# src imports
# -----------
# from IPython import embed
# embed()
from src.misc.init_modules import load_simulation_environment
import src.misc.config as config
from src.misc.globals import *

# -------------------------------------------------------------------------------------------------------------------- #
# global variables
# ----------------


# -------------------------------------------------------------------------------------------------------------------- #
# main functions
def run_single_simulation(scenario_parameters, write_lock=None, tqdm_position=0):
    if write_lock is not None:
        tqdm.set_lock(write_lock)
    SF = load_simulation_environment(scenario_parameters)
    if scenario_parameters.get("bugfix", False):
        try:
            SF.run(tqdm_position)
        except:
            traceback.print_exc()
    else:
        SF.run(tqdm_position)


def run_scenarios(constant_config_file, scenario_file, n_parallel_sim=1, n_cpu_per_sim=1, evaluate=1, log_level="info",
                  keep_old=False, continue_next_after_error=False):
    """
    This function combines constant study parameters and scenario parameters.
    Then it sets up a pool of workers and starts a simulation for each scenario.
    The required parameters are stated in the documentation.

    :param constant_config_file: this file contains all input parameters that remain constant for a study
    :type constant_config_file: str
    :param scenario_file: this file contain all input parameters that are varied for a study
    :type scenario_file: str
    :param n_parallel_sim: number of parallel simulation processes
    :type n_parallel_sim: int
    :param n_cpu_per_sim: number of cpus for a single simulation
    :type n_cpu_per_sim: int
    :param evaluate: 0: no automatic evaluation / != 0 automatic simulation after each simulation
    :type evaluate: int
    :param log_level: hierarchical output to the logging file. Possible inputs with hierarchy from low to high:
            - "verbose": lowest level -> logs everything; even code which could scale exponentially
            - "debug": standard debugging logger. code which scales exponentially should not be logged here
            - "info": basic information during simulations (default)
            - "warning": only logs warnings
    :type log_level: str
    :param keep_old: does not start new simulation if result files are already available in scenario output directory
    :type keep_old: bool
    :param continue_next_after_error: continue with next simulation if one the simulations threw an error (only SP)
    :type continue_next_after_error: bool
    """
    assert type(n_parallel_sim) == int, "n_parallel_sim must be of type int"
    # read constant and scenario config files
    constant_cfg = config.ConstantConfig(constant_config_file)
    scenario_cfgs = config.ScenarioConfig(scenario_file)

    # set constant parameters from function arguments
    # TODO # get study name and check if its a studyname
    const_abs = os.path.abspath(constant_config_file)
    study_name = os.path.basename(os.path.dirname(os.path.dirname(const_abs)))

    if study_name == "scenarios":
        print("ERROR! The path of the config files is not longer up to date!")
        print("See documentation/Data_Directory_Structure.md for the updated directory structure needed as input!")
        exit()
    if constant_cfg.get(G_STUDY_NAME) is not None and study_name != constant_cfg.get(G_STUDY_NAME):
        print("ERROR! {} from constant config is not consitent with study directory: {}".format(constant_cfg[G_STUDY_NAME], study_name))
        print("{} is now given directly by the folder name !".format(G_STUDY_NAME))
        exit()
    constant_cfg[G_STUDY_NAME] = study_name

    constant_cfg["n_cpu_per_sim"] = n_cpu_per_sim
    constant_cfg["evaluate"] = evaluate
    constant_cfg["log_level"] = log_level
    constant_cfg["keep_old"] = keep_old

    # combine constant and scenario parameters into verbose scenario parameters
    for i, scenario_cfg in enumerate(scenario_cfgs):
        scenario_cfgs[i] = constant_cfg + scenario_cfg

    # perform simulation(s)
    print(f"Simulation of {len(scenario_cfgs)} scenarios on {n_parallel_sim} processes with {n_cpu_per_sim} cpus per simulation ...")
    if n_parallel_sim == 1:
        for scenario_cfg in scenario_cfgs:
            if continue_next_after_error:
                try:
                    run_single_simulation(scenario_cfg)
                except:
                    traceback.print_exc()
            else:
                run_single_simulation(scenario_cfg)
    else:
        write_lock = mp.Lock()
        tqdm.set_lock(write_lock)
        process_dict = {i: None for i in range(n_parallel_sim)}
        completed = 0
        submitted = 0
        total = len(scenario_cfgs)

        with tqdm(total=total, position=0) as pbar:
            while (True):
                free_process_ids = {key for key in process_dict if process_dict[key] is None}
                if len(free_process_ids) != 0 and submitted < total:
                    for tqdm_id in free_process_ids:
                        task = scenario_cfgs[0]
                        p = mp.Process(target=run_single_simulation, args=(task, write_lock, tqdm_id + 1))
                        p.start()
                        del scenario_cfgs[0]
                        submitted += 1
                        process_dict[tqdm_id] = p

                for key, p in process_dict.items():
                    if p is not None:
                        if not p.is_alive():
                            completed += 1
                            pbar.update(1)
                            process_dict[key] = None

                pbar.set_postfix({"completed": completed, "submitted": submitted,
                                  "running": n_parallel_sim - len(free_process_ids)})
                if completed >= total:
                    break

                sleep(0.001)


if __name__ == "__main__":
    mp.freeze_support()
    if len(sys.argv) > 2:
        # constant_config_file, scenario_file, n_parallel_sim=1, n_cpu_per_sim=1
        x = sys.argv[1:]
        cont_conf = x[0]
        sc_conf = x[1]
        n_parallel_sim = 1
        if len(x) >= 3:
            n_parallel_sim = int(x[2])
        n_cpu_per_sim = 1
        if len(x) >= 4:
            n_cpu_per_sim = int(x[3])
        evaluate = 1
        if len(x) >= 5:
            evaluate = int(x[4])
        log_level = "info"
        if len(x) >= 6:
            if x[5] in ["verbose", "debug", "info", "warning"]:
                log_level = x[5]
            else:
                print("WARNING: bugfix parameter obsolete: chose from [verbose, debug, info, warning] in future!")
                if int(x[5]) == 1:
                    log_level = "debug"
        run_scenarios(cont_conf, sc_conf, n_parallel_sim=n_parallel_sim, n_cpu_per_sim=n_cpu_per_sim, evaluate=evaluate,
                      log_level=log_level)
    # use bugfix_or_run.py for hard coding paths!
