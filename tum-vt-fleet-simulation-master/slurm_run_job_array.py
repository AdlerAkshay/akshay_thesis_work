# -------------------------------------------------------------------------------------------------------------------- #
# standard distribution imports
# -----------------------------
import os
import sys
# additional module imports (> requirements)
# ------------------------------------------

# src imports
# -----------
import src.misc.config as config
from run_scenarios_from_csv_and_config import run_single_simulation
# -------------------------------------------------------------------------------------------------------------------- #
# global variables
# ----------------
LOG_LEVEL = "info"

CPU_PER_SIM = int(os.environ.get('SLURM_CPUS_PER_TASK', 1))
task_id = int(os.environ['SLURM_ARRAY_TASK_ID'])
n_cpu = int(os.environ["SLURM_NTASKS"])
proc_id = int(os.environ['SLURM_PROCID'])

SCENARIO_ID = task_id * n_cpu + proc_id


# -------------------------------------------------------------------------------------------------------------------- #
# main functions
def main(constant_config_csv_f, scenario_config_csv_f, n_cpu_per_sim=1, evaluate=1, bugfix=False, keep_old=False):
    """This function combines constant study parameters and scenario parameters.
    Then it sets up a pool of workers and starts a simulation for each scenario.
    The required parameters are stated in the documentation.

    :param constant_config_csv_f: this file contains all input parameters that remain constant for a study
    :type constant_config_csv_f: str
    :param scenario_config_csv_f: this file contain all input parameters that are varied for a study
    :type scenario_config_csv_f: str
    :param n_cpu_per_sim: number of cpus for a single simulation
    :type n_cpu_per_sim: int
    :param evaluate: no automatic evaluation / != 0 automatic simulation after each simulation
    :type evaluate: int
    :param bugfix: removes try,except structure for single simulation if True
    :type bugfix: bool
    :param keep_old: does not start new simulation if result files are already available in scenario output directory
    :type keep_old: bool
    :return: None
    """

    constant_cfg = config.ConstantConfig(constant_config_csv_f)
    scenario_cfgs = config.ScenarioConfig(scenario_config_csv_f)
    try:
        only_scenario_cfg = scenario_cfgs[SCENARIO_ID]
        print(f"Scenario id {SCENARIO_ID}:", str(only_scenario_cfg))
    except IndexError:
        print(f"No scenario with id {SCENARIO_ID}. Number of Scenarios: {len(scenario_cfgs)}")
        return

    # set constant parameters from function arguments
    constant_cfg["n_cpu_per_sim"] = n_cpu_per_sim
    constant_cfg["evaluate"] = evaluate
    constant_cfg["keep_old"] = keep_old
    constant_cfg["log_level"] = LOG_LEVEL

    # base: constant parameters, overwrite/add scenario parameters
    scenario_cfg = constant_cfg + only_scenario_cfg
    run_single_simulation(scenario_cfg)


if __name__ == "__main__":
    if len(sys.argv) != 3:
        raise IOError("False Number of Input Parameters!")
    constant_csv_f = sys.argv[1]
    scenario_csv_f = sys.argv[2]
    main(constant_csv_f, scenario_csv_f, n_cpu_per_sim=CPU_PER_SIM)