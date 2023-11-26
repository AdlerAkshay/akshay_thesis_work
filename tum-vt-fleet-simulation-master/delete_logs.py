import sys
import os

MAINPATH = os.path.abspath(__file__)

scriptname = os.path.basename(__file__)
__doc__ = """

this script can be used to delete all logs within a either
    - a given folder
    - a given results folder (then looks for all scenario folders within this folder)

Call:
-----
python {} path
""".format(scriptname)

def delete_log_in_scenario(scenario_folder):
    for f in os.listdir(scenario_folder):
        if f.startswith("00_simulation") and f.endswith(".log"):
            try:
                print(f"... remove {os.path.join(scenario_folder, f)}")
                os.remove(os.path.join(scenario_folder, f))
            except OSError as e:
                print(e)

def is_scenario_folder(scenario_folder):
    if os.path.isfile(os.path.join(scenario_folder, "00_config.json")):
        return True
    else:
        return False

def is_results_folder(results_folder):
    if os.path.basename(results_folder) == "results":
        return True
    else:
        return False

if __name__ == '__main__':
    print(__doc__)
    if len(sys.argv) == 2:

        p = sys.argv[1]

        if is_scenario_folder(p):
            delete_log_in_scenario(p)
        elif is_results_folder(p):
            for sc in os.listdir(p):
                scp = os.path.join(p, sc)
                if is_scenario_folder(scp):
                    delete_log_in_scenario(scp)
        else:
            print(f"{p} is neither a scenario folder nor a results folder!")
