import sys
import os

scriptname = os.path.basename(__file__)
__doc__ = """

this script can be used to initialize the directories need when starting a new study
it creates the folders
FleetPy/studies/
FleetPy/studies/study_name
FleetPy/studies/study_name/preprocessing
FleetPy/studies/study_name/scenarios/
FleetPy/studies/study_name/results/
FleetPy/studies/study_name/evaluation/
See Data_Directory_Structure.md for more information!

Call:
-----
python {} study_name
""".format(scriptname)

def create_study_directories(study_name):
    """ creates all directories within the current directory for a new study
    :param study_name: (str) name of the new study
    """
    protected = ["src", "documentation", "data"]
    if study_name in protected:
        print("ERROR {} can't be used as study_name!".format(study_name))
        exit()
    abs_path = os.path.dirname(os.path.abspath(__file__))
    studies_folder = os.path.join(abs_path, "FleetPy", "studies")
    if not os.path.isdir(studies_folder):
        print("Initializing Studies Folder {}".format(studies_folder))
        os.mkdir(studies_folder)
    study_folder = os.path.join(studies_folder, study_name)
    if os.path.isdir(study_folder):
        print("Warning: {} already existent!".format(study_name))
    else:
        os.mkdir(study_folder)
        print("creating {}".format(study_folder))
    preprocessing_folder = os.path.join(study_folder, "preprocessing")
    if not os.path.isdir(preprocessing_folder):
        os.mkdir(preprocessing_folder)
        print("creating {}".format(preprocessing_folder))
    scenarios_folder = os.path.join(study_folder, "scenarios")
    if not os.path.isdir(scenarios_folder):
        os.mkdir(scenarios_folder)
        print("creating {}".format(scenarios_folder))
    results_folder = os.path.join(study_folder, "results")
    if not os.path.isdir(results_folder):
        os.mkdir(results_folder)
        print("creating {}".format(preprocessing_folder))
    evaluation_folder = os.path.join(study_folder, "evaluation")
    if not os.path.isdir(evaluation_folder):
        os.mkdir(evaluation_folder)
        print("creating {}".format(evaluation_folder))

if __name__ == '__main__':
    print(__doc__)
    if len(sys.argv) == 2:
        create_study_directories(sys.argv[1])