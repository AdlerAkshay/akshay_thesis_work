# -------------------------------------------------------------------------------------------------------------------- #
# standard distribution imports
# -----------------------------
import logging
import importlib
import os
import json

# additional module imports (> requirements)
# ------------------------------------------
# from IPython import embed

# src imports
# -----------

from src.FleetSimulationBase import FleetSimulationBase

# -------------------------------------------------------------------------------------------------------------------- #
# global variables
# ----------------
from src.misc.globals import *
LOG = logging.getLogger(__name__)


# -------------------------------------------------------------------------------------------------------------------- #
# functions
# ---------


# -------------------------------------------------------------------------------------------------------------------- #
# main
# ----

#TODO: Fahrzeugkapazität bei Parcel-Requests checken

class RPPFleetSimulation(FleetSimulationBase):
    """
    Init main simulation module. Check the documentation for a flowchart of this particular simulation environment.
    Main attributes:
    - agent list per time step query public transport and fleet operator for offers and immediate decide
    - fleet operator offers ride pooling service
    - division of study area
        + first/last mile service in different parts of the study area
        + different parking costs/toll/subsidy in different parts of the study area
    """

    def check_sim_env_spec_inputs(self, scenario_parameters):
        if scenario_parameters[G_AR_MAX_DEC_T] != 0:
            raise EnvironmentError(f"Scenario parameter {G_AR_MAX_DEC_T} has to be set to 0 for simulations in the "
                                   f"{self.__class__.__name__} environment!")

    def add_init(self, scenario_parameters):
        """
        Simulation specific additional init.
        :param scenario_parameters: row of pandas data-frame; entries are saved as x["key"]
        """
        super().add_init(scenario_parameters)

    def step(self, sim_time):
        raise NotImplementedError

    def add_evaluate(self):
        """Runs standard and simulation environment specific evaluations over simulation results."""
        # output_dir = self.dir_names[G_DIR_OUTPUT]
        # from src.evaluation.temporal import run_complete_temporal_evaluation
        # run_complete_temporal_evaluation(output_dir, method="snapshot")
        pass