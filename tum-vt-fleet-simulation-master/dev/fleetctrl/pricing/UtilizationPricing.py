import pandas as pd
from src.misc.functions import PiecewiseContinuousLinearFunction
import logging
LOG = logging.getLogger(__name__)


class UtilizationSurgePricing:
    def __init__(self, util_file, evaluation_interval):
        """This method generates the piecewise linear continuous function based on a file of util -> prize factors.

        :param util_file: (utilization -> surge factors) for a piecewise linear function
        :type util_file: file path
        :param evaluation_interval: time interval in which changes in price may take place
        :type evaluation_interval: int
        """
        p_df = pd.read_csv(util_file)
        base_fare_tuples = []
        distance_fare_tuples = []
        global_fare_tuples = []
        for _, entries in p_df.iterrows():
            #utilization,base_fare_factor,distance_fare_factor,general_factor
            utilization = entries["utilization"]
            if utilization < 0  or utilization > 1:
                raise AssertionError(f"return_utilization_surge_factor(): utilization {utilization} not in [0,1]!")
            base_fare_tuples.append( (utilization, entries["base_fare_factor"]) )
            distance_fare_tuples.append( (utilization, entries["distance_fare_factor"]) )
            global_fare_tuples.append( (utilization, entries["general_factor"]) )

        self.pw_base_fare = PiecewiseContinuousLinearFunction(base_fare_tuples)
        self.pw_distance_fare = PiecewiseContinuousLinearFunction(distance_fare_tuples)
        self.pw_global_fare = PiecewiseContinuousLinearFunction(global_fare_tuples)

        self.evaluation_interval = evaluation_interval
        self.last_price_evaluation = -1
        self.last_function_call = -1
        self.avg_util = None
        self.N_evals = 0

        self.current_fare_tuple = (1.0, 1.0, 1.0)

    def return_utilization_surge_factor(self, sim_time, utilization):
        """This method returns the current surge factor based on the utilization and the pre-defined setting.

        if the evaluation_interval is given, the prices dont change for the length of this interval
        at end, prices are adopted based on the avg utilization within this interval

        :param sim_time: current_simulation time
        :type sim_time: int
        :param utilization: value between [0,1]
        :type utilization: float
        :return: tuple of base_fare_factor, distance_fare_factor, general_factor
        :rtype: (float, float, float)
        """
        if utilization < 0 or utilization > 1:
            raise AssertionError(f"return_utilization_surge_factor(): utilization {utilization} not in [0,1]!")
        if sim_time == self.last_function_call:
            return self.current_fare_tuple
        else:
            if self.N_evals == 0:
                self.avg_util = utilization
                self.N_evals += 1
            else:
                self.avg_util = float(self.N_evals)/(self.N_evals + 1) * self.avg_util + 1.0/(self.N_evals + 1) * utilization
                self.N_evals += 1
            if sim_time - self.last_price_evaluation >= self.evaluation_interval:
                self.current_fare_tuple = (self.pw_base_fare.get_y(self.avg_util), self.pw_distance_fare.get_y(self.avg_util), self.pw_global_fare.get_y(self.avg_util))
                LOG.info("new evaluation of fare at {} with util {} -> {}".format(sim_time, self.avg_util, self.current_fare_tuple))
                self.avg_util = None
                self.N_evals = 0
                self.last_price_evaluation = sim_time
            return self.current_fare_tuple