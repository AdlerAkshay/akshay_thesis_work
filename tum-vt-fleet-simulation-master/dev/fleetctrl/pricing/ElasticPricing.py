import pandas as pd
import logging
LOG = logging.getLogger(__name__)

class ElasticPrizing:
    def __init__(self, pricing_file):
        """ this class reads time dependent pricing scales from a pricing file
        these factors can be returned during simulation time """
        p_df = pd.read_csv(pricing_file)
        self.time_prices = []
        for _, entries in p_df.iterrows():
            #time,base_fare_factor,distance_fare_factor,general_factor
            self.time_prices.append( (entries["time"], entries["base_fare_factor"], entries["distance_fare_factor"], entries["general_factor"]) )
        self.time_prices.sort(key = lambda x:x[0])
        self.current_scales = self.time_prices.pop(0)
        LOG.info("initialzied elastic pricing; current scales: {}".format(self.current_scales))

    def get_elastic_price_factors(self, sim_time):
        """ returns current time dependent fare scales
        :param sim_time: current simulation time
        :type sim_time: int
        :return: tuple of base_fare_factor, distance_fare_factor, general_factor
        :rtype: (float, float, float)
        """
        while len(self.time_prices) > 0 and self.time_prices[0][0] < sim_time:
            self.current_scales = self.time_prices.pop(0)
            LOG.info("update elastic pricing at {} to {}".format(sim_time, self.current_scales))
        return self.current_scales[1], self.current_scales[2], self.current_scales[3]
        