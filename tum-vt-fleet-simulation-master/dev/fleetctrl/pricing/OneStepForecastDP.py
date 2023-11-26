import os
import logging
import numpy as np
import pandas as pd
from src.fleetctrl.pricing.DynamicPricingBase import DynamicPrizingBase

from src.misc.globals import *
LOG = logging.getLogger(__name__)


class OneStepForecastDP(DynamicPrizingBase):
    """This dynamic pricing class is described in PhD thesis of Florian. It assumes a price-factor (p) sensitivity
    according to the (sign-inverted) logistic function
    D(p) = (1 + exp[a-b]) / (1 + exp[a*p - b])
    and tries to increase the prices in zones with under-supply such that the expected number of reduced trips matches
    the reachability corrected imbalance in the current (repositioning) forecast horizon.
    """
    def __init__(self, fleetctrl, operator_attributes, solver="Gurobi"):
        """Initialization of dynamic pricing class.

        :param fleetctrl: FleetControl class
        :param operator_attributes: operator dictionary that can contain additionally required parameters
        :param solver: solver for optimization problems
        """
        super().__init__(fleetctrl, operator_attributes, solver)
        # children classes:
        # - check of additionally required attributes from operator_attributes
        # - save these as class attributes
        self.zone_system = fleetctrl.zones
        if self.zone_system is None:
            raise IOError(f"Dynamic pricing policy {self.__class__.__name__} requires zone systems!")
        # operator-estimated parameters of inverse logistic function
        self.a = operator_attributes[G_OP_DYN_P_LOG_A]
        self.b = operator_attributes[G_OP_DYN_P_LOG_B]
        horizon = operator_attributes[G_OP_REPO_TH_DEF]
        self.horizon_start = horizon[0]
        self.horizon_end = horizon[1]
        self.dp_update_step = operator_attributes[G_OP_REPO_TS]
        self.max_pricing_factor = operator_attributes[G_OP_DYN_MAX_PF]
        self.supply_fc_type = operator_attributes.get(G_OP_FC_SUPPLY)
        #
        self.current_base_fare_factor = 1.0
        self.current_distance_fare_factor = 1.0
        self.current_general_factors = {}
        #
        self.record_df_cols = ["sim_time", "o_zone_id",
                               "expected_deficit", "max_demand",
                               "general_factor", "capped_general_factor"]
        self.record_df_index_cols = self.record_df_cols[:2]
        self.record_df = pd.DataFrame([], columns=self.record_df_cols)
        self.record_df.set_index(self.record_df_index_cols, inplace=True)

    def get_elastic_price_factors(self, sim_time, expected_pu_time=None, o_pos=None, d_pos=None):
        """This method returns current time dependent fare scales. If it is called with sim_time, the price factors
        are updated first.

        :param sim_time: current simulation time
        :type sim_time: int
        :param expected_pu_time: expected pick-up time (re
        :type expected_pu_time: float
        :param o_pos: origin of request
        :type o_pos: tuple
        :param d_pos: destination of request
        :type d_pos: tuple
        :return: tuple of base_fare_factor, distance_fare_factor, general_factor
        :rtype: (float, float, float)
        """
        o_zone = self.zone_system.get_zone_from_pos(o_pos)
        general_factor = self.current_general_factors.get(o_zone, 1.0)
        return self.current_base_fare_factor, self.current_distance_fare_factor, general_factor

    def update_current_price_factors(self, sim_time):
        """This method updates the current time dependent fare scales based on the dynamic pricing strategy.

        :param sim_time: current simulation time
        :type sim_time: int
        :return: None
        """
        if sim_time % self.dp_update_step != 0:
            return
        list_zones_all = self.zone_system.get_complete_zone_list()
        list_zones = sorted([zone for zone in list_zones_all if zone != -1])
        number_zones = len(list_zones)
        orig_zone_imbalances = np.zeros(number_zones)
        # demand forecast
        t0 = sim_time + self.horizon_start
        t1 = sim_time + self.horizon_end
        demand_fc_dict = self.zone_system.get_trip_departure_forecasts(t0, t1)
        if self.supply_fc_type == "idle_plus_trip_fc":
            # supply forecast based on idle vehicles and arrival forecast
            supply_by_zone_dict = self._get_idle_plus_trip_forecast(t0, t1)
        else:
            # supply forecast based on current state
            supply_by_zone_dict = self._get_current_state_fc(t0, t1)
        # compute imbalance per zone
        for zone in list_zones:
            orig_zone_imbalances[zone] = supply_by_zone_dict.get(zone, 0) - demand_fc_dict.get(zone, 0)
        # compute reachability corrected term
        zone_corr_matrix = self.zone_system.get_zone_correlation_matrix()
        red_zone_corr_matrix = zone_corr_matrix[np.ix_(list_zones, list_zones)]
        # beware of matrix multiplication in python! matrix is defined like this that row sum is 1!
        rc_zone_imbalances = orig_zone_imbalances @ red_zone_corr_matrix
        deficit_zone_array = np.minimum(np.array(rc_zone_imbalances).flatten(), 0)
        # computation of pricing factors
        current_general_factors = {}
        for zone in list_zones:
            deficit = deficit_zone_array[zone]
            if deficit < 0:
                max_demand = demand_fc_dict.get(zone, 0)
                if max_demand > 0:
                    # beware: mathematically, deficit is not allowed to be larger than max_demand
                    # small value of 0.01 does not affect solution if max_pricing factor is set
                    m_def = max(deficit, -max_demand+0.01)
                    pf = self.b/self.a + 1/self.a * np.log((1 + np.exp(self.a - self.b))/(1 + m_def / max_demand) - 1)
                    current_general_factors[zone] = min(self.max_pricing_factor, pf)
                    self.record_df.loc[(sim_time, zone), ["expected_deficit", "max_demand", "general_factor",
                                                          "capped_general_factor"]] = [m_def, max_demand, pf,
                                                                                       current_general_factors[zone]]
        self.current_general_factors = current_general_factors
        #
        if os.path.isfile(self.record_f):
            write_mode = "a"
            write_header = False
        else:
            write_mode = "w"
            write_header = True
        self.record_df.to_csv(self.record_f, header=write_header, mode=write_mode)
        self.record_df = pd.DataFrame([], columns=self.record_df_cols)
        self.record_df.set_index(self.record_df_index_cols, inplace=True)
        LOG.debug(f"Simulation time {sim_time}: updated current general dynamic pricing factor:\n"
                  f"{current_general_factors}")

    def _get_current_state_fc(self, t0, t1):
        """Return number of vehicles that will be / become available in each zone in time horizon [t0, t1] according
        to current vehicle plans.

        :param t0: start of time horizon
        :param t1: end of time horizon
        :return: {}: zone -> number available vehicles
        """
        zone_dict = {}
        for zone_id in self.zone_system.get_all_zones():
            zone_dict[zone_id] = 0
        for vid, current_veh_plan in self.fleetctrl.veh_plans.items():
            veh_obj = self.fleetctrl.sim_vehicles[vid]
            # 1) idle vehicles
            if not current_veh_plan.list_plan_stops:
                zone_id = self.zone_system.get_zone_from_pos(veh_obj.pos)
                if zone_id >= 0:
                    zone_dict[zone_id] += 1
            else:
                last_ps = current_veh_plan.list_plan_stops[-1]
                last_time = last_ps.planned_arrival_time
                arr, dep = last_ps.get_planned_arrival_and_departure_time()
                if dep is not None:
                    last_time = dep
                else:
                    last_time = arr
                if t0 <= last_time < t1:
                    zone_id = self.zone_system.get_zone_from_pos(last_ps.pos)
                    zone_dict[zone_id] += 1
        return zone_dict

    def _get_idle_plus_trip_forecast(self, t0, t1):
        """Return number of vehicles that are idle and are expected to arrive in each zone in time horizon [t0, t1]
        according to trip forecasts.

        :param t0: start of time horizon
        :param t1: end of time horizon
        :return: {}: zone -> number available vehicles
        """
        zone_dict = {}
        for zone_id in self.zone_system.get_all_zones():
            zone_dict[zone_id] = 0
        for vid, current_veh_plan in self.fleetctrl.veh_plans.items():
            veh_obj = self.fleetctrl.sim_vehicles[vid]
            # 1) idle vehicles
            if not current_veh_plan.list_plan_stops:
                zone_id = self.zone_system.get_zone_from_pos(veh_obj.pos)
                if zone_id >= 0:
                    zone_dict[zone_id] += 1
        arrival_fc_dict = self.zone_system.get_trip_arrival_forecasts(t0, t1)
        for zone_id, arr_fc_val in arrival_fc_dict.items():
            zone_dict[zone_id] += arr_fc_val
        return zone_dict
