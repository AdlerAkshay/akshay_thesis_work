# -------------------------------------------------------------------------------------------------------------------- #
# standard distribution imports
# -----------------------------
import logging
import os
from copy import deepcopy
from abc import abstractmethod, ABCMeta

# additional module imports (> requirements)
# ------------------------------------------
import numpy as np
import pandas as pd
pd.options.mode.chained_assignment = None  # TODO # disables warning when overwriting Dataframes

# src imports
# -----------
from src.misc.functions import PiecewiseContinuousLinearFunction
from src.demand.TravelerModels import RequestBase, offer_str
# -------------------------------------------------------------------------------------------------------------------- #
# global variables
# ----------------
from src.misc.globals import *

LOG = logging.getLogger(__name__)

"""
Wed Jun 10 11:42:59 2020
@author: Gabriel
"""
# ratio of seats over total capacity
ratio_seats_total_bus = 48 / (
            48 + 55)  # source: https://www.mvg.de/ueber/das-unternehmen/fahrzeuge.html, MAN Lions City (G)
ratio_seats_total_tram = 69 / 147  # source: https://www.mvg.de/ueber/das-unternehmen/fahrzeuge.html, T1 - Avenio
ratio_seats_total_metro = 220 / 940  # source: https://www.mvg.de/ueber/das-unternehmen/fahrzeuge.html, C2
ratio_seats_total_sbahn = (176 + 16) / (176 + 16 + 352)  # source: https://de.wikipedia.org/wiki/DB-Baureihe_423
# should be weighted according to vehicles in operation and respective capacities at each time-step;
# for simplicity, all PT modes weighed the same
# TODO # change for first revision
total_ratio = ratio_seats_total_bus * 0.25 + ratio_seats_total_tram * 0.25 + ratio_seats_total_metro * 0.25 \
              + ratio_seats_total_sbahn * 0.25
# =============================================================================
#     eta_max is derived based on visual inspection of figure 6 from
#     Tirachini et al. (2017) Estimation of crowding discomfort in public transport: Results from Santiago de Chile
#     for 4pax/m2
# =============================================================================
eta_max = (1.5 + 2 + 1.9 + 1.7 + 1.7) / 5
rel_crowding = [0, total_ratio, 1, 2]
vot_factor = [1, 1, eta_max, 10000]
VOT_OVER_CROWDING_F = PiecewiseContinuousLinearFunction(list(zip(rel_crowding, vot_factor)))


def vot_over_crowding(rel_crowding_factor):
    """This function calls a static piecewise linear function that describes the value of time as a function of
    crowding in a PT system.

    :param rel_crowding_factor: relative crowding factor [0,1] describing the utilized capacity of the PT system
    :return: scale factor of value of (travel time)
    """
    return VOT_OVER_CROWDING_F.get_y(rel_crowding_factor)


class ChoicePtPvAmodInter(RequestBase):
    """Description of this traveler model in ISTTT2020 paper."""
    type = "ChoicePtPvAmodInter"

    def __init__(self, rq_row, routing_engine, simulation_time_step, scenario_parameters):
        super().__init__(rq_row, routing_engine, simulation_time_step, scenario_parameters)
        self.im_subsidy = 0
        self.im_pt_fare = 0
        self.pv_toll = 0
        self.pv_park = 0
        self.chosen_utility = None

    def _add_record(self, record_dict):
        """This method enables the output of Traveler Model specific output

        :param record_dict: standard record output
        :return: extended record output
        """
        record_dict[G_RQ_SUB] = self.im_subsidy
        record_dict[G_RQ_IM_PT_FARE] = self.im_pt_fare
        record_dict[G_RQ_TOLL] = self.pv_toll
        record_dict[G_RQ_PARK] = self.pv_park
        record_dict[G_RQ_C_UTIL] = self.chosen_utility
        return record_dict

    def choose_offer(self, sc_pars, sim_time):
        """This method returns the key of the chosen offer, i.e. the operator id of the chosen mode in case of AMoD.
        -1: public transport
        -2: private vehicle
        0..n: MoD fleet provider
        -3..-(n+3): intermodal transport with MoD provider n
        None: none of the above

        :param sc_pars: scenario parameters containing coefficients of mode-choice model
        :type sc_pars: dict
        :return: chosen_offer_key
        """
        # generate utilities and probabilities
        raise DeprecationWarning("this traveler class might no longer be up to date")
        util = {}
        rel_probabilities = {}
        for k, tmp in self.offer.items():
            # public transportation offer
            if k == G_MC_DEC_PT:
                util[k] = -tmp[G_OFFER_FARE] \
                          - sc_pars[G_MC_VOT] * vot_over_crowding(tmp[G_OFFER_CROWD]) * tmp[G_OFFER_DRIVE] \
                          - sc_pars[G_MC_VOT] * (tmp.get(G_OFFER_ACCESS_W, 0) + tmp.get(G_OFFER_EGRESS_W, 0)) \
                          - sc_pars[G_MC_TRANSFER_P] * tmp[G_OFFER_TRANSFERS]
            # private vehicle "offer"/option
            elif k == G_MC_DEC_PV:
                util[k] = sc_pars[G_MC_U0_PV] - sc_pars[G_MC_VOT] * tmp[G_OFFER_DRIVE] \
                          - sc_pars[G_MC_C_D_PV] * tmp[G_OFFER_DIST] - tmp[G_OFFER_TOLL] - tmp[G_OFFER_PARK]
            # AMoD
            elif k >= 0:
                util[k] = -tmp[G_OFFER_FARE] - sc_pars[G_MC_VOT] * (tmp[G_OFFER_DRIVE] + tmp[G_OFFER_WAIT])
            # Inter-Modal offers
            elif k < -2:
                pt_tt = tmp[G_IM_OFFER_PT_END] - tmp[G_IM_OFFER_PT_START]
                util[k] = -tmp[G_IM_OFFER_PT_COST] - tmp[G_IM_OFFER_MOD_COST] + tmp[G_IM_OFFER_MOD_SUB] \
                          - sc_pars[G_MC_VOT] * (vot_over_crowding(tmp[G_OFFER_CROWD]) * pt_tt
                                                 + tmp[G_OFFER_ACCESS_W] + tmp[G_OFFER_EGRESS_W]
                                                 + tmp[G_IM_OFFER_MOD_DRIVE] + tmp[G_OFFER_WAIT]) \
                          - sc_pars[G_MC_TRANSFER_P] * tmp[G_OFFER_TRANSFERS]
            # logit model: p(m) = exp[-U_m] / (sum_m exp[-U_m]) -> normalization is handled by distributions-module
            # exp[-110] = 0 -> but it should still be larger than 0! computation in euro instead of cent
            prob = np.exp(util[k] / 100)
            if prob == np.inf:
                LOG.warning("infinite probability in mode choice")
            rel_probabilities[k] = np.exp(util[k] / 100)
        # mode choice
        LOG.debug("evaluate mode choice for rid {}: util: {} : rel_probs: {}".format(self.rid, util, rel_probabilities))
        chosen_offer_key = draw_from_distribution_dict(rel_probabilities)
        self.service_opid = chosen_offer_key
        # set attributes (chosen utility, subsidy, ...)
        self.chosen_utility = util[chosen_offer_key]
        tmp = self.offer[chosen_offer_key]
        self.fare = tmp.get(G_OFFER_FARE, 0)
        self.im_subsidy = tmp.get(G_IM_OFFER_MOD_SUB, 0)
        self.im_pt_fare = tmp.get(G_IM_OFFER_PT_COST, 0)
        self.pv_toll = tmp.get(G_OFFER_TOLL, 0)
        self.pv_park = tmp.get(G_OFFER_PARK, 0)
        return chosen_offer_key
# -------------------------------------------------------------------------------------------------------------------- #

default_number_of_passenger_distribution =\
    [0.7641080928774975, 0.18213156786785017, 0.04341258563180729, 0.010347753622845077]


def distributed_number_of_passengers(distribution=None):
    """This function returns a random number of passengers based on a given (or default) distribution

    :param distribution: probability distribution for certain numbers of passengers
    :type distribution: list or None
    :return: number of passengers (int)"""
    if distribution is None:
        p = default_number_of_passenger_distribution
    else:
        p = distribution
    x = np.random.random()
    s = p[0]
    pax = len(p)
    for i in range(1, len(p)):
        if x < s:
            pax = i
            break
        else:
            s += p[i]
    return pax


class BMWStudyRequest(RequestBase):
    """This request is used for BMW project RidePooling Simulation
    request is sensitive to waiting_times:
    - all offers are accepted if waiting time is below G_AR_MAX_WT
    - all offers are decline if waiting time is higher than G_AR_MAX_WT_2
    - linear decrease of probability of acceptance between G_AR_MAX_WT and G_AR_MAX_WT_2
    """
    type = "BMWStudyRequest"

    def __init__(self, rq_row, routing_engine, simulation_time_step, scenario_parameters):
        super().__init__(rq_row, routing_engine, simulation_time_step, scenario_parameters)
        self.nr_pax = distributed_number_of_passengers()

    def choose_offer(self, sc_parameters, simulation_time):
        LOG.debug("choose offer {}".format(offer_str(self.offer)))
        test_all_decline = super().choose_offer(sc_parameters, simulation_time)
        if test_all_decline is not None and test_all_decline < 0:
            return -1
        if len(self.offer) == 0:
            return None
        elif len(self.offer) == 1:
            op = list(self.offer.keys())[0]
            if self.offer[op].service_declined:
                LOG.debug(" -> no offer!")
                return -1
            wt = self.offer[op][G_OFFER_WAIT]
            if wt <= sc_parameters[G_AR_MAX_WT]:
                LOG.debug(f" -> accept {wt} <= {sc_parameters[G_AR_MAX_WT]}")
                self.fare = self.offer[op].get(G_OFFER_FARE, 0)
                return op
            elif wt > sc_parameters[G_AR_MAX_WT_2]:
                LOG.debug(f" -> decline. too long?? {wt} > {sc_parameters[G_AR_MAX_WT_2]}")
                return -1
            else:
                acc_prob = (sc_parameters[G_AR_MAX_WT_2] - wt) / (
                            sc_parameters[G_AR_MAX_WT_2] - sc_parameters[G_AR_MAX_WT])
                r = np.random.random()
                LOG.debug(f" -> random prob {acc_prob}")
                if r < acc_prob:
                    LOG.debug(f" -> accept")
                    self.fare = self.offer[op].get(G_OFFER_FARE, 0)
                    return op
                else:
                    LOG.debug(f" -> decline")
                    return -1
        else:
            LOG.error(f"not implemented {offer_str(self.offer)}")
            raise NotImplementedError
# -------------------------------------------------------------------------------------------------------------------- #


class IRSStudyRequest(RequestBase):
    """This request is used for the IRS study simulations"""
    # type = "IRSStudyRequest"

    def __init__(self, rq_row, routing_engine, simulation_time_step, scenario_parameters):
        super().__init__(rq_row, routing_engine, simulation_time_step, scenario_parameters)
        self.nr_pax = distributed_number_of_passengers([0.75, 0.15, 0.07, 0.03])

    def choose_offer(self, sc_parameters, simulation_time):
        LOG.debug("choose offer {}".format(offer_str(self.offer)))
        test_all_decline = super().choose_offer(sc_parameters, simulation_time)
        if test_all_decline is not None and test_all_decline < 0: # all operators decline service
            return -1
        if len(self.offer) == 0:
            return None
        elif len(self.offer) == 1:
            op = list(self.offer.keys())[0]
            if self.offer[op].service_declined():
                LOG.debug(" -> no offer!")
                return -1
            wt = self.offer[op][G_OFFER_WAIT]
            if wt <= sc_parameters[G_AR_MAX_WT]:
                LOG.debug(f" -> accept {wt} <= {sc_parameters[G_AR_MAX_WT]}")
                self.fare = self.offer[op].get(G_OFFER_FARE, 0)
                return op
            else:
                LOG.debug(f" -> decline. too long?? {wt} > {sc_parameters[G_AR_MAX_WT]}")
                return -1
        else:
            LOG.error(f"not implemented {offer_str(self.offer)}")
            raise NotImplementedError
# -------------------------------------------------------------------------------------------------------------------- #



def heterogeneous_decision_groups(number_of_groups=4, distribution_of_groups="random"):
    """This function returns the group a customer is associated with. Depending on this group the customer may have
    different decision_process_parameters
    :param number_of_groups: number of available groups
    :type number_of_groups: int
    :param distribution_of_groups: type of distribution between the available groups
    :type distribution_of_groups: string, only 'random' available at the moment
    :return: group of the customer (int)
    """
    if distribution_of_groups == 'random':
        decision_group = np.random.randint(number_of_groups)
    else:
        LOG.warning("Distribution type of decision group not specified. Group is set to None.")
        decision_group = None

    return decision_group


def heterogeneous_decision_process_parameters(default_parameters=None):
    """This function returns a set of decision process parameters, either based on default parameters (if given) or
    random values within constraints defined within the function for each parameter

    :param default_parameters: set of parameters that are not supposed to be randomly distributed but fixed
    :type default_parameters: dict or None
    :return: set of parameters
    """
    if default_parameters is None:
        default_parameters = {}
    thresh_constraints = ['int', 30, 50]  #
    ssp_constraints = ['int', 80, 120]
    feedback_constraints = ['float', 0.99, 0.999]
    diff_rate_constraints = ['float', 2, 4]  #
    waiting_time_factor = ['float', 1, 1.5]
    reaction_time = ['int', 1, 3]
    parameter_dict = {G_RQ_DEC_THRESH: thresh_constraints, G_RQ_DEC_SPP: ssp_constraints,
                      G_RQ_DEC_FEEDBACK: feedback_constraints, G_RQ_DEC_DIFF_RATE: diff_rate_constraints,
                      G_RQ_DEC_WT_FAC: waiting_time_factor, G_RQ_DEC_REAC: reaction_time}
    parameters_to_return = {}
    for param, constraints in parameter_dict.items():
        if param in default_parameters:
            parameters_to_return[param] = default_parameters[param]
        else:
            if constraints[0] == 'int':
                random_value = np.random.random_integers(constraints[1],constraints[2])
            elif constraints[0] == 'float':
                random_value = np.random.uniform(constraints[1], constraints[2])
            else:
                random_value = np.random.random()
            parameters_to_return[param] = random_value
    return parameters_to_return


def decision_parameter_sets_by_group(customer):
    """This function returns parameter sets according to the group of a customer.
    :param customer: customer, that is about to get the parameters
    :type customer: request object
    :return: parameter set (dict)
    """
    if customer.decision_group is None:
        return {}
    parameters_to_return = {}
    if customer.decision_group == 0:  # low age, low perceived pressure
        parameters_to_return[G_RQ_DEC_THRESH] = 40
        parameters_to_return[G_RQ_DEC_DIFF_RATE] = 1
        parameters_to_return[G_RQ_DEC_FEEDBACK] = 0.99
        parameters_to_return[G_RQ_DEC_WT_FAC] = 1
        parameters_to_return[G_RQ_DEC_REAC] = 1
    elif customer.decision_group == 1:  # low age, high perceived pressure
        parameters_to_return[G_RQ_DEC_THRESH] = 30
        parameters_to_return[G_RQ_DEC_DIFF_RATE] = 3
        parameters_to_return[G_RQ_DEC_FEEDBACK] = 0.99
        parameters_to_return[G_RQ_DEC_WT_FAC] = 1.5
        parameters_to_return[G_RQ_DEC_REAC] = 1
    elif customer.decision_group == 2:  # high age, low perceived pressure
        parameters_to_return[G_RQ_DEC_THRESH] = 50
        parameters_to_return[G_RQ_DEC_DIFF_RATE] = 1
        parameters_to_return[G_RQ_DEC_FEEDBACK] = 0.98
        parameters_to_return[G_RQ_DEC_WT_FAC] = 1
        parameters_to_return[G_RQ_DEC_REAC] = 3
    elif customer.decision_group == 3:  # high age, high perceived pressure
        parameters_to_return[G_RQ_DEC_THRESH] = 40
        parameters_to_return[G_RQ_DEC_DIFF_RATE] = 3
        parameters_to_return[G_RQ_DEC_FEEDBACK] = 0.98
        parameters_to_return[G_RQ_DEC_WT_FAC] = 1.5
        parameters_to_return[G_RQ_DEC_REAC] = 3
    else:
        LOG.warning("Decision group not defined")
        raise NotImplementedError
    return parameters_to_return


class DiffusionModelRequest(RequestBase):
    """This request is used for simulations with more sophisticated user models based on Yu and Hyland, 2020"""
    type = "DiffusionModelRequest"

    def __init__(self, rq_row, routing_engine, simulation_time_step, scenario_parameters):
        super().__init__(rq_row, routing_engine, simulation_time_step, scenario_parameters)

        self.nr_pax = distributed_number_of_passengers([0.75, 0.15, 0.07, 0.03])

        # fixed_decision_parameters = {G_RQ_DEC_THRESH: 40, G_RQ_DEC_SPP: 100, G_RQ_DEC_FEEDBACK: 0.991,
        #                              G_RQ_DEC_DIFF_RATE: 3.1}
        # individual_decision_parameters = heterogeneous_decision_process_parameters(fixed_decision_parameters)
        individual_decision_parameters = heterogeneous_decision_process_parameters({G_RQ_DEC_SPP:100,
                                                                                    G_RQ_DEC_FEEDBACK:0.991})
        self.decision_group = heterogeneous_decision_groups()
        groupwise_decision_parameters = decision_parameter_sets_by_group(self)
        self.decision_threshold = groupwise_decision_parameters.get(G_RQ_DEC_THRESH,
                                                                    individual_decision_parameters[G_RQ_DEC_THRESH])
        self.decision_steps_per_process = groupwise_decision_parameters.get(G_RQ_DEC_SPP,
                                                                            individual_decision_parameters[G_RQ_DEC_SPP])
        self.decision_feedback_rate = groupwise_decision_parameters.get(G_RQ_DEC_FEEDBACK,
                                                                        individual_decision_parameters[G_RQ_DEC_FEEDBACK])
        self.decision_diffusion_rate = groupwise_decision_parameters.get(G_RQ_DEC_DIFF_RATE,
                                                                         individual_decision_parameters[G_RQ_DEC_DIFF_RATE])
        self.decision_wt_factor = groupwise_decision_parameters.get(G_RQ_DEC_WT_FAC,
                                                                    individual_decision_parameters[G_RQ_DEC_WT_FAC])
        self.decision_reaction_time = groupwise_decision_parameters.get(G_RQ_DEC_REAC,
                                                                        individual_decision_parameters[G_RQ_DEC_REAC])
        self.reaction_time_left = int(self.decision_reaction_time)
        self.simulation_time_step = simulation_time_step
        self.offer_quality = None
        self.max_offer_quality = 1
        self.min_offer_quality = -1
        self.initial_decision_preference_state = 0
        self.decision_preference_state = self.initial_decision_preference_state
        self.decision_process_history = []
        self.nr_decisions = 0
        self.all_stats = []
        self.max_decisions = rq_row.get(G_RQ_DEC_MAX, 60)
        self.latest_decision_time = self.rq_time + self.max_decisions*self.simulation_time_step
        self.decision = 0
        self.chosen_operator_id = None

    def user_boards_vehicle(self, simulation_time, op_id, vid, pu_pos, t_access):
        super().user_boards_vehicle(simulation_time, op_id, vid, pu_pos, t_access)
        self.decision = 1
        self.update_all_stats()

    def cancels_booking(self, sim_time):
        if self.decision != 0:
            return False
        decision = self.choose_offer(None, sim_time)
        if decision is not None and decision < 0:
            return True
        else:
            False

    def choose_offer(self, scenario_parameters, simulation_time):
        if len(self.offer) == 1:
            op = list(self.offer.keys())[0]
            if self.offer[op].offered_waiting_time is None:
                LOG.debug(" -> no offer!")
                return -1
        elif len(self.offer) == 0:
            LOG.debug(" -> no offer!")
            return -1
        decision = self.decision_process()
        if decision is not None and decision >= 0:
            self.chosen_operator_id = list(self.offer.keys())[0]
            return self.chosen_operator_id
        else:
            return decision

    '''
    def choose_offer(self, sc_parameters, simulation_time):
        raise DeprecationWarning("dont know if the choose_offer process here is still uptodate") # TODO #
        # opts = [offer_id for offer_id, offer_val in self.offer.items() if offer_val is not None and len(offer_val.keys()) > 0]
        LOG.debug("choose offer {}".format(offer_str(self.offer)))
        if len(self.offer) == 0:
            return None
        elif len(self.offer) == 1:
            op = list(self.offer.keys())[0]
            if self.offer[op].service_declined():
                LOG.debug(" -> no offer!")
                return None
            else:
                self.chosen_operator_id = op
                return op
        else:
            LOG.error(f"not implemented {offer_str(self.offer)}")
            raise NotImplementedError
    '''

    def receive_offer(self, operator_id, operator_offer, simulation_time, sc_parameters=None):
        # Check if offer changed, otherwise return
        if operator_id in self.offer and self.offer[operator_id] == operator_offer:
            return
        self.offer[operator_id] = operator_offer
        if sc_parameters is None:
            self.offer_quality = 0
        #elif not operator_offer:
        elif operator_offer.service_declined():
            LOG.debug(f" -> no offer: waiting_time > op_max_wait_time")
        else:
            # wt = self.offer[operator_id][G_OFFER_WAIT]
            wt = self.offer[operator_id].offered_waiting_time
            if wt <= sc_parameters[G_AR_MAX_WT]*self.decision_wt_factor:
                LOG.debug(f" -> good offer {wt} <= {sc_parameters[G_AR_MAX_WT]}")
                self.offer_quality = self.max_offer_quality
            elif wt > sc_parameters[G_AR_MAX_WT_2]*self.decision_wt_factor:
                LOG.debug(f" -> bad offer. too long {wt} > {sc_parameters[G_AR_MAX_WT_2]}")
                self.offer_quality = self.min_offer_quality
            else:
                self.offer_quality = ((self.min_offer_quality - self.max_offer_quality) /
                                      (self.decision_wt_factor*(sc_parameters[G_AR_MAX_WT_2] - sc_parameters[G_AR_MAX_WT]))) * \
                                     (wt - sc_parameters[G_AR_MAX_WT]*self.decision_wt_factor) + self.max_offer_quality
                LOG.debug(f" -> offer quality {self.offer_quality}")
        self.decision_preference_state = self.initial_decision_preference_state
        self.nr_decisions = 0
        self.decision = 0

    def decision_process(self):
        if self.nr_decisions >= self.max_decisions:
            if self.decision_preference_state > 0:
                self.decision = 1
            else:
                self.decision = -1
        elif self.reaction_time_left > 0:
            self.reaction_time_left -= 1
            self.nr_decisions += 1
        else:
            self.nr_decisions += 1
            for i in range(self.decision_steps_per_process):
                random_epsilon = np.random.normal(scale=self.decision_diffusion_rate)
                self.decision_preference_state = self.decision_feedback_rate * self.decision_preference_state +\
                                                                               self.offer_quality + random_epsilon
                self.decision_process_history.append(self.decision_preference_state)
                if self.decision_preference_state >= self.decision_threshold:
                    self.decision = 1
                    break
                elif self.decision_preference_state <= -self.decision_threshold:
                    self.decision = -1
                    break
        if self.decision != 0:
            self.update_all_stats()
            return self.decision

    def update_all_stats(self):
        self.all_stats.append({'offer_quality': self.offer_quality, 'duration': self.nr_decisions,
                               'decision': self.decision})

    def _add_record(self, record_dict):
        """This method enables the output of Traveler Model specific output

        :param record_dict: standard record output
        :return: extended record output
        """
        record_dict['decision_group'] = self.decision_group
        record_dict['decision_threshold'] = self.decision_threshold
        record_dict['decision_steps_per_process'] = self.decision_steps_per_process
        record_dict['decision_feedback_rate'] = self.decision_feedback_rate
        record_dict['decision_diffusion_rate'] = self.decision_diffusion_rate
        record_dict['decision_waiting_time_factor'] = self.decision_wt_factor
        record_dict['decision_reaction_time'] = self.decision_reaction_time
        record_dict['decision_history_summary'] = [np.round(x,2) for a, x in enumerate(self.decision_process_history) if
                                                   a % self.decision_steps_per_process == 0]
        record_dict['stats'] = self.all_stats
        record_dict['final_offer_quality'] = self.offer_quality
        if self.chosen_operator_id is not None and \
                self.offer[self.chosen_operator_id].additional_offer_parameters.get('locked') and self.decision < 0:
            record_dict['customer_rejects_second_offer'] = True
        else:
            record_dict['customer_rejects_second_offer'] = False
        record_dict['final_decision_process_duration_in_seconds'] = self.nr_decisions * self.simulation_time_step
        return record_dict

class ExChangeRequest(RequestBase):
    """This request class is used for the easyride broker exchange simulation.
    it is supposed to receive multiple offers from different amod operators
    # rid chooses:
    # 1) pref op, if offer and willing
    # 2) else: other op if offer and willing
    # 3) pref op, if offer
    """
    type = "ExChangeRequest"

    def choose_offer(self, scenario_parameters, simulation_time):
        test_all_decline = super().choose_offer(scenario_parameters, simulation_time)
        if test_all_decline is not None and test_all_decline < 0: # all operators declined
            return -1
        list_options = [i for i, off in self.offer.items() if not off.service_declined()]
        if len(self.offer) == 0:
            return None
        if len(list_options) == 0:
            return -1
        if len(list_options) == 1:
            self.fare = self.offer[list_options[0]].get(G_OFFER_FARE, 0)
            return list_options[0]
        elif len(list_options) == 2:
            other_op = [i for i in list_options if i != self.preferred_operator][0]
            if self.preferred_operator in list_options:
                if self.offer[self.preferred_operator][G_OFFER_WILLING_FLAG]:
                    self.fare = self.offer[self.preferred_operator].get(G_OFFER_FARE, 0)
                    return self.preferred_operator
                elif self.offer[other_op][G_OFFER_WILLING_FLAG]:
                    self.fare = self.offer[other_op].get(G_OFFER_FARE, 0)
                    return other_op
                else:
                    self.fare = self.offer[self.preferred_operator].get(G_OFFER_FARE, 0)
                    return self.preferred_operator
            else:
                raise EnvironmentError("EasyRideBrokerExChangRequest got multiple offers but none from preferred operator!")
        else:
            raise EnvironmentError("Too many offers for EasyRideBrokerExChangeRequest! operators with offer: {}".format(list_options))

    def _add_record(self, record_dict):
        """This method enables the output of Traveler Model specific output
        -> add preferred operator
        :param record_dict: standard record output
        :return: extended record output
        """
        record_dict["preferred_operator"] = self.preferred_operator
        return record_dict

class MOIATestRequest(RequestBase):
    """this request is used for moia calibration. the additional column entry "served" indicates if the rquest was served by
    MOIA in reality. if thats not the case, this request will automatically decline an offer from the fleet control"""
    type = "MOIATestRequest"

    def __init__(self, rq_row, routing_engine, simulation_time_step, scenario_parameters):
        super().__init__(rq_row, routing_engine, simulation_time_step, scenario_parameters)
        self.was_served = rq_row["served"]

    def choose_offer(self, scenario_parameters, simulation_time):
        LOG.debug(f"MOIATestRequest request choose offer: {self.rid} : {offer_str(self.offer)}")
        test_all_decline = super().choose_offer(scenario_parameters, simulation_time)
        if test_all_decline is not None and test_all_decline < 0: # all operators declined
            return -1
        opts = [offer_id for offer_id, operator_offer in self.offer.items() if
                operator_offer is not None and not operator_offer.service_declined()]
        if len(self.offer) == 0:
            return None
        if len(opts) == 0:
            return -1
        elif len(opts) == 1:
            if not self.was_served:
                return -1
            else:
                return opts[0]
        else:
            LOG.error(f"not implemented {self.offer}")

    def user_boards_vehicle(self, simulation_time, op_id, vid, pu_pos, t_access):
        return super().user_boards_vehicle(simulation_time, op_id, vid, pu_pos, t_access)
    