# -------------------------------------------------------------------------------------------------------------------- #
# standard distribution imports
# -----------------------------
import logging
# import os
# import pandas as pd
# import numpy as np

# additional module imports (> requirements)
# ------------------------------------------
# from IPython import embed
# import time

# src imports
# -----------
from abc import ABC

from src.FleetSimulationBase import FleetSimulationBase

# -------------------------------------------------------------------------------------------------------------------- #
# global variables
# ----------------
from src.misc.globals import *
LOG = logging.getLogger(__name__)

# TODO # can the user interaction be modelled by IOS(immediate offer simulation)-Model with new TravelerModel?

# -------------------------------------------------------------------------------------------------------------------- #
# functions
# ---------


# -------------------------------------------------------------------------------------------------------------------- #
# main
# ----
class CombineIRSpoolingNCMsimulation(FleetSimulationBase, ABC):
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
        pass

    def step(self, sim_time):
        # 1)
        self.update_sim_state_fleets(sim_time - self.time_step, sim_time)
        new_travel_times = self.routing_engine.update_network(sim_time)  # TODO #
        if new_travel_times:
            for op_id in range(self.n_op):
                self.operators[op_id].inform_network_travel_time_update(sim_time)
        last_time = sim_time - self.time_step
        if last_time < self.start_time:
            last_time = None
        # 2)
        list_new_traveler_rid_obj = []
        if last_time is not None:
            # requests are saved in seconds internally
            for t in range(last_time + 1, sim_time):
                list_new_traveler_rid_obj.extend(self.demand.get_new_travelers(t))
        list_new_traveler_rid_obj.extend(self.demand.get_new_travelers(sim_time))
        last_time = sim_time
        # 3)
        unassigned_rqs = {}
        for rid, rq_obj in list_new_traveler_rid_obj:
            for op_id in range(self.n_op):
                LOG.debug(f"Request {rid}: To operator {op_id} ...")
                self.operators[op_id].user_request(rq_obj, sim_time)
                unassigned_rqs[rid] = rq_obj
        # 4)
        new_offers = {}
        for op_id, op_obj in enumerate(self.operators):
            rid_to_offer = op_obj.time_trigger(sim_time)  # here offers are created in batch assignment
            for rid, offer in rid_to_offer.items():
                LOG.debug(f"Offers received for rid {rid} : {offer}")
                # first offer
                rq_obj = unassigned_rqs.get(rid)
                if rq_obj:
                    rq_obj.receive_offer(op_id, offer, sim_time, self.scenario_parameters)
                    new_offers[rid] = rq_obj
                # ... possible update of offer (not necessary here)
        # 5)
        # self._check_waiting_request_cancellations(sim_time)
        LOG.debug(f"sim time {sim_time}")
        LOG.debug(f"new offers : {new_offers}")
        LOG.debug(f"unassigned rqs : {unassigned_rqs.keys()}")
        # 6)
        if len(new_offers) > 0:
            for rid, rq_obj in new_offers.items():
                LOG.debug(f"Request {rid}: calling mode choice ...")
                chosen_operator = rq_obj.choose_offer(self.scenario_parameters, sim_time)
                LOG.debug(f"Request {rid}: choice: {chosen_operator} ...")
                for op_id, op_obj in enumerate(self.operators):
                    # this will create an error for multiple operators in case of an assignment retry ( TODO )
                    if not chosen_operator == op_id:
                        op_obj.user_cancels_request(rid, sim_time)
                if chosen_operator is None or chosen_operator < 0:
                    # TODO # operator id, if no operator is chosen?
                    self.demand.record_user(rid)
                    del self.demand.rq_db[rid]
                del unassigned_rqs[rid]
        # 7)
        cancelling_customers = []
        for rid, rq_obj in self.demand.rq_db.items():
            if rq_obj.service_vid:
                rq_obj.decision = 1  # in case a customer was picked up before second decision was made
            if rq_obj.decision == 0:
                self.demand.rq_db[rid].decision_process()
                if self.demand.rq_db[rid].decision != 0:
                    for op_id, op_obj in enumerate(self.operators):
                        if op_id == self.demand.rq_db[rid].chosen_operator_id:
                            if self.demand.rq_db[rid].decision == 1:
                                if rid not in self.demand.waiting_rq:
                                    self.demand.waiting_rq[rid] = rq_obj
                                op_obj.user_confirms_booking(rid, sim_time)
                                continue
                        if self.demand.rq_db[rid].decision == -1:
                            cancelling_customers.append((rid, op_obj))
        # 8)
        for rid, op_obj in cancelling_customers:
            # print(f"{rid} cancels request at {sim_time}")
            op_obj.user_cancels_request(rid, sim_time)
            self.demand.record_user(rid)
            del self.demand.rq_db[rid]
        self.record_stats(force=False)

    def add_evaluate(self):
        pass

    
    # from some previous merge
    '''
    def run(self):
        """
        loop over time:
            # 1) update fleets, count moving fleet vehicles per cluster of last time steps
            # 2) get new travelers
            # 3) call ride pooling optimisation
        """
        if not self._started:
            last_time = None
            self._started = True
            unassigned_rqs = {}
            list_of_opt_period_times = []
            for i in self.operators:
                list_of_opt_period_times.append(i.optimisation_time_step)
            for sim_time in range(self.start_time, self.end_time, self.time_step):
                # print("simulation time: "+str(sim_time))
                # 1)
                self.update_sim_state_fleets(sim_time - self.time_step, sim_time)
                # 2)
                new_travel_times = self.routing_engine.update_network(sim_time)  # TODO #
                if new_travel_times:
                    for op_id in range(self.n_op):
                        self.operators[op_id].inform_network_travel_time_update(sim_time)
                # 3)
                list_new_traveler_rid_obj = []
                if last_time is not None:
                    # requests are saved in seconds internally
                    for t in range(last_time + 1, sim_time):
                        list_new_traveler_rid_obj.extend(self.demand.get_new_travelers(t))
                list_new_traveler_rid_obj.extend(self.demand.get_new_travelers(sim_time))
                last_time = sim_time
                # 4)
                for rid, rq_obj in list_new_traveler_rid_obj:
                    # 5)
                    # 5c) MoD
                    for op_id in range(self.n_op):
                        LOG.debug(f"Request {rid}: To operator {op_id} ...")
                        self.operators[op_id].user_request(rq_obj, sim_time)
                        unassigned_rqs[rid] = rq_obj
                new_offers = {}
                for op_id, op_obj in enumerate(self.operators):
                    rid_to_offer = op_obj.time_trigger(sim_time)    # here offers are created in batch assignment
                    for rid, offer in rid_to_offer.items():
                        LOG.debug(f"Offers received for rid {rid} : {offer}")
                        # first offer
                        rq_obj = unassigned_rqs.get(rid)
                        if rq_obj:
                            rq_obj.receive_offer(op_id, offer, sim_time, self.scenario_parameters)
                            new_offers[rid] = rq_obj
                        # ... possible update of offer (not necessary here)
                LOG.debug(f"sim time {sim_time}")
                LOG.debug(f"new offers : {new_offers}")
                LOG.debug(f"unassigned rqs : {unassigned_rqs.keys()}")
                if len(new_offers) > 0:
                    for rid, rq_obj in new_offers.items():
                        # 5e) mode choice and consequences
                        LOG.debug(f"Request {rid}: calling mode choice ...")
                        chosen_operator = rq_obj.choose_offer(self.scenario_parameters, sim_time)
                        LOG.debug(f"Request {rid}: choice: {chosen_operator} ...")
                        for op_id, op_obj in enumerate(self.operators):
                            # this will create an error for multiple operators in case of an assignment retry ( TODO )
                            if not chosen_operator == op_id:
                                op_obj.user_cancels_request(rid, sim_time)
                        if chosen_operator is None or chosen_operator < 0:
                            # TODO # operator id, if no operator is chosen?
                            self.demand.record_user(rid)
                            del self.demand.rq_db[rid]
                        del unassigned_rqs[rid]
                cancelling_customers = []
                for rid, rq_obj in self.demand.rq_db.items():
                    if rq_obj.service_vid:
                        rq_obj.decision = 1  # in case a customer was picked up before second decision was made
                    if rq_obj.decision == 0:
                        self.demand.rq_db[rid].decision_process()
                        if self.demand.rq_db[rid].decision != 0:
                            for op_id, op_obj in enumerate(self.operators):
                                if op_id == self.demand.rq_db[rid].chosen_operator_id:
                                    if self.demand.rq_db[rid].decision == 1:
                                        op_obj.user_confirms_booking(rid, sim_time)
                                        continue
                                if self.demand.rq_db[rid].decision == -1:
                                    cancelling_customers.append((rid, op_obj))
                for rid, op_obj in cancelling_customers:
                    # print(f"{rid} cancels request at {sim_time}")
                    op_obj.user_cancels_request(rid, sim_time)
                    self.demand.record_user(rid)
                    del self.demand.rq_db[rid]
                self.record_stats(force=False)
            # save final state, record remaining travelers and vehicle tasks
            self.save_final_state()
            self.record_remaining_assignments()
        # call evaluation
        self.evaluate()
    '''    

    #from last merge (18.04.2021)
    '''
    def step(self, sim_time):
        # 1)
        self.update_sim_state_fleets(sim_time - self.time_step, sim_time)
        new_travel_times = self.routing_engine.update_network(sim_time)  # TODO #
        if new_travel_times:
            for op_id in range(self.n_op):
                self.operators[op_id].inform_network_travel_time_update(sim_time)
        last_time = sim_time - self.time_step
        if last_time < self.start_time:
            last_time = None
        # 2)
        list_new_traveler_rid_obj = []
        if last_time is not None:
            # requests are saved in seconds internally
            for t in range(last_time + 1, sim_time):
                list_new_traveler_rid_obj.extend(self.demand.get_new_travelers(t))
        list_new_traveler_rid_obj.extend(self.demand.get_new_travelers(sim_time))
        # last_time = sim_time
        # 3)
        # unassigned_rqs = {}
        for rid, rq_obj in list_new_traveler_rid_obj:
            for op_id in range(self.n_op):
                self.operators[op_id].user_request(rq_obj, sim_time)
                self.demand.undecided_rq[rid] = rq_obj
                # unassigned_rqs[rid] = rq_obj
        list_undecided_travelers = list(self.demand.get_undecided_travelers(sim_time))
        for (rid, rq_obj) in list_undecided_travelers:
            # it is possible that the second decision takes longer than for the car to get to the pu location
            # in this case the decision is set to positive in the request class and is now deleted from undecided_rq
            if rq_obj.get_service_vehicle():
                del self.demand.undecided_rq[rid]
                continue
            for op_id in range(self.n_op):
                amod_offer = self.operators[op_id].get_current_offer(rid)
                LOG.debug(f"amod offer {amod_offer}")
                if amod_offer is not None:
                    rq_obj.receive_offer(op_id, amod_offer, sim_time, self.scenario_parameters)
            self._rid_chooses_offer(rid, rq_obj, sim_time)
        # 4)
        self._check_waiting_request_cancellations(sim_time)
        # 5)
        for op_id, op_obj in enumerate(self.operators):
            op_obj.time_trigger(sim_time)

        self.record_stats()

    def _check_waiting_request_cancellations(self, sim_time):
        """This method builds the interface for traveler models, where users can cancel their booking after selecting
        an operator.

        :param sim_time: current simulation time
        :return: None
        """
        waiting_rq = list(self.demand.waiting_rq.items())
        for rid, rq_obj in waiting_rq:
            chosen_operator = rq_obj.get_chosen_operator()
            in_vehicle = rq_obj.get_service_vehicle()
            if chosen_operator is not None:
                amod_offer = self.operators[chosen_operator].get_current_offer(rid)
                if amod_offer.additional_offer_parameters.get('locked') is not None:
                    rq_obj.receive_offer(chosen_operator, amod_offer, sim_time, self.scenario_parameters)
                    if in_vehicle is None and chosen_operator is not None and rq_obj.cancels_booking(sim_time):
                        self.operators[chosen_operator].user_cancels_request(rid, sim_time)
                        self.demand.record_user(rid)
                        del self.demand.rq_db[rid]
                        del self.demand.waiting_rq[rid]

    def add_evaluate(self):
        pass
    '''
