# -------------------------------------------------------------------------------------------------------------------- #
# standard distribution imports
# -----------------------------
import logging
import os

# additional module imports (> requirements)
# ------------------------------------------
from abc import ABC

# from IPython import embed

# src imports
# -----------
from src.FleetSimulationBase import FleetSimulationBase

# -------------------------------------------------------------------------------------------------------------------- #
# global variables
# ----------------
from src.misc.globals import *
LOG = logging.getLogger(__name__)


# TODO # CAN THIS ONE BE DELETED?
# -------------------------------------------------------------------------------------------------------------------- #
# functions
# ---------


# -------------------------------------------------------------------------------------------------------------------- #
# main
# ----
class TestMultiStepRideHailingSimulation(FleetSimulationBase, ABC):
    """
    Init main simulation module. Check the documentation for a flowchart of this particular simulation environment.
    Main attributes:
    - agent list per time step query public transport and fleet operator for offers and immediate decide
    - fleet operator offers ride pooling service
    - division of study area
        + first/last mile service in different parts of the study area
        + different parking costs/toll/subsidy in different parts of the study area
    """

    def __init__(self, scenario_parameters):
        self.lock_time = scenario_parameters[G_RA_LOCK_TIME]
        super().__init__(scenario_parameters)
        '''
        for op in self.operators:
            try:
                op.set_time_window_constraint_hardness(scenario_parameters[G_RA_TW_HARD])
            except KeyError:
                LOG.debug(f"Unable to set time window hardness constraints for operator {op}.")
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
                            rq_obj.receive_offer(op_id, offer, sim_time)
                            new_offers[rid] = rq_obj
                        # ... possible update of offer (not necessary here)
                LOG.debug(f"sim time {sim_time}")
                LOG.debug(f"new offers : {new_offers}")
                LOG.debug(f"unassigned rqs : {unassigned_rqs.keys()}")
                if len(new_offers) > 0:
                    for rid, rq_obj in new_offers.items():
                        # 5e) mode choice and consequences
                        LOG.debug(f"Request {rid}: calling mode choice ...")
                        chosen_operator = rq_obj.choose_offer(self.scenario_parameters)
                        LOG.debug(f"Request {rid}: choice: {chosen_operator} ...")
                        for op_id, op_obj in enumerate(self.operators):
                            # this will create an error for multiple operators in case of a assignment retry ( TODO )
                            if chosen_operator == op_id:
                                op_obj.user_confirms_booking(rid, sim_time)
                            else:
                                op_obj.user_cancels_request(rid, sim_time)
                        if chosen_operator is None or chosen_operator < 0:  # TODO # op_id, if no operator is chosen?
                            self.demand.record_user(rid)
                            del self.demand.rq_db[rid]
                        del unassigned_rqs[rid]
                self.record_stats(force=False)
            # save final state, record remaining travelers and vehicle tasks
            self.save_final_state()
            self.record_remaining_assignments()
        # call evaluation
        self.evaluate()

    def update_sim_state_fleets(self, last_time, next_time, force=False):
        """
        This method updates the simulation vehicles, records, ends and starts tasks and returns some data that
        will be used for additional state updates (fleet control information, demand, network, ...)
        :param last_time: simulation time before the state update
        :param next_time: simulation time of the state update
        :param force: flag that can force vehicle plan to be updated
        :type force: bool
        """
        LOG.debug(f"updating MoD state from {last_time} to {next_time}")
        for opid_vid_tuple, veh_obj in self.sim_vehicles.items():
            op_id, vid = opid_vid_tuple
            boarding_requests, alighting_requests, passed_vrl, dict_start_alighting =\
                veh_obj.update_veh_state(last_time, next_time)
            for rid, alighting_start_time_and_pos in dict_start_alighting.items():
                # record user stats at beginning of alighting process
                alighting_start_time = alighting_start_time_and_pos[0]
                LOG.debug(f"rid {rid} deboarding at {alighting_start_time}")
                self.demand.record_alighting_start(rid, vid, op_id, alighting_start_time)
                # TODO # travel time is always exactly too short, seems to be a minor issue about if a leg is ended
                #  before or after a simulation step
            for rid, alighting_end_time in alighting_requests.items():
                # # record user stats at end of alighting process
                self.demand.user_ends_alighting(rid, vid, op_id, alighting_end_time)
                self.operators[op_id].acknowledge_alighting(rid, vid, alighting_end_time)
            for rid, boarding_time_and_pos in boarding_requests.items():
                boarding_time = boarding_time_and_pos[0]
                LOG.debug(f"rid {rid} boarding at {boarding_time}")
                self.demand.record_boarding(rid, vid, op_id, boarding_time)
                self.operators[op_id].acknowledge_boarding(rid, vid, boarding_time)
            # send update to operator
            self.operators[op_id].receive_status_update(vid, next_time, passed_vrl, force)

    def add_evaluate(self):
        import pandas as pd
        all_times_dict = {}
        for i in self.__dict__:
            if 'time_' in i and 'step' not in i:
                all_times_dict[i] = self.__dict__[i]
        all_vid_df = pd.DataFrame.from_dict(all_times_dict, orient="index")
        all_vid_df.to_csv(os.path.join(self.dir_names[G_DIR_OUTPUT], "computational_time_evaluation.csv"))
        return
