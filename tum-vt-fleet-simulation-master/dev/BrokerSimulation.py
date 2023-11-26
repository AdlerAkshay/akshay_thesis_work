# -------------------------------------------------------------------------------------------------------------------- #
# standard distribution imports
# -----------------------------
import logging
import numpy as np

# additional module imports (> requirements)
# ------------------------------------------

# src imports
# -----------
from src.FleetSimulationBase import FleetSimulationBase
from src.ImmediateDecisionsSimulation import ImmediateDecisionsSimulation

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


class ExchangeRequestsSimulation(FleetSimulationBase):
    """
    this fleetsimulation is used for the easyride AP2400 broker scenarios
    in this case exchanging of requests between two operators is simulated
    thereby the following simulation flow is simulated
    each request has a preferred operator
    if a customer enters the system it first requests the trip from the preferred operator
        the operator evalues if it is able and willing to served the customer (see EasyRideBrokerFleetControl.py)
        if willing and able -> the customer is served by this operator
        else: 
            the request is sent to the other operator
            if the other operator is willing and able, the customer is served by this operator
            if the other operator is not willing or not able:
                if the preferred operator was able but not willing: the preferred operator has to serve the customer
                if the preferred operator was also not able: the customer remains unserved and leaves the system
    """
    def __init__(self, scenario_parameters):
        super().__init__(scenario_parameters)

    def add_init(self, scenario_parameters):
        """
        Simulation specific additional init.
        :param scenario_parameters: row of pandas data-frame; entries are saved as x["key"]
        """
        # Assign a preferred operator to each traveler
        # (do not overwrite if request already has non-None preferred operator)
        if scenario_parameters[G_SLAVE_CPU] > 1:
            N_cores = scenario_parameters[G_SLAVE_CPU]
            dir_names = self.dir_names
            alonso_mora_timeout_per_veh_tree = self.list_op_dicts[0].get(G_RA_TB_TO_PER_VEH, None)
            from src.fleetctrl.pooling.batch.AlonsoMora.AlonsoMoraParallelization import ParallelizationManager
            Parallelisation_Manager = ParallelizationManager(N_cores, scenario_parameters, dir_names, alonso_mora_timeout_per_veh_tree)
            for op_id, op in enumerate(self.operators):
                try:
                    op.register_parallelization_manager(Parallelisation_Manager)
                except:
                    LOG.warning("couldnt register parallelization for op {}".format(op_id))
        super().add_init(scenario_parameters)
        if G_MULTIOP_PREF_OP_RSEED in scenario_parameters:
            np.random.seed(scenario_parameters[G_MULTIOP_PREF_OP_RSEED])
        p = scenario_parameters.get(G_MULTIOP_PREF_OP_PROB, [1/self.n_op for o in range(self.n_op)])
        LOG.debug(f"Assigning preferred operator to each request with operator shares {p}")
        for rid, req in self.demand:
            if getattr(req, "preferred_operator", None) is None:
                setattr(req, "preferred_operator", np.random.choice(self.n_op, p=p))

    @staticmethod
    def _is_nonempty_offer(offer):
        flag_count = 0
        if G_OFFER_WILLING_FLAG in offer:
            flag_count += 1
        if G_OFFER_PREFERRED_OP in offer:
            flag_count += 1
        return len(offer) > flag_count

    def step(self, sim_time):
        """This method determines the simulation flow in a time step.
            # 1) update fleets and network
            # 2) get new travelers, add to undecided request
            # 3) sequential processes for each undecided request: request -> offer -> user-decision
                # 3a) evaluate willingness of preffered operator
                # 3b) if pref op able and willing -> accept offer
                # 3c) if pref op not able or not willing
                #   3ci) request trip from other operator
                #   3cii) if other operator able and willing -> accept this offer
                #   3ciii) else 
                            if pref operator was able -> use pref operator
                            else -> leave system unserved
            # 4) periodically for waiting requests: run decision process -> possibly leave system (cancellation)
            # 5) periodically operator: call ride pooling optimization, repositioning, charging management

        :param sim_time: new simulation time
        :return: None
        """
        # 1)
        self.update_sim_state_fleets(sim_time - self.time_step, sim_time)
        new_travel_times = self.routing_engine.update_network(sim_time)
        if new_travel_times:
            for op_id in range(self.n_op):
                self.operators[op_id].inform_network_travel_time_update(sim_time)
        # 2)
        list_undecided_travelers = list(self.demand.get_undecided_travelers(sim_time))
        last_time = sim_time - self.time_step
        if last_time < self.start_time:
            last_time = None
        list_new_traveler_rid_obj = self.demand.get_new_travelers(sim_time, since=last_time)
        # 3)
        for rid, rq_obj in list_undecided_travelers + list_new_traveler_rid_obj:
            # 4a) send requests to preferred operators
            pref_op_id = rq_obj.preferred_operator
            op_received_offers = [] #list of operators that have been requested
            LOG.debug(f"Request {rid}: Checking AMoD option of preferred operator {pref_op_id} ...")
            self.operators[pref_op_id].user_request(rq_obj, sim_time)
            pref_amod_offer = self.operators[pref_op_id].get_current_offer(rid)
            op_received_offers.append(pref_op_id)
            LOG.debug(f"amod offer {pref_amod_offer}")
            rq_obj.receive_offer(pref_op_id, pref_amod_offer, sim_time)
            # pref op doesnt want to or cant
            if pref_amod_offer.service_declined() or not pref_amod_offer.get(G_OFFER_WILLING_FLAG, False):
                LOG.debug(" -> operator doesnt want to or cant")
                for i, operator in enumerate(self.operators):
                    if i == pref_op_id:
                        continue
                    LOG.debug(f"Request {rid}: Checking AMoD option of non-preferred operator {i} ...")
                    operator.user_request(rq_obj, sim_time)
                    amod_offer = self.operators[i].get_current_offer(rid)
                    op_received_offers.append(i)
                    LOG.debug(f"amod offer {amod_offer}")
                    rq_obj.receive_offer(i, amod_offer, sim_time)

            # rid chooses:
            # 1) pref op, if offer and willing
            # 2) else: other op if offer and willing
            # 3) pref op, if offer
            self._rid_chooses_offer(rid, rq_obj, sim_time)
        # 4)
        self._check_waiting_request_cancellations(sim_time)
        # 5)
        for op in self.operators:
            op.time_trigger(sim_time)
        # record at the end of each time step
        self.record_stats()

    def evaluate(self):
        """Runs standard and simulation environment specific evaluations over simulation results."""
        output_dir = self.dir_names[G_DIR_OUTPUT]
        # standard evaluation
        from src.evaluation.standard import standard_evaluation
        standard_evaluation(output_dir)


# ================================================================================================================== #

class BrokerBaseSimulation(ExchangeRequestsSimulation):
    """
    this fleetsimulation is also used for the evaluation of the exchange request scenarios but simulates independent operators
    each customer only requests trips from the preferred operator and is deleted unserved, if this operator is not able to serve the customer
    """

    def step(self, sim_time):
        """This method determines the simulation flow in a time step.
            # 1) update fleets and network
            # 2) get new travelers, add to undecided request
            # 3) sequential processes for each undecided request: request -> offer -> user-decision
            # 4) periodically for waiting requests: run decision process -> possibly leave system (cancellation)
            # 5) periodically operator: call ride pooling optimization, repositioning, charging management

        :param sim_time: new simulation time
        :return: None
        """
        # 1)
        self.update_sim_state_fleets(sim_time - self.time_step, sim_time)
        new_travel_times = self.routing_engine.update_network(sim_time)
        if new_travel_times:
            for op_id in range(self.n_op):
                self.operators[op_id].inform_network_travel_time_update(sim_time)
        # 2)
        list_undecided_travelers = list(self.demand.get_undecided_travelers(sim_time))
        last_time = sim_time - self.time_step
        if last_time < self.start_time:
            last_time = None
        list_new_traveler_rid_obj = self.demand.get_new_travelers(sim_time, since=last_time)
        # 3)
        for rid, rq_obj in list_undecided_travelers + list_new_traveler_rid_obj:
            pref_op_id = rq_obj.preferred_operator
            LOG.debug(f"Request {rid}: Checking AMoD option of preferred operator {pref_op_id} ...")
            self.operators[pref_op_id].user_request(rq_obj, sim_time)
            amod_offer = self.operators[pref_op_id].get_current_offer(rid)
            LOG.debug(f"amod offer {amod_offer}")
            if amod_offer is not None:
                rq_obj.receive_offer(pref_op_id, amod_offer, sim_time)
            self._rid_chooses_offer(rid, rq_obj, sim_time)
        # 4)
        self._check_waiting_request_cancellations(sim_time)
        # 5)
        for op in self.operators:
            op.time_trigger(sim_time)
        # record at the end of each time step
        self.record_stats()

