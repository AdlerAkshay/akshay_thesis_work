import numpy as np
import time
from src.fleetctrl.FleetControlBase import FleetControlBase
from dev.fleetctrl.reservation.BatchSchedulingRevelationHorizonBase import BatchSchedulingRevelationHorizonBase
from dev.fleetctrl.reservation.misc.RequestGroup import QuasiVehiclePlan
from dev.fleetctrl.reservation.ReservationRequestBatch import ReservationRequestBatch
from src.misc.globals import *

from typing import Any, Dict, List, Tuple

import logging
LOG = logging.getLogger(__name__)

NEW_VID_PENALTY = 1000000   # penalty for introducing a new vehicle in case a match between batches is not possible

def batch_requests(sorted_requests, number_requests_in_batch):
    """ this function is used to created request batches with fixed batch sizes according to their
    sorting order
    :param sorted_requests: sorted list of plan request objects
    :param number_requests_in_batch: int number of number objects per batch
    :return: list of list of request object batches. the length of the inner list does not exceed number_requests_in_batch"""
    batch_list = []
    current_batch = []
    for i, rq in enumerate(sorted_requests):
        if i != 0 and i % number_requests_in_batch == 0:
            batch_list.append(current_batch)
            current_batch = []
        current_batch.append(rq)
    if len(current_batch) != 0:
        batch_list.append(current_batch)
    return batch_list
        
class ForwardBatchOptimization(BatchSchedulingRevelationHorizonBase):
    """ this algorithm batches reservation requests in batch of sizes specified by the input parameter "op_res_batch_size"
    within the batches all possible requests groups are calculated (V2RB without a vehicle)
    in the assignement step, these batches are connected one after another starting with the first one
    the connection is made by solving a maximum priority matching problem to schedule request groups within batches
    the batches (requests) are in the current version sorted by their earliest pick-up time
    this method is currently only stable if all reservation requests are known beforehand (no update of solution possible)"""
    def __init__(self, fleetctrl : FleetControlBase, operator_attributes : dict, dir_names : dict, solver : str="Gurobi"):
        super().__init__(fleetctrl, operator_attributes, dir_names, solver=solver)
        
    def _create_sorted_request_batches(self, sim_time: int) -> List[ReservationRequestBatch]:
        LOG.debug("reservation fwd batch optimization!")
        request_batches = batch_requests(sorted([prq for rid, prq in self.active_reservation_requests.items() if self._reavealed_rids.get(rid) is None], key=lambda x:x.get_o_stop_info()[1]), self.max_batch_size)
        LOG.debug("{} batches with max size {} and {} requests created!".format(len(request_batches), self.max_batch_size, len(self.active_reservation_requests)))
        batch_rg_list = []
        last_routing_time = sim_time
        for i, rq_list in enumerate(request_batches):
            LOG.debug(" -> create request batch {}/{}".format(i, len(request_batches)))
            if self._use_own_routing_engine:
                batch_start_time = min([rq.get_o_stop_info()[1] for rq in rq_list])
                if batch_start_time is not None:
                    for t in range(last_routing_time, batch_start_time):
                        self.routing_engine.update_network(t)
                    last_routing_time = batch_start_time
            rg = ReservationRequestBatch()
            for rq in rq_list:
                rg.full_insertion_request_to_batch(rq, self.routing_engine, self.fleetctrl, self.vehicle_capacity)
            batch_rg_list.append(rg)
            
        if self._use_own_routing_engine:
            self.routing_engine.reset_network(sim_time)
            
        return batch_rg_list