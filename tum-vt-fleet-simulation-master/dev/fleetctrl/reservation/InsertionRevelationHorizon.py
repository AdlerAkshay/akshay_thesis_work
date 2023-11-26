from __future__ import annotations
from typing import Dict

from src.fleetctrl.planning.VehiclePlan import VehiclePlan, RoutingTargetPlanStop

from src.misc.globals import *
from dev.fleetctrl.reservation.RevelationHorizonBase import RevelationHorizonBase
from src.fleetctrl.pooling.immediate.insertion import reservation_insertion_with_heuristics, simple_remove
from dev.fleetctrl.reservation.misc.RequestGroup import QuasiVehicle, QuasiVehiclePlan

import logging
LOG = logging.getLogger(__name__)

class InsertionRevelationHorizon(RevelationHorizonBase):
    """ this reservation class treats reservation requests with a naive rolling horizon approach:
    innitially reservation requests are assigned to vehicles by an insertion heuristic;
    this assignment is kept until the simulation time approches the earliest pickup time within the rolling horizon;
    then requests are revealed to the global optimisation and removed from the reservation class
    """
    def __init__(self, fleetctrl, operator_attributes, dir_names, solver="Gurobi"):
        super().__init__(fleetctrl, operator_attributes, dir_names, solver=solver)

    def _batch_optimisation(self, sim_time):
        """ in this module reservation solutions are created by inserting them one by one into the current solution via an insertion heuristic"""
        for i, rid in enumerate(list(self._unprocessed_rids.keys())):
            LOG.info("reservation batch process rid {}/{}".format(i, len(self._unprocessed_rids)))
            self.return_immediate_reservation_offer(rid, sim_time)  # does the job for now
        self._unprocessed_rids = {}
        
    def return_immediate_reservation_offer(self, rid, sim_time):
        try:
            del self._unprocessed_rids[rid]
        except:
            pass
        return super().return_immediate_reservation_offer(rid, sim_time)
    