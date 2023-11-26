import logging
from src.misc.globals import G_DIR_OUTPUT

import time
import numpy as np

import src.fleetctrl.pooling.GeneralPoolingFunctions as GeneralPoolingFunctions
from src.fleetctrl.pooling.batch.BatchAssignmentAlgorithmBase import BatchAssignmentAlgorithmBase, SimulationVehicleStruct
from dev.fleetctrl.pooling.batch.ParallelTempering.HeatBath import HeatBath, markovDecision
from src.fleetctrl.planning.VehiclePlan import VehiclePlan, BoardingPlanStop
from src.fleetctrl.pooling.immediate.insertion import simple_remove
from src.misc.globals import *

LOG = logging.getLogger(__name__)
LARGE_INT = 100000
MAX_T = 1000000

def markovDecisionTwoTemps(cfv1, cfv2, t1, t2, minimize = True):
    """ this function returns if a solution transition is made based on a
    probabilistic markov process
        p = min(exp( (cfv1 - cfv2)(1/t1 - 1/t2) ), 1)
    :param old_cfv: cost_function_value of old solution
    :param new_cfv: cost function value of new solution
    :param Temperature: temperature value
    :param minimize: if False, a higher cfv is prefered -> process turned around 
    :return: True, if new_sol is accepted, False else
    """
    LOG.debug(f"markovDecisionTwoTemps cfv1 {cfv1} cfv2 {cfv2} t1 {t1} t2 {t2}")
    delta = cfv1 - cfv2
    if not minimize:
        delta = -delta
    LOG.debug(f"delta {delta}")
    if t1 == 0.0:
        if delta > 0:
            return True
        else:
            return False
    elif t2 == 0.0:
        if delta < 0:
            return True
        else:
            return False
    elif t1 == t2:
        return False
    else:
        r = np.random.random()
        beta = (1.0/t1 - 1.0/t2)
        LOG.debug(f"beta {beta} | delta beta {delta*beta}")
        if delta*beta > 0:
            return True
        else:
            p = np.math.exp(delta*beta)
            LOG.debug(f"r {r} p {p}")
            if r < p:
                return True
            else:
                return False

def tempDistribution(number_heat_baths, min_t, max_t):
    """ this function creates the initial temperature distribution for a set of heatbaths for parallel tempering
    a polynomial distance is chosen in this distribution
    # TODO # other distributions?
    :param number_heat_baths: Number of heatbaths used in the parallel tempering algorithm
    :param min_t: minimum temperature value
    :param max_t: maximum temperature value
    :return: sorted list from low to high of number_heat_baths temperature values from [min_t, max_t] """
    Ts = []
    exponent = 8
    if number_heat_baths == 1:
        return [min_t]
    for i in range(number_heat_baths):
        T = (float(i)/(number_heat_baths - 1))**exponent *(max_t-min_t) + min_t
        Ts.append(T)
    return Ts

def get_minimum_feasible_vehicle_plan(veh_obj, assigned_veh_plan, sim_time, routing_engine):
    """ this function returns the minimal number of vrls that are needed to run a feasible simulation for a single vehicle
    -> only ob_rids are kept and ps that are locked or unfeasible_locked 
    :param veh_obj: correspondig vehicle object
    :param assigned_veh_plan: currently assigned vehicle plan
    :param sim_tim: current simulation time
    :param routing_engine: network routing engine object
    :return: tuple of (minimum feasible veh_plan object, dict {rid -> 1} for requests belonging to this minimum plan) """
    LOG.debug("get minimum feasible veh plan {}".format(sim_time))
    LOG.debug(f"{veh_obj}\n{assigned_veh_plan}")
    minimum_vrls_to_assign = []
    rids_to_assign = {rq.get_rid_struct() : 1 for rq in veh_obj.pax}    #ob rids have to be included
    LOG.debug(f"rids to assign: {rids_to_assign}")
    to_keep_inidces = []
    for i, ps in enumerate(assigned_veh_plan.list_plan_stops):
        to_keep = False
        if ps.is_locked() or ps.is_infeasible_locked():   #locked plan stops have to be included
            to_keep = True
            for rid in ps.get_list_boarding_rids() + ps.get_list_alighting_rids():  #rids at locked plans stops have to be included too
                rids_to_assign[rid] = 1
        else:
            for rid in ps.get_list_boarding_rids() + ps.get_list_alighting_rids():  #if an included rid is part of plan stop, this plan stop has to be included, too
                if rids_to_assign.get(rid) is not None:
                    to_keep = True
                    break
        if to_keep:
            to_keep_inidces.append(i)
    LOG.debug(f"to keep indices {to_keep_inidces}")
    new_list_plan_stops = []
    for i in to_keep_inidces:   # keep those plan stops, but remove rids not in rids_to_assign
        old_ps = assigned_veh_plan.list_plan_stops[i]
        new_boarding_dict = {}
        new_max_trip_time_dict = {}
        new_earliest_pickup_time_dict = {}
        new_latest_pickup_time_dict = {}
        ept, lpt, mtt, lat = ps.get_boarding_time_constraint_dicts()
        for rid in old_ps.get_list_boarding_rids():
            if rids_to_assign.get(rid):
                try:
                    new_boarding_dict[1].append(rid)
                except:
                    new_boarding_dict[1] = [rid]
                new_earliest_pickup_time_dict[rid] = ept[rid]
                new_latest_pickup_time_dict[rid] = lpt[rid]
        for rid in old_ps.get_list_alighting_rids():
            if rids_to_assign.get(rid):
                try:
                    new_boarding_dict[-1].append(rid)
                except:
                    new_boarding_dict[-1] = [rid]
                new_max_trip_time_dict[rid] = mtt[rid]
        if len(new_boarding_dict.keys()) > 0:
            new_ps = BoardingPlanStop(ps.pos, boarding_dict=new_boarding_dict, max_trip_time_dict=new_max_trip_time_dict,
                                        earliest_pickup_time_dict=new_earliest_pickup_time_dict, latest_pickup_time_dict=new_latest_pickup_time_dict,
                                        change_nr_pax=old_ps.get_change_nr_pax(), duration=ps.get_duration_and_earliest_departure()[0], locked=old_ps.is_locked())            
            new_list_plan_stops.append(new_ps)
    minimum_veh_plan = VehiclePlan(veh_obj, sim_time, routing_engine, new_list_plan_stops)
    LOG.debug("minimum plan {}".format(minimum_veh_plan))
    LOG.debug("rids to assign {}".format(rids_to_assign))
    return minimum_veh_plan, rids_to_assign
    

class ParallelTemperingAssignment(BatchAssignmentAlgorithmBase):
    def __init__(self, fleetcontrol, routing_engine, sim_time, obj_function, operator_attributes, optimisation_cores=1, seed = 6061992, veh_objs_to_build={}):
        """ 
        this class computes new ride-pooling assignments with the meta heuristic parallel tempering
                this meta heuristic uses a set of coupled heatbaths searching through the solution space at different temperatures
                while exchanging their current solution based on markov-probabilities
        the detailed description of the use of the methods within a fleetcontrol-class can be found in BatchAssignmentAlgorithmDesign.py
        # TODO # to finalize/parametrize
        :param fleetcontrol: reference to fleetcontrol
        :param routing_engine: reference to routing_engine
        :param sim_time: current simulation time
        :param std_bt: fleet const boarding time parameter
        :param add_bt: fleet additional boarding time parameter
        :param obj_funtion: objective function to rate vehicle plans
        :param N_heatbaths: number of heatbaths used for parallel tempering
        :param max_Temperature: highest temperature of the heatbath distribution (T = 0 always included)
        :param seed: random seed
        :param Parallelization_handler: TODO not implemented!
        """

        super().__init__(fleetcontrol, routing_engine, sim_time, obj_function, operator_attributes,
            optimisation_cores=optimisation_cores, seed=seed, veh_objs_to_build=veh_objs_to_build)

        self.number_heatbaths = operator_attributes.get(G_RA_PT_N_HB, 8)
        self.heatbath_iterations = operator_attributes.get(G_RA_PT_HB_ITS, 20)
        self.break_condition = operator_attributes.get(G_RA_PT_BREAK, 0.1)
        self.std_bt = operator_attributes[G_OP_CONST_BT]
        self.add_bt = operator_attributes.get(G_OP_ADD_BT,0)
        max_Temperature = MAX_T # TODO

        self.heatbath_temperatures = tempDistribution(self.number_heatbaths, 0.0, max_Temperature)
        self.minimal_temperature = min(self.heatbath_temperatures)
        self.heatbaths = []
        if optimisation_cores == 1:
            self.heatbaths = [
                HeatBath(self.fleetcontrol, self.routing_engine, sim_time, obj_function, operator_attributes, optimisation_cores=1, seed=seed,
                    veh_objs_to_build=veh_objs_to_build, Temperature=self.heatbath_temperatures[i], hbid=i)
                    for i in range(self.number_heatbaths)]
        self.parallelization_manager = None

        self.current_best_sol = {}
        self.current_best_cfv = None

    def register_parallelization_manager(self, parallelization_manager):
        self.parallelization_manager = parallelization_manager
        if parallelization_manager is not None:
            self.heatbaths = []
            self.parallelization_manager.initOp(self.fo_id, self.objective_function, self.operator_attributes, self.heatbath_temperatures)

    def add_new_request(self, rid, prq, consider_for_global_optimisation = True, is_allready_assigned = False):
        """ this function adds a new request to the modules database and set entries that
        possible v2rbs are going to be computed in the next opt step.
        :param rid: plan_request_id
        :param prq: plan_request_obj 
        :param consider_for_global_optimisation: if false, it will not be looked for better solutions in global optimisation
                    but it is still part of the solution, if it is allready assigned
        :param is_allready_assigned: if not considered for global optimisation, this flag indicates, if the rid is allready assigned
            in the init solution"""
        super().add_new_request(rid, prq, consider_for_global_optimisation=consider_for_global_optimisation, is_allready_assigned=is_allready_assigned)
        if self.parallelization_manager is not None:
            self.parallelization_manager.add_new_request(self.fo_id, rid, prq, consider_for_global_optimisation=consider_for_global_optimisation, is_allready_assigned=is_allready_assigned)
        else:
            for hb in self.heatbaths:
                hb.add_new_request(rid, prq, consider_for_global_optimisation=consider_for_global_optimisation, is_allready_assigned=is_allready_assigned)

    def set_request_assigned(self, rid):
        """ the request is set as assigned in the optimisation constraints
        -> a solution not containing this request is infeasible
        :param rid: request_id
        """
        super().set_request_assigned(rid)
        if self.parallelization_manager is not None:
            self.parallelization_manager.set_request_assigned(self.fo_id, rid)
        else:
            for hb in self.heatbaths:
                hb.set_request_assigned(rid)

    def set_database_in_case_of_boarding(self, rid, vid):
        """ this function updates the database if a boarding occurs
        not needed in this algorithm, therefore nothing happens """
        super().set_database_in_case_of_boarding(rid, vid)
        if self.parallelization_manager is not None:
            self.parallelization_manager.set_database_in_case_of_boarding(self.fo_id, rid, vid)
        else:
            for hb in self.heatbaths:
                hb.set_database_in_case_of_boarding(rid, vid)

    def set_database_in_case_of_alighting(self, rid, vid):
        """ this function updates the database if an alighting process occurs
        -> deletes this request from the algorithm database
        :param rid: request id that alights the vehicle
        :param vid: vehicle id the request alights from
        """
        super().set_database_in_case_of_alighting(rid, vid)
        if self.parallelization_manager is not None:
            self.parallelization_manager.set_database_in_case_of_alighting(self.fo_id, rid, vid)
        else:
            for hb in self.heatbaths:
                hb.set_database_in_case_of_alighting(rid, vid)

    def clear_databases(self):
        """ this function is used to clear the database
        not needed in this algorithm, therefore nothing happens"""
        pass

    def delete_request(self, rid):
        """ this function deletes the request from the algorithms database and solution
        :param rid: request id 
        """
        if self.parallelization_manager is not None:
            self.parallelization_manager.delete_request(self.fo_id, rid)
        else:
            for hb in self.heatbaths:
                hb.delete_request(rid)
        super().delete_request(rid)

    def set_mutually_exclusive_assignment_constraint(self, list_sub_rids, base_rid):
        if self.parallelization_manager is not None:
            self.parallelization_manager.set_mutually_exclusive_assignment_constraint(self.fo_id, list_sub_rids, base_rid)
        else:
            for hb in self.heatbaths:
                hb.set_mutually_exclusive_assignment_constraint(list_sub_rids, base_rid)
        return super().set_mutually_exclusive_assignment_constraint(list_sub_rids, base_rid)

    def get_vehicle_plan_without_rid(self, veh_obj, vehicle_plan, rid_to_remove, sim_time):
        """this function returns the best vehicle plan by removing the rid_to_remove from the vehicle plan
        :param veh_obj: corresponding vehicle obj
        :param vehicle_plan: vehicle_plan where rid_remove is included
        :param rid_to_remove: request_id that should be removed from the rtv_key
        :param sim_time: current simulation time
        :return: best_veh_plan if vehicle_plan rid_to_remove is part of vehicle_plan, None else
        """
        new_plan = simple_remove(veh_obj, vehicle_plan, rid_to_remove, sim_time, self.routing_engine, self.objective_function, self.active_requests, self.std_bt, self.add_bt)
        return new_plan

    def get_current_assignment(self, vid):
        """ returns the vehicle plan assigned to vid currently
        :param vid: vehicle id
        :return: vehicle plan
        """
        return self.current_best_sol.get(vid)

    def lock_request_to_vehicle(self, rid, vid):
        """locks the request to the assigned vehicle"""
        # LOG.info("locking rid to vid {} {}".format(rid, vid))
        super().lock_request_to_vehicle(rid, vid)
        if self.parallelization_manager is not None:
            self.parallelization_manager.lock_request_to_vehicle(self.fo_id, rid, vid)
        else:
            for hb in self.heatbaths:
                hb.lock_request_to_vehicle(rid, vid)

    def compute_new_vehicle_assignments(self, sim_time, vid_to_list_passed_VRLs, veh_objs_to_build = {}, new_travel_times = False, build_from_scratch = False):
        """ this method computes new assignments with the PT algorithm
        the solution can be retrieved with the self.get_optimisation_solution method later
        :param sim_time: current simulation time
        :param vid_to_list_passed_VRLs: dict vid -> list_passed_VRLs since last optimisation call to update solution
        """
        if self.parallelization_manager is None:
            #import matplotlib.pyplot as plt #TODO can be deleted later (but maybe retrieve cfv-values otherwise)
            plotvals = {hb.T : [] for hb in self.heatbaths}

            self.sim_time = sim_time
            LOG.debug("update current solution")
            self._update_current_solutions(sim_time, vid_to_list_passed_VRLs)   #update and create new init solutions
            LOG.debug("update done!")
            new_best_sol = None
            new_best_cfv = float("inf")
            old_best_cfv = float("inf")
            t = time.time()
            for vid, plan in self.current_best_sol.items():
                LOG.debug(f"{vid} -> {plan}")
            i = 0
            while True:
                for hb in self.heatbaths:
                    x = hb.temperature_movements(self.heatbath_iterations) #independed heatbath search
                    if i != 0:
                        plotvals[hb.T].extend(x)
                    hb_best_sol, hb_best_cfv = hb.get_best_sol_found()
                    if hb_best_sol is not None and hb_best_cfv < new_best_cfv:
                        new_best_cfv = hb_best_cfv
                        new_best_sol = hb_best_sol
                if i > 8 and abs(old_best_cfv - new_best_cfv) <= self.break_condition:
                    LOG.info("no change in solution qualitity -> break after: {}".format(time.time() -t))
                    break
                LOG.info("PT switch {} | current global opt: {}".format(i, new_best_cfv))
                t_to_id_cfv = {hb.get_temperature() : (hb.hb_id, hb.get_current_cfv()) for hb in self.heatbaths}
                self._parallel_tempering_switch(t_to_id_cfv)   # heatbath coupling and solution exchange
                old_best_cfv = new_best_cfv
                i += 1

            self.current_best_cfv = new_best_cfv    #set new sol
            self.current_best_sol = new_best_sol.copy()
            LOG.debug("PT OPT RESULTS with cfv {}".format(self.current_best_cfv))
            n_ass = 0
            for vid, plan in self.current_best_sol.items():
                LOG.debug(f"vid {vid} -> {plan}")
                n_ass += len(plan.pax_info)
            LOG.debug("PT OPT RESULTS with cfv {}".format(self.current_best_cfv))

            # TODO del later
            # plt.figure(figsize=(7,7))
            # for T, vals in sorted(plotvals.items(), key = lambda x:x[0]):
            #     plt.plot([i for i in range(len(vals))], vals, label = f"{T}")
            # plt.xlabel("Number Iterations")
            # plt.ylabel("CFV")
            # plt.title(f"{self.current_best_cfv} | N_rq {len(self.mutually_exclusive_cluster_id_to_rids.keys())} | Ass {n_ass}")
            # plt.legend()
            # if False: #new_best_cfv != 0:
            #     plt.show()
            # else:
            #     import os
            #     plt.savefig(os.path.join(self.fleetcontrol.dir_names[G_DIR_OUTPUT], "pt_{}.png".format(sim_time)))
            #     plt.close()
        else:
            self.sim_time = sim_time
            LOG.debug("update current solution")
            self._update_current_solutions(sim_time, vid_to_list_passed_VRLs)   #update and create new init solutions
            LOG.debug("update done!")
            new_best_sol = None
            new_best_cfv = float("inf")
            old_best_cfv = float("inf")
            tm = time.time()

            i = 0
            while True:
                #hb_id_to_sol[hb.hb_id] = (hb.get_temperature(), hb_best_sol, hb_best_cfv, hb_cur_sol, hb_cur_cfv)
                hb_id_to_sol = self.parallelization_manager._temperature_movement(self.fo_id, self.heatbath_iterations)
                t_to_id_cfv = {}
                for hb_id, sol_tuple in hb_id_to_sol.items():
                    t, hb_best_sol, hb_best_cfv, _, cur_cfv = sol_tuple
                    t_to_id_cfv[t] = (hb_id, cur_cfv)
                    if hb_best_sol is not None and hb_best_cfv < new_best_cfv:
                        new_best_cfv = hb_best_cfv
                        new_best_sol = hb_best_sol
                if i > 8 and abs(old_best_cfv - new_best_cfv) <= self.break_condition:
                    LOG.info("no change in solution qualitity -> break after: {}".format(time.time() -t))
                    break
                LOG.debug("PT switch {} | current global opt: {}".format(i, new_best_cfv))
                # parallel tempering switch
                self._parallel_tempering_switch(t_to_id_cfv)
                old_best_cfv = new_best_cfv
                i += 1
            self.current_best_sol = new_best_sol
            self.current_best_cfv = new_best_cfv

    def get_optimisation_solution(self, vid):
        """ returns the computed asignment solution for the specific vehicle
        :param vid: vehicle id
        :return: assigned vehicle plan for this vehicle
        """
        return self.current_best_sol.get(vid, VehiclePlan(self.veh_objs[vid], self.sim_time, self.routing_engine, []))

    def set_assignment(self, vid, assigned_plan, is_external_vehicle_plan = False):
        """ sets the vehicleplan as assigned in the algorithm database; if the plan is not computed within the this algorithm, the is_external_vehicle_plan flag should be set to true
        :param vid: vehicle id
        :param assigned_plan: vehicle plan object that has been assigned
        :param is_external_vehicle_plan: should be set to True, if the assigned_plan has not been computed within this algorithm
        """
        if is_external_vehicle_plan:
            self.current_best_sol[vid] = assigned_plan

    def _update_current_solutions(self, sim_time, vid_to_list_passed_VRLs):
        """ this internal function updates the current solution and provides information to create
        a new initial solution in the heatbaths
        :param sim_time: current simulation time
        :param vid_to_list_passed_VRLs: dict vid -> list_passed_VRLs since last optimisation call to update solution
        """
        LOG.debug("PT: update current solution")

        # just take the current solution for all inits
        new_veh_objs = {vid : SimulationVehicleStruct(veh, self.fleetcontrol.veh_plans[vid], sim_time, self.routing_engine) for vid, veh in enumerate(self.fleetcontrol.sim_vehicles)}
        new_veh_plans = {vid : self.fleetcontrol.veh_plans[vid].copy_and_remove_empty_planstops(new_veh_objs[vid], sim_time, self.routing_engine) for vid in new_veh_objs.keys()}
        if self.parallelization_manager is not None:
            self.parallelization_manager._setNewSol(self.fo_id, new_veh_plans, new_veh_objs, sim_time)
            for rid, prq in self.active_requests.items():
                consider_for_opt = True
                if not self.rid_to_consider_for_global_optimisation.get(rid):
                    consider_for_opt = False
                assigned = True
                if self.unassigned_requests.get(self._get_associated_baserid(rid)):
                    assigned = False
                self.parallelization_manager.add_new_request(self.fo_id, rid, prq, consider_for_global_optimisation = consider_for_opt, is_allready_assigned = assigned)
        else:
            for hb in self.heatbaths:
                hb.set_new_sol(new_veh_plans, new_veh_objs, sim_time)
        LOG.debug("CURRENT BEST CFV: {}".format(self.current_best_cfv))
        # PT move to exchange solutions (to sort by qualitity of init solution)
        #self._parallel_tempering_switch()

    def setAdditionalInitForParallelization(self, current_assignments, v2r_ob, requests_to_compute, rr, v2r, active_requests):
        """ sets addtional init parameters for parallelization
        nothing happens here currently
        TODO: check if needed here to
        """
        pass

    def _parallel_tempering_switch2(self, t_to_id_cfv):
        """ this function describes the coupling and solution exchange between heatbaths
        based on their temperature and their current cost function value
        :param t_to_id_cfv: dict hb temperature -> hb_id, current cfv
        """
        switch_temps = {}   # switch hb_id -> new_t
        t_sorted_hbs = list(sorted(t_to_id_cfv.keys()))
        for x, t1 in enumerate(t_sorted_hbs):
            if x == len(t_sorted_hbs) - 1:
                break
            hbid1, cfv1 = t_to_id_cfv[t1]
            for y in range(x+1, len(t_sorted_hbs)):
                t2 = t_sorted_hbs[y]
                hbid2, cfv2 = t_to_id_cfv[t2]
                if switch_temps.get(hbid1) is None and switch_temps.get(hbid2) is None:
                    if t1 < t2:
                        LOG.debug("switch states: T1 {} with cfv {} <-> T2 {} with cfv {}".format(t1, cfv1, t2, cfv2))
                        if markovDecision(cfv1, cfv2, t1):
                            LOG.debug(" -> yep")
                            switch_temps[hbid1] = t2
                            switch_temps[hbid2] = t1
                            break     
        if not self.parallelization_manager:
            for hb in self.heatbaths:
                if switch_temps.get(hb.hb_id) is not None:
                    hb.set_temperature(switch_temps[hb.hb_id])
        else:
            self.parallelization_manager._switchTemp(self.fo_id, switch_temps)     

    def _parallel_tempering_switch(self, t_to_id_cfv):
        """ this function describes the coupling and solution exchange between heatbaths
        based on their temperature and their current cost function value
        :param t_to_id_cfv: dict hb temperature -> hb_id, current cfv
        """
        switch_temps = {}   # switch hb_id -> new_t
        checked = {}    # (hb_id1 , hb_id2) -> if checked
        t_shuffeld_hbs = list((t_to_id_cfv.keys()))
        #np.random.shuffle(t_shuffeld_hbs)
        t_shuffeld_hbs.sort()

        for x, t1 in enumerate(t_shuffeld_hbs):
            hbid1, cfv1 = t_to_id_cfv[t1]
            if switch_temps.get(hbid1) is not None:
                continue
            for y, t2 in enumerate(t_shuffeld_hbs):
                hbid2, cfv2 = t_to_id_cfv[t2]
                if hbid1 == hbid2:
                    continue
                if switch_temps.get(hbid2) is not None:
                    continue
                if hbid1 < hbid2:
                    s = (hbid1, hbid2)
                else:
                    s = (hbid2, hbid1)
                if checked.get(s) is not None:
                    continue
                checked[s] = 1
                LOG.debug("switch states: T1 {} with cfv {} <-> T2 {} with cfv {}".format(t1, cfv1, t2, cfv2))
                if markovDecisionTwoTemps(cfv1, cfv2, t1, t2):
                    LOG.debug(" -> yep")
                    switch_temps[hbid1] = t2
                    switch_temps[hbid2] = t1
                    break     
        if not self.parallelization_manager:
            for hb in self.heatbaths:
                if switch_temps.get(hb.hb_id) is not None:
                    hb.set_temperature(switch_temps[hb.hb_id])
        else:
            self.parallelization_manager._switchTemp(self.fo_id, switch_temps)  
