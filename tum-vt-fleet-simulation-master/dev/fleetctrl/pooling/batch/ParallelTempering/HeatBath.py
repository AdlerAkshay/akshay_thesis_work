from src.fleetctrl.pooling.GeneralPoolingFunctions import get_assigned_rids_from_vehplan
from src.fleetctrl.pooling.batch.BatchAssignmentAlgorithmBase import BatchAssignmentAlgorithmBase
import numpy as np

from src.fleetctrl.pooling.immediate.insertion import simple_insert, simple_remove
from src.fleetctrl.planning.VehiclePlan import VehiclePlan
from src.misc.globals import *

import logging 
LOG = logging.getLogger(__name__)


def markovDecision(old_cfv, new_cfv, Temperature, minimize = True):
    """ this function returns if a solution transition is made based on a
    probabilistic markov process
        p(True) = 1 if new_cfv < old_cfv
                = exp(- (new_cfv - old_cfv)/T) else
        p(False) = 1 - p(True)
    :param old_cfv: cost_function_value of old solution
    :param new_cfv: cost function value of new solution
    :param Temperature: temperature value
    :param minimize: if False, a higher cfv is prefered -> process turned around 
    :return: True, if new_sol is accepted, False else
    """
    delta = new_cfv - old_cfv
    if not minimize:
        delta = -delta
    if delta < 0:
        return True
    elif Temperature == 0.0:
        return False
    else:
        r = np.random.random()
        try:
            p = np.math.exp(-delta/Temperature) #.exp(-delta/deltaT)
        except:
            if -delta/Temperature > 0:
                p = float("inf")
            else:
                print("exp error")
                print(delta, Temperature, new_cfv, old_cfv)
                exit()
        if r < p:
            return True
        else:
            return False

def hardcopy_sol(sol, veh_dict):
    """ this function returns a deep copy of the input solution
    :param sol: dict vid -> vehplan 
    :veh_dict: dict vid->veh_obj
    :return: dict vid -> vehplan (but copied)
    """
    new_sol = {}
    for vid, veh_obj in veh_dict.items():
        vehplan = sol.get(vid)
        if vehplan is None:
            vehplan = VehiclePlan(veh_obj, None, None, [])
        else:
            vehplan = vehplan.copy()
        new_sol[vid] = vehplan
    return new_sol

class HeatBath(BatchAssignmentAlgorithmBase):
    def __init__(self, fleetcontrol, routing_engine, sim_time, obj_function, operator_attributes, optimisation_cores=1, seed = 6061992, veh_objs_to_build={},
            Temperature=None, hbid = 0):
        """ this class simulates a heatbath at a given temperature
                a solution ride-pooling assignment is changed randomly be locally destruction and repair operators
                the new solution is accepted based on a markov process defined by the temperature
        :param fleetcontrol: fleetcontrol reference
        :param routing_engine: routing_engine reference
        :param sim_time: current simulation time
        :param obj_function: function to rate a specific vehicle plan (see: ?)
        :param operator_attributes: dictionary of operator attributes from sc config
        :param optimisation_cores: number cores (ignored in this class)
        :param seed: random seed
        :param veh_objs_to_build: vid -> SimVehicleStruct
        :param Temperature: temperature of the heatbath (necessary!)
        :param hbid: id of heatbath (necessary when more than one)
        """

        super().__init__(fleetcontrol, routing_engine, sim_time, obj_function, operator_attributes, optimisation_cores=optimisation_cores, seed=seed, veh_objs_to_build=veh_objs_to_build)
        
        self.T = Temperature
        self.hb_id = hbid

        self.rid_to_assigned_vid = {}   # rid -> vid of current sol
        self.vid_to_assigned_rids = {}  # vid -> rid -> 1 of current sol
        
        self.current_sol = {}
        self.cfv = None
        self.current_best_sol = {}
        self.current_best_cfv = self.cfv
        self.unassigned_rids = {}   # unassigned rids in current sol
        self.removable_rids = {}    # rid -> 1; which is can be removed from sol (all except locked ones)
        if self.fleetcontrol is not None:
            for vid in self.veh_objs.keys():
                assigned_plan = self.fleetcontrol.veh_plans[vid]
                self.current_sol[vid] = assigned_plan.copy()
                rids = get_assigned_rids_from_vehplan(assigned_plan)
                for rid in rids:
                    self._add_rid_vid_assignment_to_db(rid, vid)
            self.cfv = self._compute_global_cfv()

    def __str__(self):
        return "HeatBath with T {}".format(self.T)

    def str_current_sol(self):
        str_list = [f"current sol of {self} with cfv {self.cfv}:"]
        for vid in self.veh_objs.keys():
            str_list.append(f"vid {vid} -> {self.current_sol.get(vid)}")
        str_list.append("rid to assigned vid {}".format(self.rid_to_assigned_vid))
        str_list.append("vid to assigned rids: {}".format(self.vid_to_assigned_rids))
        str_list.append("unassigned_rids {}".format(self.unassigned_rids))
        str_list.append("removable rids {}".format(self.removable_rids))
        return "\n".join(str_list)

    def get_current_cfv(self):
        return self.cfv

    def get_best_cfv(self):
        return self.current_best_cfv

    def get_temperature(self):
        return self.T

    def set_temperature(self, T):
        self.T = T

    def get_best_sol_found(self):
        """ returns the best solution found with the corresponding costfunction value
        :return: tuple of (best solution dict vid -> veh_plan, costfuntion value)
        """
        return self.current_best_sol, self.current_best_cfv

    def get_current_sol_found(self):
        """ returns the current solution found with the corresponding costfunction value
        :return: tuple of (current solution dict vid -> veh_plan, costfuntion value)
        """
        return self.current_sol, self.current_best_cfv

    def set_new_sol(self, new_sol, new_veh_dict, sim_time):
        """ sets a new solution in the heatbath
        :param new_sol: new_solution to be set (dict vid -> veh_plan)
        :param new_veh_dict: vid -> SimulationVehicleStruct of current vehicle state
        :param sim_time: current simulation time
        """
        self.sim_time = sim_time
        self.veh_objs = new_veh_dict

        self.current_sol = hardcopy_sol(new_sol, self.veh_objs)
        self.cfv = None

        self.current_best_sol = hardcopy_sol(new_sol, self.veh_objs)
        self.current_best_cfv = None

        new_global_cfv = self._compute_global_cfv()
        self.cfv = new_global_cfv
        self.current_best_cfv = new_global_cfv

        self.unassigned_rids = {self._get_associated_baserid(rid) : 1 for rid in self.active_requests.keys()}
        self.removable_rids = {}
        for vid, veh_plan in self.current_sol.items():
            rids = get_assigned_rids_from_vehplan(veh_plan)
            for rid in rids:
                base_rid = self._get_associated_baserid(rid)
                try:
                    del self.unassigned_rids[base_rid]
                except KeyError:
                    pass
                if self.r2v_locked.get(base_rid) is None:
                    self.removable_rids[base_rid] = 1
                self._add_rid_vid_assignment_to_db(base_rid, vid)
        LOG.verbose("init sol {}".format(self.str_current_sol()))


    def temperature_movements(self, N_iterations):
        """ computes N_iterations of local solution changes which are adopted based on markov processes with the corresponding Temperature
        :param N_iterations: number of local solution changes to be computed
        """
        # LOG.debug(f"temperature movements of {self}")
        # LOG.debug("state before:\n{}".format(self.str_current_sol()))
        N_d_r_prob = [0.6, 0.9, 1.0]    # TODO # paramtrization!
        cfv_vals = [self.cfv]   # TODO delete!
        for i in range(N_iterations):
            ndr = 2
            r = np.random.rand()
            for i, p in enumerate(N_d_r_prob):
                if p <= r:
                    ndr = i + 1
                    break 

            LOG.verbose(f"random destruction of {ndr}")
            # TODO # implement more ways to change solution!
            new_sol_changes, new_cfv, removed_rids = self._random_destruction(N_remove=ndr)  # remove ndr requests randomly from solution
            if len(self.unassigned_rids) > 0:   # assign more then to delete
                ndr += 1
            LOG.verbose(f"random sequential repair of {ndr}")
            new_sol_changes, new_cfv, new_assigned_rids = self._random_sequential_repair(N_repair = ndr, cfv = new_cfv, sol_changes = new_sol_changes, removed_rids = removed_rids)  # try to reinsert ndr requests into the solution

            # check feasibility of new sol (assignment constraint)
            is_feasible = True
            for rid in removed_rids.keys():
                base_rid = self._get_associated_baserid(rid)
                if self.unassigned_requests.get(base_rid) is not None:
                    is_feasible = False
                    break
            if is_feasible and markovDecision(self.cfv, new_cfv, self.T):   # accept sol based on markov process
                LOG.debug(f"HB {self.T} accepts new sol {self.cfv} -> {new_cfv}")
                self._accept_new_sol(new_sol_changes, new_cfv, removed_rids, new_assigned_rids)
            cfv_vals.append(self.cfv)
        LOG.verbose("state after:\n{}".format(self.str_current_sol()))
        return cfv_vals # can be deleted; only for debug? TODO

    def add_new_request(self, rid, prq, consider_for_global_optimisation = True, is_allready_assigned = False):
        """ adds a new request to the database and marks it as unassigned
        :param rid: request id
        :param prq: planrequest object
        """
        super().add_new_request(rid, prq, consider_for_global_optimisation=consider_for_global_optimisation, is_allready_assigned=is_allready_assigned)

    def set_request_assigned(self, rid):
        """ sets the assigned constraint for the request rid (solution is treated infeasible if this request is not included)
        :param rid: request id
        """
        super().set_request_assigned(rid)

    def delete_request(self, rid):
        """ deletes request from heatbath and its solution
        :param rid: request id
        """
        # LOG.debug("del request hb")
        # LOG.debug(f"{self.mutually_exclusive_cluster_id_to_rids}")
        # LOG.debug(f"{self.rid_to_mutually_exclusive_cluster_id}")
        # if self.rid_to_assigned_vid.get(rid, None) is not None:
        #     assigned_vid = self.rid_to_assigned_vid[rid]
        #     assigned_plan = self.current_sol[assigned_vid]
        #     new_plan = None
        #     if assigned_plan is not None:
        #         rids_in_plan = assigned_plan.get_dedicated_rid_list()
        #         for sub_rid in self._get_all_rids_representing_this_base_rid(rid):
        #             LOG.debug("del request: sub rid {} in plan {}".format(sub_rid, rids_in_plan))
        #             if sub_rid in rids_in_plan:
        #                 new_plan = simple_remove(self.veh_objs[assigned_vid], assigned_plan, sub_rid, self.sim_time, self.routing_engine, self.objective_function, self.active_requests, self.std_bt, self.add_bt)
        #                 break
        #     self.current_sol[assigned_vid] = new_plan
        #     self._remove_rid_vid_assignment_from_db(rid, assigned_vid)
        super().delete_request(rid)

    def clear_databases(self):
        return super().clear_databases()

    def compute_new_vehicle_assignments(self, sim_time, vid_to_list_passed_VRLs, veh_objs_to_build = {}, new_travel_times = False, build_from_scratch = False):
        raise NotImplementedError("This class doesnt compute new vehicle assignments!")

    def get_optimisation_solution(self, vid):
        return self.current_best_sol[vid]

    def lock_request_to_vehicle(self, rid, vid):
        return super().lock_request_to_vehicle(rid, vid)

    def set_database_in_case_of_alighting(self, rid, vid):
        return super().set_database_in_case_of_alighting(rid, vid)

    def set_database_in_case_of_boarding(self, rid, vid):
        return super().set_database_in_case_of_boarding(rid, vid)

    def set_assignment(self, vid, assigned_plan, is_external_vehicle_plan):
        return super().set_assignment(vid, assigned_plan, is_external_vehicle_plan=is_external_vehicle_plan)

    def get_current_assignment(self, vid):
        return self.current_best_sol[vid]

    def _compute_global_cfv(self):
        """ compute the global cost function value of the current solution (sum of costfunction values for each assigned plan)
        :return: costfunction value
        """
        global_cfv = 0
        for vid, assigned_vehplan in self.current_sol.items():
            if assigned_vehplan is not None:
                global_cfv += self._computePlanCostfunctionValue(assigned_vehplan, self.veh_objs[vid])
        return global_cfv

    def _add_rid_vid_assignment_to_db(self, rid, vid):
        """ adds the new assgnment to the dictionaries vid_to_assigned_rids and rid_to_assigned_vid
        !The dictionaries unassigned_rids and removeable rids are not treated here! (TODO?)
        :param rid: request id
        :param vid: assigned vehicle id
        """
        #LOG.debug(f"add rid vid assignment {rid} -> {vid}")
        assigned_vid = self.rid_to_assigned_vid.get(rid)
        if assigned_vid is not None and assigned_vid != vid:
            #LOG.warning("trying to add assignment without removing before! {} <-> {}".format(vid, assigned_vid))
            try:
                del self.vid_to_assigned_rids[assigned_vid][rid]
            except KeyError:
                pass 
        self.rid_to_assigned_vid[rid] = vid
        try:
            self.vid_to_assigned_rids[vid][rid] = 1
        except KeyError:
            self.vid_to_assigned_rids[vid] = {rid : 1}

    def _remove_rid_vid_assignment_from_db(self, rid, vid):
        """ removes the new assgnment to the dictionaries vid_to_assigned_rids and rid_to_assigned_vid
        !The dictionaries unassigned_rids and removeable rids are not treated here! (TODO?)
        :param rid: request id
        :param vid: previously assigned vehicle id
        """
        #LOG.debug(f"rm rid vid assignment {rid} -> {vid}")
        assigned_vid = self.rid_to_assigned_vid[rid]
        if assigned_vid != vid:
            #LOG.warning("trying to remove assignment which is not there! {} <-> {}".format(vid, assigned_vid))
            try:
                del self.vid_to_assigned_rids[assigned_vid][rid]
            except KeyError:
                pass            
        del self.rid_to_assigned_vid[rid]
        try:
            del self.vid_to_assigned_rids[vid][rid]
        except KeyError:
            pass
        try:
            del self.removable_rids[rid]
        except KeyError:
            pass

    def _return_random_rid_vid_insertion(self, rid, vid, vid_vehplan_to_consider = None):
        """ returns the first randomized possible insertion of a request into a vehicle assigned plan
        :param rid: request id
        :param vid: vehicle id
        :param vid_vehplan_to_consider: optinal; uses this vehicleplan for insertion if given, the insertionplan of the current solution otherwise
        :return: tuple (best_plan, best_delta); best_plan: best vehplan found for insertion (None if none found), best_delta: change in cost function value compared to cfv of previously assigned plan
        """
        veh_obj = self.veh_objs[vid]
        if vid_vehplan_to_consider is None:
            assigned_plan = self.current_sol.get(vid, VehiclePlan(veh_obj, self.sim_time, self.routing_engine, []))
        else:
            assigned_plan = vid_vehplan_to_consider
        prev_plan_cfv = self._computePlanCostfunctionValue(assigned_plan, veh_obj)
        best_plan = None
        best_delta = 0
        for inserted_plan in simple_insert(self.routing_engine, self.sim_time, veh_obj, assigned_plan, self.active_requests[rid], self.std_bt, self.add_bt, random_order=True):
            new_plan_cfv = self._computePlanCostfunctionValue(inserted_plan, veh_obj)
            #LOG.debug(f" -> new cfv {new_plan_cfv}")
            new_delta = new_plan_cfv - prev_plan_cfv
            if new_delta < best_delta:
                best_plan = inserted_plan
                best_delta = new_delta
                break
        #LOG.debug(f" -> BEST: {best_delta}")
        return best_plan, best_delta

    def _return_random_rid_insertion(self, rid, not_accepted_sol_changes_to_consider = {}):
        """ returns the first randomized possible insertion of a request into the current solution
        :param rid: request id
        :param not_accepted_sol_changes_to_consider: dict vid -> vehicleplan optinal; uses vehicleplan of this dictionary for insertion if given, the insertionplan of the current solution otherwise
        :return: tuple (best_plan, best_delta, best_vid); best_plan: best vehplan found for insertion (None if none found), best_delta: change in cost function value compared to cfv of previously assigned plan, best_vid: vehicle id of corresponding plan
        """
        best_plan = None
        best_delta = 0
        best_vid = None
        rand_vids = list(self.veh_objs.keys())
        np.random.shuffle(rand_vids)
        for vid in rand_vids:
            sub_rids = list(self._get_all_rids_representing_this_base_rid(rid))
            np.random.shuffle(sub_rids)
            for sub_rid in sub_rids:
                vid_best_plan, vid_best_delta = self._return_best_rid_vid_insertion(sub_rid, vid, vid_vehplan_to_consider=not_accepted_sol_changes_to_consider.get(vid, None))
                if vid_best_plan is not None and vid_best_delta < best_delta:
                    best_plan = vid_best_plan
                    best_delta = vid_best_delta
                    best_vid = vid
                    return best_plan, best_delta, best_vid
        return best_plan, best_delta, best_vid

    def _return_best_rid_vid_insertion(self, rid, vid, vid_vehplan_to_consider = None):
        """ returns the best possible insertion of a request into a vehicle assigned plan
        :param rid: request id
        :param vid: vehicle id
        :param vid_vehplan_to_consider: optinal; uses this vehicleplan for insertion if given, the insertionplan of the current solution otherwise
        :return: tuple (best_plan, best_delta); best_plan: best vehplan found for insertion (None if none found), best_delta: change in cost function value compared to cfv of previously assigned plan
        """
        veh_obj = self.veh_objs[vid]
        if vid_vehplan_to_consider is None:
            assigned_plan = self.current_sol.get(vid, VehiclePlan(veh_obj, self.sim_time, self.routing_engine, []))
        else:
            assigned_plan = vid_vehplan_to_consider
        prev_plan_cfv = self._computePlanCostfunctionValue(assigned_plan, veh_obj)
        best_plan = None
        best_delta = 0
        for inserted_plan in simple_insert(self.routing_engine, self.sim_time, veh_obj, assigned_plan, self.active_requests[rid], self.std_bt, self.add_bt):
            new_plan_cfv = self._computePlanCostfunctionValue(inserted_plan, veh_obj)
            #LOG.debug(f" -> new cfv {new_plan_cfv}")
            new_delta = new_plan_cfv - prev_plan_cfv
            if new_delta < best_delta:
                best_plan = inserted_plan
                best_delta = new_delta
        #LOG.debug(f" -> BEST: {best_delta}")
        return best_plan, best_delta

    def _return_best_rid_insertion(self, rid, not_accepted_sol_changes_to_consider = {}):
        """ returns the best possible insertion of a request into the current solution
        :param rid: request id
        :param not_accepted_sol_changes_to_consider: dict vid -> vehicleplan optinal; uses vehicleplan of this dictionary for insertion if given, the insertionplan of the current solution otherwise
        :return: tuple (best_plan, best_delta, best_vid); best_plan: best vehplan found for insertion (None if none found), best_delta: change in cost function value compared to cfv of previously assigned plan, best_vid: vehicle id of corresponding plan
        """
        best_plan = None
        best_delta = 0
        best_vid = None
        for sub_rid in self._get_all_rids_representing_this_base_rid(rid):
            LOG.verbose("insert {}".format(sub_rid))
            for vid in self.veh_objs.keys():
                vid_best_plan, vid_best_delta = self._return_best_rid_vid_insertion(sub_rid, vid, vid_vehplan_to_consider=not_accepted_sol_changes_to_consider.get(vid, None))
                if vid_best_plan is not None and vid_best_delta < best_delta:
                    best_plan = vid_best_plan
                    best_delta = vid_best_delta
                    best_vid = vid
        return best_plan, best_delta, best_vid

    def _accept_new_sol(self, new_sol_changes, new_cfv, removed_rids, new_assigned_rids):
        """ this function adopts the current solution based on local solution changes that have been made
        :param new_sol_changes: dict of changes compared to the current solution | vid -> new_veh_plan
        :param new_cfv: new global costfunction value when new_sol_changes are included in the current solution
        :param removed_rids: dict rid -> 1, for all rids that have benn remove from the current solution compared to new_sol_changes
        :param new_assigned_rids: dict rid -> 1, for all rids that have been reinserted into the current solution compared to new_sol_changes (and removed rids)
        """
        # LOG.debug("accept new sol:")
        # LOG.debug("current sol:")
        # for vid, plan in self.current_sol.items():
        #     LOG.debug(f"vid {vid} -> {plan}")
        # LOG.debug("updates:")
        # for vid, plan in new_sol_changes.items():
        #     LOG.debug(f"vid {vid} -> {plan}")
        self.current_sol.update(new_sol_changes)
        self.cfv = new_cfv
        for rid in removed_rids.keys():
            self._remove_rid_vid_assignment_from_db(rid, self.rid_to_assigned_vid[rid]) # update rid to vid assignment dict
            self.unassigned_rids[rid] = 1   # removed rid is now unassigned in the current sol
            try:
                del self.removable_rids[rid]    # rid is now allready removed
            except KeyError:
                pass
        for rid, assigned_vid in new_assigned_rids.items():
            self._add_rid_vid_assignment_to_db(rid, assigned_vid)
            try:
                del self.unassigned_rids[rid]
            except KeyError:
                pass
            self.removable_rids[rid] = 1    # an onboard request should never be in the new_assigned_rids

        if new_cfv < self.current_best_cfv: # update current best sol found if needed
            self.current_best_cfv = new_cfv
            self.current_best_sol = hardcopy_sol(self.current_sol, self.veh_objs)

    def _computePlanCostfunctionValue(self, veh_plan, veh_obj):
        """ computes the cost function value of a single vehicle plan
        :param veh_plan: corresponding vehicle plan
        :param veh_obj: corresponding simulation vehicle (or struct) object
        :return: costfunction value
        """
        cfv = self.objective_function(self.sim_time, veh_obj, veh_plan, self.active_requests, self.routing_engine)
        veh_plan.set_utility(cfv)
        return cfv

    def _random_destruction(self, N_remove = 1):
        """ randomly removes N_remove rids from the solutions (from the set of self.removeable_rids.keys())
                if the set is smaller than N_remove the function breaks if empty and less rids are removed
        :param N_remove: number of rids to remove from solution
        :return: tuple of (sol_changes, new_cfv, removed_rids) sol_changes: dict vid -> vehplan that changed, new_cfv : new global cost function value, removed_rids: dict rid -> 1 of rids that have been removed from solution
        """
        new_cfv = self.cfv
        removed_rids = {}
        new_sol_changes = {}
        for i in range(N_remove):
            if len(self.removable_rids.keys()) == 0:
                break
            remove_rid = np.random.choice(list(self.removable_rids.keys())) # choose randomly
            if removed_rids.get(remove_rid) is not None:    # test if allready removed
                if len(removed_rids) >= len(self.removable_rids.keys()):    # no more rids can be removed
                    break
                else:   # retry
                    i -= 1
                    continue
            assigned_vid = self.rid_to_assigned_vid[remove_rid] # get current assigned vid and vehplan in sol
            veh_obj = self.veh_objs[assigned_vid]
            if new_sol_changes.get(assigned_vid, None) is not None: #check if vehplan has been changed in former remove step
                current_plan = new_sol_changes[assigned_vid]
            else:
                current_plan = self.current_sol[assigned_vid]
            current_sub_rids = current_plan.get_involved_request_ids()
            LOG.verbose(f"random destruction: {remove_rid} from {assigned_vid}")
            LOG.verbose(f"prev: {current_plan}")
            for sub_rid in self._get_all_rids_representing_this_base_rid(remove_rid):
                if sub_rid in current_sub_rids:
                    new_plan = simple_remove(veh_obj, current_plan, sub_rid, self.sim_time, self.routing_engine, self.objective_function, self.active_requests, self.std_bt, self.add_bt) #remove rid from vehplan
                    break
            LOG.verbose(f"after: {new_plan}")
            new_cfv += self._computePlanCostfunctionValue(new_plan, veh_obj) - self._computePlanCostfunctionValue(current_plan, veh_obj)    # compute new global cfv based on local changes
            new_sol_changes[assigned_vid] = new_plan
            removed_rids[remove_rid] = 1
        return new_sol_changes, new_cfv, removed_rids

    def _random_sequential_repair(self, N_repair = 1, cfv = None, sol_changes = {}, removed_rids = {}):
        """ tries to insert randomly one rid from self.unassigned_rids (doesnt retry if this is not possible (TODO?))
        :param N_repair: number of rids that are tried to be inserted into the current solution
        :param cfv: needs to be given if the function is called upon a changed solution (e.g. by _random_destruction before). costfunction value of current solution, takes self.cfv if cfv == None
        :param sol_changes: needs to be given if the function is called upon a changed solution. sol changes dict vid -> vehplan compared to self.current_sol
        :param removed_rids: removed rids within the sol changes. dict rid -> 1 if rid has been removed in a former destruction step
        :return: tuple of (sol_changes, new_cfv, new_assigned_rids) sol_changes: dict vid -> vehplan that changed, new_cfv : new global cost function value, new_assigned_rids: dict rid -> 1 of rids that have been reinsorted into the solution
        """
        if cfv is None:
            new_cfv = self.cfv
        else:
            new_cfv = cfv
        new_assigned_rids = {}
        assignable_rids = list(self.unassigned_rids.keys()) + list(removed_rids.keys())
        for i in range(N_repair):
            if len(assignable_rids) == 0:
                break
            repair_rid = np.random.choice(assignable_rids)
            #LOG.debug(f"repair rid {repair_rid}")
            repair_function = np.random.randint(2)
            if repair_function == 0:
                best_plan, best_delta, best_vid = self._return_best_rid_insertion(repair_rid, not_accepted_sol_changes_to_consider = sol_changes)
            else:
                best_plan, best_delta, best_vid = self._return_random_rid_insertion(repair_rid, not_accepted_sol_changes_to_consider = sol_changes)
            if best_plan is not None:
                new_assigned_rids[repair_rid] = best_vid
                new_cfv += best_delta
                sol_changes[best_vid] = best_plan
                #LOG.debug(f"sol found: {best_plan}")
            assignable_rids.remove(repair_rid)
        return sol_changes, new_cfv, new_assigned_rids
