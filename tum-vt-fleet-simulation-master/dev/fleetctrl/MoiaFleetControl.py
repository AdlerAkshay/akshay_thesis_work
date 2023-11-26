import logging
import time
import os

from src.simulation.Offers import TravellerOffer
from src.fleetctrl.planning.PlanRequest import PlanRequest
from src.fleetctrl.planning.VehiclePlan import VehiclePlan
from src.fleetctrl.RidePoolingBatchOptimizationFleetControlBase import RidePoolingBatchOptimizationFleetControlBase
from dev.fleetctrl.AlonsoMoraFleetcontrolBase import AlonsoMoraFleetControlBase
from src.fleetctrl.pooling.batch.AlonsoMora.MoiaAlonsoMoraParallelization import MoiaParallelizationManager
from src.fleetctrl.pooling.GeneralPoolingFunctions import get_assigned_rids_from_vehplan
from src.fleetctrl.pooling.immediate.insertion import single_insertion, insertion_with_heuristics
from src.fleetctrl.pooling.immediate.searchVehicles import veh_search_for_immediate_request, veh_search_for_reservation_request
from src.fleetctrl.pooling.batch.BatchAssignmentAlgorithmBase import SimulationVehicleStruct
from dev.fleetctrl.charging.thresholds import exchange_low_soc_vehicles_at_depot
from src.infra.BoardingPointInfrastructure import BoardingPointInfrastructure

from src.misc.globals import *

LOG = logging.getLogger(__name__)
LARGE_INT = 100000


def get_hamburg_taxi_fare(simulation_time, direct_distance, number_passengers):
    """ hamburg taxi fare from https://www.hamburg.de/taxi/2936756/taxi-fahrpreise/
    in moia project the fare may not exceed taxi fare
    """
    is_work_day = False
    if simulation_time < 432000:
        is_work_day = True
    day_seconds = simulation_time%86400
    peak_time = False
    if ((day_seconds < 36000) and (day_seconds > 25200)) or ((day_seconds <68400) and (day_seconds>57600)):
        peak_time = True
    if not is_work_day or not peak_time:
        fare = 350
        if number_passengers >= 4:
            fare += 600
        if direct_distance < 4000:
            return fare + direct_distance * 0.245
        else:
            fare += 980
            rest_distance = direct_distance - 4000
            if rest_distance < 5000:
                return fare + rest_distance * 0.220
            else:
                fare += 1100
                rest_distance -= 5000
                return fare + rest_distance * 0.150
    else:
        fare = 420
        if number_passengers >= 4:
            fare += 600
        if direct_distance < 4000:
            return fare + direct_distance * 0.250
        else:
            fare += 1000
            rest_distance = direct_distance - 4000
            if rest_distance < 5000:
                return fare + rest_distance * 0.230
            else:
                fare += 1150
                rest_distance -= 5000
                return fare + rest_distance * 0.160

class MOIAfleetcontrol(AlonsoMoraFleetControlBase):
    def __init__(self, op_id, operator_attributes, list_vehicles, routing_engine, zone_system, scenario_parameters,
                 dir_names, charging_management=None):
        """Fleet control class for MOIA framework
        request enter system continously
            offer has to be created immediatly by an insertion heuristic
            request replies immediatly
            -> there can never be 2 requests at the same time waiting for an offer!
        reoptimisation of solution after certain time interval

        :param op_id: operator id
        :type op_id: int
        :param operator_attributes: dictionary with keys from globals and respective values
        :type operator_attributes: dict
        :param list_vehicles: simulation vehicles; their assigned plans should be instances of the VehicleRouteLeg class
        :type list_vehicles: list
        :param routing_engine: routing engine
        :type routing_engine: Network
        :param scenario_parameters: access to all scenario parameters (if necessary)
        :type scenario_parameters: dict
        :param dirnames: directories for output and input
        :type dirnames: dict
        """
        super().__init__(op_id, operator_attributes, list_vehicles, routing_engine, zone_system, scenario_parameters,
                         dir_names, charging_management=charging_management)
        sim_start_time = scenario_parameters[G_SIM_START_TIME]

        # TODO! but currently not feasible with subrids operator_attributes.get(G_OP_MAX_WT_2, None)
        self.max_wait_time_2 = None
        self.offer_pickup_time_interval = operator_attributes.get(G_OP_OFF_TW, None)

        try:
            # maximum walking distance to consider for subrids [m]
            self.max_walking_distance = scenario_parameters[G_BP_MAX_DIS]
        except:
            LOG.error("{} not given for VariableBoardingPointsFleetcontrol!".format(G_BP_MAX_DIS))
        try:
            self.walking_speed = scenario_parameters[G_WALKING_SPEED]  # average walkign speed (m/s)
        except:
            LOG.error("{} not given for VariableBoardingPointsFleetcontrol!".format(G_WALKING_SPEED))
        try:
            # maximum other boarding locations to consider at start and end for each requests (the original included)
            self.max_other_boarding_stops = scenario_parameters.get(G_BP_MAX_BPS, None)
            if self.max_other_boarding_stops < 1:
                raise EnvironmentError("at least one boarding point should be considered!")
            if self.max_other_boarding_stops is None:
                LOG.warning("max other boarding stops paramter not given!")
        except:
            LOG.error("{} not given for VariableBoardingPointsFleetcontrol!".format(G_BP_MAX_BPS))

        self.boardingpoints = None
        if dir_names.get(G_DIR_INFRA, None) is not None:
            if os.path.isfile(os.path.join(dir_names[G_DIR_INFRA], "boarding_points.csv")):
                self.boardingpoints = BoardingPointInfrastructure(dir_names[G_DIR_INFRA], self.routing_engine)
        if self.boardingpoints is None:
            raise EnvironmentError(f"could find input specifaction for boarding point infrastructure! "
                                   f"{G_DIR_BP} not given in scenario file!")

        self.lock_after_first_assignment = operator_attributes.get(G_RA_LOCK_RID_VID, False)

        self.unassigned_requests_1 = {}
        self.unassigned_requests_2 = {}

        self.base_request_id_to_sub_rids = {}  # base_rid -> sub_rid -> 1
        self.sub_rid_to_base_request_id = {}  # sub_rid -> base_rid

        # base_rid -> (vid, veh_plan, virtual request id) veh_plan is current solution for offer which will be destroyed
        # or accepted once a request decides
        # (multiple base rids possible but refer to same phyical customer (intermodal!))
        self.current_vid_plan_virrid_tuple_dict = {}
        # base_rid -> 1 für new requests, that accepted offer but havent experienced the global optimisation yet
        self.new_accepted_base_rids = {}
        # vehicle plans without reloc ps dynamic adoption
        self.last_opt_time = -100000
        self.veh_plans_without_reloc = {} # vid -> vehplan dynamically updated during user requests

        self.fare_per_pax = {1: 1.0, 2: 1.75, 3: 2.5, 4: 3.25, 5: 3.75, 6: 4.25}
        # dynamic pricing legacy
        self.elastic_pricing = None
        self.util_pricing = None
        if operator_attributes.get(G_OP_UTIL_SURGE) and operator_attributes.get(G_OP_ELA_PRICE):
            raise NotImplementedError(f"{G_OP_ELA_PRICE} and {G_OP_UTIL_SURGE} cannot"
                                      f" be given together for the same operator!")
        elif operator_attributes.get(G_OP_UTIL_SURGE):
            util_file = os.path.join(self.dir_names[G_DIR_FCTRL], "elastic_pricing", operator_attributes[G_OP_UTIL_SURGE])
            from dev.fleetctrl.pricing.UtilizationPricing import UtilizationSurgePricing
            evaluation_interval = operator_attributes.get(G_OP_UTIL_EVAL_INT, 0)
            self.util_pricing = UtilizationSurgePricing(util_file, evaluation_interval)
            self._init_dynamic_fleetcontrol_output_key("utilization_all")
            self._init_dynamic_fleetcontrol_output_key("effective utilized vehicles")
            self._init_dynamic_fleetcontrol_output_key("active vehicles")
            self._init_dynamic_fleetcontrol_output_key("base fare factor")
            self._init_dynamic_fleetcontrol_output_key("distance fare factor")
            self._init_dynamic_fleetcontrol_output_key("general factor")
        elif operator_attributes.get(G_OP_ELA_PRICE):
            price_file = os.path.join(self.dir_names[G_DIR_FCTRL], "elastic_pricing",
                                      operator_attributes[G_OP_ELA_PRICE])
            from dev.fleetctrl.pricing.ElasticPricing import ElasticPrizing
            self.elastic_pricing = ElasticPrizing(price_file)

        # dynamic fleet sizing legacy
        self.target_utilization = operator_attributes.get(G_OP_DYFS_TARGET_UTIL, None)
        self.target_utilization_interval = None
        self.minimum_active_fleetsize = None
        self.max_duration_underutilized = None
        self.start_time_underutilization = sim_start_time
        if self.target_utilization is not None:
            self.target_utilization_interval = operator_attributes[G_OP_DYFS_TARGET_UTIL_INT]
            self.minimum_active_fleetsize = operator_attributes.get(G_OP_DYFS_MIN_ACT_FS, 0)
            self.max_duration_underutilized = operator_attributes.get(G_OP_DYFS_UNDER_UTIL_DUR, 0)
            self.start_time_underutilization -= (self.max_duration_underutilized + 1)
            self._init_dynamic_fleetcontrol_output_key("utilization")
            self._init_dynamic_fleetcontrol_output_key("effective utilized vehicles")
            self._init_dynamic_fleetcontrol_output_key("active vehicles")
            self._init_dynamic_fleetcontrol_output_key("activate vehicles")

        #comp times offer
        self.current_immediate_requests = 0
        self.current_reservation_requests = 0
        self.cur_offer_times = 0
        self.t_offer_start = -1

        #comp times since last offer phase
        self.t_offer_phase_start = time.time()

        self._init_dynamic_fleetcontrol_output_key("bp_opt")
        self._init_dynamic_fleetcontrol_output_key("N_res_rqs")
        self._init_dynamic_fleetcontrol_output_key("N_im_rqs")
        self._init_dynamic_fleetcontrol_output_key("t_off")
        self._init_dynamic_fleetcontrol_output_key("t_off_phase")

        self.pending_insertions = {}    # rid -> sub_rid -> (best_vid, best_plan, best_obj) | will be used in get_user_offer()
        self.insertion_fetch_needed = {}    # rid -> 1 if fetch from parallel processes needed before creating offer
        self.allready_computed_vir_prqs = {}    # rid -> list_vir_prqs if a pre-offer is called

    def add_init(self, operator_attributes, scenario_parameters):
        active_vehicle_file_name = operator_attributes.get(G_OP_ACT_FLEET_SIZE)
        if active_vehicle_file_name is None:
            active_vehicle_file = None
        else:
            active_vehicle_file = os.path.join(self.dir_names[G_DIR_FCTRL], "elastic_fleet_size",
                                               active_vehicle_file_name)
            if not os.path.isfile(active_vehicle_file):
                raise IOError(f"Could not find active vehicle file {active_vehicle_file}")
        if self.charging_management is not None and active_vehicle_file is not None:
            self.charging_management.read_active_vehicle_file(self, active_vehicle_file, scenario_parameters)
            LOG.info("active vehicle file loaded!")
            
        n_cores = scenario_parameters[G_SLAVE_CPU]
        LOG.info("add init: {}".format(n_cores))
        if n_cores > 1 and self.Parallelization_Manager is None:
            LOG.info("initialize MOIA Parallelization Manager")
            self.Parallelization_Manager = MoiaParallelizationManager(n_cores, scenario_parameters, self.dir_names,
                                                                  self.alonso_mora_timeout_per_veh_tree)
            self.Parallelization_Manager.additionalInitOp(self.op_id, operator_attributes)
        if self.Parallelization_Manager is not None:
            self.AM_Module.register_parallelization_manager(self.Parallelization_Manager)

    def fast_intermediate_user_offer(self, rq, sim_time):
        """ this method is used by the mobitopp fleet sim to filter intermodal request based on a simple heuristic.
        offer parameters are approximated by average waiting and travel time.
        these parameters are used to only request offers from a subset of all intermodel offers
        :param rq: traveler object
        :param sim_time: simulation time
        :return: expected pickup time, expected drop off time
        """
        rid_struct = rq.get_rid_struct()
        variable_boarding_point_prqs = self._get_variable_boarding_point_requests(
            rq)  # find feasible boarding locatations an created virtual rids
        self.allready_computed_vir_prqs[rid_struct] = variable_boarding_point_prqs
        earliest_drop_off_time = None
        corresponding_pickup_time = None
        for vir_prq in variable_boarding_point_prqs:
            _, pu_earliest, pu_latest = vir_prq.get_o_stop_info()
            _, _, mtt = vir_prq.get_d_stop_info()
            ept = (pu_latest + pu_earliest)/2.0
            edt = ept + (mtt + vir_prq.init_direct_tt)/2.0
            if earliest_drop_off_time is None or edt < earliest_drop_off_time:
                earliest_drop_off_time = edt
                corresponding_pickup_time = ept
        return corresponding_pickup_time, earliest_drop_off_time

    def user_request(self, rq, sim_time, calculate_parallel = False):
        """This method is triggered for a new incoming request. It generally adds the rq to the database. It has to
        return an offer to the user. An empty dictionary means no offer is made!

        :param rq: request object containing all request information
        :type rq: RequestDesign
        :param sim_time: current simulation time
        :type sim_time: float
        :param calculate_parallel: if this flag is set, it is tried to run the insertion on parallel process
        :type calculate_parallel: bool
        :return: offer
        :rtype: dict
        """
        self.sim_time = sim_time
        if len(self.pending_insertions) == 0 and len(self.insertion_fetch_needed) == 0:
            self.t_offer_start = time.time()
        # update dynamic vehplan dict if new time step started
        self._update_vehplans_without_reloc(sim_time)

        rid_struct = rq.get_rid_struct()
        variable_boarding_point_prqs = self.allready_computed_vir_prqs.get(rid_struct)
        if variable_boarding_point_prqs is None:
            variable_boarding_point_prqs = self._get_variable_boarding_point_requests(
                rq)  # find feasible boarding locatations an created virtual rids
        LOG.debug("user_request: {} number variable: {}".format(rq.get_rid_struct(), len(variable_boarding_point_prqs)))
        is_reservation_request = False
        if len(variable_boarding_point_prqs) > 0: 

            for vir_prq in variable_boarding_point_prqs:
                base_rid = vir_prq.get_rid()
                if vir_prq.t_pu_earliest - sim_time > self.opt_horizon:
                    is_reservation_request = True
                self.rq_dict[vir_prq.get_rid_struct()] = vir_prq
                try:
                    self.base_request_id_to_sub_rids[base_rid][vir_prq.get_rid_struct()] = 1
                except KeyError:
                    self.base_request_id_to_sub_rids[base_rid] = {vir_prq.get_rid_struct(): 1}
                self.sub_rid_to_base_request_id[vir_prq.get_rid_struct()] = base_rid

            if is_reservation_request:
                self._reservation_user_insertion(
                    sim_time, variable_boarding_point_prqs, rid_struct, calculate_parallel=calculate_parallel
                )
            else:
                self._immediate_user_insertion(
                    sim_time, variable_boarding_point_prqs, rid_struct, calculate_parallel=calculate_parallel
                )
            if is_reservation_request:
                self.current_reservation_requests += 1
            else:
                self.current_immediate_requests += 1
        else:
            LOG.warning("didnt find bp for {}".format(rid_struct))
            base_rid = rid_struct
            self.new_requests[base_rid] = 1
            self.unassigned_requests_1[base_rid] = 1
            self.base_request_id_to_sub_rids[base_rid] = {(base_rid, 0): 1}
            self.sub_rid_to_base_request_id[(base_rid, 0)] = base_rid

            offer = TravellerOffer(rid_struct, self.op_id, None, None, None)
            self.active_request_offers[rid_struct] = offer

    def get_current_offer(self, rid):
        """ this method returns the currently active offer for the request rid
        if a current offer is active:
            the current TravellerOffer is returned
        if the service is decline and the request didnt leave the system yet:
            a "service_declined" TravellerOffer is returned (at least offered_waiting_time is set to None in TravellerOffer init)
        if an offer is not evaluated yet:
            None is returned

        use the method "_create_user_offer" to create single user offers

        :param rid: request id
        :type rid: int
        :return: TravellerOffer or None for the request
        :rtype: TravellerOffer or None
        """
        if self.insertion_fetch_needed.get(rid) is not None:
            insertion_results = self.Parallelization_Manager.fetchInsertionResults()
            found_rids = {}
            for sub_rid, vid, plan, obj in insertion_results:
                base_rid = self.sub_rid_to_base_request_id[sub_rid]
                found_rids[base_rid] = 1
                try:
                    self.pending_insertions[base_rid][sub_rid] = (vid, plan, obj)
                except:
                    self.pending_insertions[base_rid] = { sub_rid : (vid, plan, obj) }
            if found_rids.get(rid) is None:
                raise EnvironmentError("insertion for rid {} not computed on parallel cores | {}".format(rid, found_rids.keys()))
            for brid in found_rids.keys():
                del self.insertion_fetch_needed[brid]
        if self.pending_insertions.get(rid) is not None:
            sub_rid_to_insertions = self.pending_insertions[rid]
            base_prq = None
            best_plan = None
            best_ass_vid = None
            best_obj = float("inf")
            variable_boarding_point_prqs = []
            for sub_rid, insertion_solution in sub_rid_to_insertions.items():
                vid, plan, obj = insertion_solution
                rq = self.rq_dict[sub_rid]
                variable_boarding_point_prqs.append(rq)
                if vid is not None and obj < best_obj:
                    best_obj = obj
                    best_plan = plan
                    base_prq = rq
                    best_ass_vid = vid
            if best_plan is not None:
                LOG.debug("base prqs: {}".format(base_prq))
                is_reservation_request = False
                if base_prq.t_pu_earliest - self.sim_time > self.opt_horizon:
                    is_reservation_request = True
                base_rid = base_prq.get_rid()
                if not is_reservation_request:
                    self.AM_Module.add_new_request(base_prq.get_rid_struct(), base_prq)
                else:
                    self.AM_Module.add_new_request(base_prq.get_rid_struct(), base_prq,
                                                 consider_for_global_optimisation=False)

                for other_prq in variable_boarding_point_prqs:  # create other requests in walking range
                    if other_prq == base_prq:
                        continue
                    # consider_for_global_optimisation=False because these requests arent actively part of the global
                    # solution algorithm
                    self.AM_Module.add_new_request(other_prq.get_rid_struct(), other_prq,
                                                 consider_for_global_optimisation=False)
                self.AM_Module.set_mutually_exclusive_assignment_constraint(
                    [o_prq.get_rid_struct() for o_prq in variable_boarding_point_prqs], base_rid)
                self.new_requests[base_rid] = 1
                self.unassigned_requests_1[base_rid] = 1

                # create offer
                offer = self._create_user_offer(base_prq, self.sim_time, best_plan)
                base_rid_struct = base_prq.get_rid_struct()
                self.active_request_offers[rid] = offer

                self.current_vid_plan_virrid_tuple_dict[rid] = (best_ass_vid, best_plan, base_rid_struct)
                if is_reservation_request:
                    self.reserved_base_rids[rid] = base_prq.t_pu_earliest
                LOG.debug(f"new offer for rid {base_rid_struct} : {offer}")
            else:
                LOG.debug(f"no offer for rid {rid}")
                offer = TravellerOffer(rid, self.op_id, None, None, None)
                self.active_request_offers[rid] = offer
            del self.pending_insertions[rid]
        if len(self.pending_insertions) == 0 and len(self.insertion_fetch_needed) == 0:
            self.cur_offer_times += round(time.time() - self.t_offer_start, 5)
        return self.active_request_offers.get(rid, None)

    def user_confirms_booking(self, rid, simulation_time):
        """This method is used to confirm a customer booking. This can trigger some database processes.

        -> delete current_vid_plan_virrid_tuple_dict[rid] and assigned plan to vehicle
            -> in case a physical customer has sent multiple requests (first/last mile) first the confirmation has to be called,
                then all requests have to be declined!
        -> call confirm_booking for all subrids -> fixing of boarding locations after global optimisation

        :param rid: request id
        :type rid: int
        :param simulation_time: current simulation time
        :type simulation_time: float
        """
        LOG.debug("user confirms booking : vid_plan_dict : {}".format(self.current_vid_plan_virrid_tuple_dict))
        vid, assigned_plan, assigned_sub_rid = self.current_vid_plan_virrid_tuple_dict[rid]
        # TODO # change for first-last-mile: self.new_accepted_base_rids also for reservation rids?
        self.new_accepted_base_rids[rid] = 1

        try:
            del self.active_request_offers[rid]
        except KeyError:
            pass

        for sub_rid in list(self.base_request_id_to_sub_rids[rid].keys()):
            LOG.debug("user confirms booking {} subrid {}".format(rid, sub_rid))
            super().user_confirms_booking(sub_rid, simulation_time)

        self.assign_vehicle_plan(self.sim_vehicles[vid], assigned_plan, simulation_time)
        self.veh_plans_without_reloc[vid] = assigned_plan   # there cant be a reloc stop here
        self.current_vid_plan_virrid_tuple_dict = {}

        if self.Parallelization_Manager is not None:
            self.Parallelization_Manager.updateOfferVehiclePlans(self.op_id, [], {vid : assigned_plan}, {assigned_sub_rid : self.rq_dict[assigned_sub_rid]})

    def user_cancels_request(self, rid, simulation_time):
        """This method is used to confirm a customer cancellation. This can trigger some database processes.

        :param rid: request id
        :type rid: int
        :param simulation_time: current simulation time
        :type simulation_time: float
        """
        try:
            del self.active_request_offers[rid]
        except KeyError:
            pass
        for sub_rid in self.base_request_id_to_sub_rids[rid].keys():
            super().user_cancels_request(sub_rid, simulation_time)
            del self.sub_rid_to_base_request_id[sub_rid]
        del self.base_request_id_to_sub_rids[rid]
        self.current_vid_plan_virrid_tuple_dict = {}
        if self.reserved_base_rids.get(rid) is not None:
            del self.reserved_base_rids[rid]

    def acknowledge_boarding(self, rid, vid, simulation_time):
        """This method can trigger some database processes whenever a passenger is starting to board a vehicle.

        :param rid: request id
        :type rid: int
        :param vid: vehicle id
        :type vid: int
        :param simulation_time: current simulation time
        :type simulation_time: float
        """
        base_request_id = rid
        LOG.debug(f"acknowledge boarding {rid} in {vid} at {simulation_time}")
        for sub_rid in self.base_request_id_to_sub_rids[base_request_id].keys():
            if self.rid_to_assigned_vid.get(sub_rid) is not None:
                if self.rid_to_assigned_vid[sub_rid] != vid:
                    LOG.error("wrong vehicle boarded compared to database!"
                              " {} -> {}".format(vid, self.rid_to_assigned_vid[sub_rid]))
                    LOG.error("{}".format(self.rid_to_assigned_vid))
                    raise EnvironmentError
                # self.rq_dict[sub_rid].set_pickup(vid, simulation_time)
                self.AM_Module.set_database_in_case_of_boarding(base_request_id, vid)
            self.rq_dict[sub_rid].set_pickup(vid, simulation_time)
            # else:
            #     del self.sub_rid_to_base_request_id[sub_rid]
            #     del self.base_request_id_to_sub_rids[base_request_id][sub_rid]
            #     super().user_cancels_request(sub_rid, simulation_time)

    def acknowledge_alighting(self, rid, vid, simulation_time):
        """This method can trigger some database processes whenever a passenger is finishing to alight a vehicle.

        :param rid: request id
        :type rid: int
        :param vid: vehicle id
        :type vid: int
        :param simulation_time: current simulation time
        :type simulation_time: float
        """
        LOG.debug(f"acknowledge alighting {rid} from {vid} at {simulation_time}")
        for sub_rid in list(self.base_request_id_to_sub_rids[rid].keys()):
            if self.rid_to_assigned_vid.get(sub_rid) is not None:
                if self.rid_to_assigned_vid[sub_rid] != vid:
                    LOG.error("wrong vehicle deboarded compared to database!"
                              " {} -> {}".format(vid, self.rid_to_assigned_vid[sub_rid]))
                    LOG.error("{}".format(self.rid_to_assigned_vid))
                    raise EnvironmentError
                LOG.debug("alighting: rid_to_assigned: {}".format(self.rid_to_assigned_vid))
                self.AM_Module.set_database_in_case_of_alighting(rid, vid)
            del self.rq_dict[sub_rid]
            try:
                del self.rid_to_assigned_vid[sub_rid]
            except KeyError:
                pass
            del self.sub_rid_to_base_request_id[sub_rid]
        del self.base_request_id_to_sub_rids[rid]

    def lock_current_vehicle_plan(self, vid):
        # TODO # change for first-last-mile: do we have to adopt this for last mile customers that are fixed in a route??
        LOG.debug("lock plan of vid {}".format(vid))
        assigned_plan = self.veh_plans.get(vid, None)
        if assigned_plan is not None:
            for ps in assigned_plan.list_plan_stops:
                ps.set_locked(True)
        if hasattr(self, "AM_Module"):
            LOG.debug(" -> also lock in AM")
            assigned_plan = self.veh_plans.get(vid, None)
            if assigned_plan is not None:
                self.AM_Module.set_assignment(vid, assigned_plan, is_external_vehicle_plan=True)
            self.AM_Module.delete_vehicle_database_entries(vid)
            for rid in get_assigned_rids_from_vehplan(assigned_plan):
                base_rid = self.sub_rid_to_base_request_id[rid]
                self.AM_Module.lock_request_to_vehicle(base_rid, vid)
                for sub_rid in self.base_request_id_to_sub_rids[base_rid].keys():
                    if sub_rid != rid:
                        super().user_cancels_request(sub_rid, self.sim_time)
                        del self.sub_rid_to_base_request_id[sub_rid]

    def time_trigger(self, simulation_time):
        self.sim_time = simulation_time
        t_offer_phase = round(time.time() - self.t_offer_phase_start, 5)
        # clear dynamic offer dicts
        self.allready_computed_vir_prqs = {}
        # activate reservation (last mile) rids if horizen reached epa
        for base_rid, epa in sorted(self.reserved_base_rids.items(), key=lambda x: x[1]):
            if epa - simulation_time > self.opt_horizon:
                break
            else:  # TODO # check if other subrids behave correctly!
                LOG.debug(f"activate {base_rid} with epa {epa} for global optimisation at time {simulation_time}!")
                del self.reserved_base_rids[base_rid]
                for sub_rid in self.base_request_id_to_sub_rids[base_rid].keys():
                    if self.rid_to_assigned_vid.get(sub_rid) is not None:
                        self.AM_Module.add_new_request(sub_rid, self.rq_dict[sub_rid], is_allready_assigned=True)
        # global optimisation
        LOG.debug("time_trigger: {} {}".format(simulation_time, self.optimisation_time_step))
        vehicles_activated = False
        if self.sim_time % self.optimisation_time_step == 0:
            LOG.info(f"time for new optimization at {simulation_time}")
            # for vid, veh in enumerate(self.sim_vehicles):
            #     LOG.debug("vehicle: {}".format(veh))
            t1 = time.time()
            self.AM_Module.compute_new_vehicle_assignments(self.sim_time, self.vid_finished_VRLs, build_from_scratch=False)
            t2 = time.time()
            self.AM_Module.optimize_boarding_points_locally()  # optimize boarding locations
            t3 = time.time()
            LOG.info(f"new assignements computed")
            LOG.info(f"assignment took {t2 - t1} | bp opt took {t3 - t2} | together {t3 - t1}")
            self._add_to_dynamic_fleetcontrol_output(simulation_time,
                {G_FCTRL_CT_RQB : t2 - t1, "bp_opt" : t3 - t2, "N_res_rqs" : self.current_reservation_requests,
                "N_im_rqs" : self.current_immediate_requests, "t_off" : self.cur_offer_times, "t_off_phase" : t_offer_phase}
            )
            self.current_immediate_requests, self.current_reservation_requests, self.cur_offer_times = 0,0,0
            self.set_new_assignments()
            if self.lock_after_first_assignment:
                self.lock_rid_vid_assignments()
            self._clear_databases()
            self.AM_Module.clear_databases()

            # TODO # zeitconstraints für last-mile?
            # festsetzen boarding punkte (pickup and dropoff) neuer requests / setzen neuer constraints
            for base_rid in list(self.new_accepted_base_rids.keys()):
                for sub_rid in list(self.base_request_id_to_sub_rids[base_rid].keys()):
                    if self.rid_to_assigned_vid.get(sub_rid) is None:
                        try:
                            del self.base_request_id_to_sub_rids[base_rid][sub_rid]
                        except KeyError:
                            pass
                        try:
                            del self.sub_rid_to_base_request_id[sub_rid]
                        except KeyError:
                            pass
                        super().user_cancels_request(sub_rid, simulation_time)
                    else:
                        # if time constraints have to be adopted
                        new_pu_time_constraints = self._get_offered_time_interval(sub_rid)
                        if new_pu_time_constraints is not None:
                            self.change_prq_time_constraints(simulation_time, sub_rid, new_pu_time_constraints[1],
                                                             new_pu_time_constraints[0])
                del self.new_accepted_base_rids[base_rid]

            # fill charging units at depot
            self.charging_management.fill_charging_units_at_depot(self, simulation_time)
            # trigger utilization dependent vehicle activate / deactivation
            if self.target_utilization is not None:
                self.charging_management.compute_activation_for_dynamic_fleetsizing(self, simulation_time)
            # activate/deactivate vehicles (ggf check lade-zustande) time_activate/deactivate
            activated_vehs = self.charging_management.time_triggered_activate(self, simulation_time)
            if len(activated_vehs) > 0:
                vehicles_activated = True
            self.charging_management.time_triggered_deactivate(self, simulation_time)
            #   ggf charging strategy exchangeFlowVehicles...
            exchange_low_soc_vehicles_at_depot(self, simulation_time, self.min_aps_soc)

        if self.repo is not None and ((simulation_time % self.repo_time_step) == 0 or vehicles_activated):
            LOG.info("Calling repositioning algorithm! (because of activated vehicles? {})".format(vehicles_activated))
            # vehplans no longer locked, because repo called very often
            self.repo.determine_and_create_repositioning_plans(simulation_time, lock=False)

        if self.util_pricing is not None: # for dynamic output of surge prices
            util, eff_util_veh, active_veh = self.compute_current_fleet_utilization(simulation_time)
            # base_fare_factor, distance_fare_factor, general_factor =\ # TODO # this is only feasible if inactive vehicles are not considered for utilization
            #     self.util_pricing.return_utilization_surge_factor(simulation_time, util)
            util = eff_util_veh/len(self.sim_vehicles)
            base_fare_factor, distance_fare_factor, general_factor = self.util_pricing.return_utilization_surge_factor(simulation_time, util)
            dyn_output_dict = {
                "utilization_all": util,
                "effective utilized vehicles": eff_util_veh,
                "active vehicles": active_veh,
                "base fare factor": base_fare_factor,
                "distance fare factor": distance_fare_factor,
                "general factor": general_factor
            }
            self._add_to_dynamic_fleetcontrol_output(simulation_time, dyn_output_dict)

        self.t_offer_phase_start = time.time()

        return {}

    def _call_time_trigger_request_batch(self, simulation_time):
        # TODO # change and test new structure in MOIA environment
        pass

    def _get_offered_time_interval(self, sub_rid):
        """this function defines new time constraints ones the sub_id is defined, base on the firstly assigned tour
        :param sub_rid:
        :return: tuple (new_earliest_pu_time_constraint, new_latest_pu_time_constraint)
        """
        # TODO # define offer time window this way?
        if self.offer_pickup_time_interval is not None:  # set new pickup time constraints based on expected pu-time and offer time interval
            prq = self.rq_dict[sub_rid]
            _, earliest_pu, latest_pu = prq.get_o_stop_info()
            vid = self.rid_to_assigned_vid[sub_rid]
            assigned_plan = self.veh_plans[vid]
            pu_time, _ = assigned_plan.pax_info.get(sub_rid)
            if pu_time - self.offer_pickup_time_interval / 2.0 < earliest_pu:
                new_earliest_pu = earliest_pu
                new_latest_pu = earliest_pu + self.offer_pickup_time_interval
            elif pu_time + self.offer_pickup_time_interval / 2.0 > latest_pu:
                new_latest_pu = latest_pu
                new_earliest_pu = latest_pu - self.offer_pickup_time_interval
            else:
                new_earliest_pu = pu_time - self.offer_pickup_time_interval / 2.0
                new_latest_pu = pu_time + self.offer_pickup_time_interval / 2.0
            return new_earliest_pu, new_latest_pu
        else:
            return None

    def _get_variable_boarding_point_requests(self, travel_request):
        # base_plan_request = PlanRequest(travel_request, self.routing_engine, min_wait_time = self.min_wait_time,
        #                                 max_wait_time = self.max_wait_time, max_detour_time_factor = self.max_dtf,
        #                                 max_constant_detour_time = self.max_cdt,
        #                                 add_constant_detour_time = self.max_cdt,
        #                                 boarding_time = self.const_bt, sub_rid_id=0)

        # search for close boarding locations
        if not travel_request.modal_state == G_RQ_STATE_LASTMILE:
            other_starts = self.boardingpoints.return_boarding_points_in_walking_range(travel_request.o_pos,
                                                                                       max_walking_range=1.5 * self.max_walking_distance,
                                                                                       max_boarding_points=self.max_other_boarding_stops)
        else:
            other_starts = self.boardingpoints.return_boarding_points_in_walking_range(travel_request.o_pos,
                                                                                       max_walking_range=1.5 * self.max_walking_distance,
                                                                                       max_boarding_points=1)
        if not travel_request.modal_state == G_RQ_STATE_FIRSTMILE:
            other_ends = self.boardingpoints.return_boarding_points_in_walking_range(travel_request.d_pos,
                                                                                     max_walking_range=1.5 * self.max_walking_distance,
                                                                                     max_boarding_points=self.max_other_boarding_stops)
        else:
            other_ends = self.boardingpoints.return_boarding_points_in_walking_range(travel_request.d_pos,
                                                                                     max_walking_range=1.5 * self.max_walking_distance,
                                                                                     max_boarding_points=1)
        if len(other_starts) == 0 or len(other_ends) == 0:
            LOG.warning("no boarding points found for {} or {}".format(travel_request.o_pos, travel_request.d_pos))
            return []
        # sort them
        base_start_pos, base_start_walking = other_starts[0]
        base_end_pos, base_end_walking = other_ends[0]
        for i, x in enumerate(other_starts):  # TODO # test for absolute max walking time?
            o_start, o_dis = x
            if o_dis > 3000.0:
                other_starts = other_starts[:i]
                break
            if o_dis > base_start_walking + self.max_walking_distance:
                other_starts = other_starts[:i]
                break
        for i, x in enumerate(other_ends):
            o_end, e_dis = x
            if e_dis > 3000.0:
                other_ends = other_ends[:i]
                break
            if e_dis > base_end_walking + self.max_walking_distance:
                other_ends = other_ends[:i]
                break
        # create base request
        t_pu_earliest = max(travel_request.rq_time + self.min_wait_time, travel_request.earliest_start_time)
        t_pu_latest = t_pu_earliest + self.max_wait_time

        created = {}
        created_prqs = []

        new_earliest_pu = t_pu_earliest + base_start_walking / self.walking_speed
        new_latest_pu = t_pu_latest + base_start_walking / self.walking_speed  # t_pu_latest
        if new_earliest_pu > new_latest_pu:  # TODO #
            LOG.debug("walking too much start {}".format(base_start_walking / self.walking_speed))
            return []
        if base_start_pos == base_end_pos:
            return []
        base_prq = PlanRequest(travel_request, self.routing_engine,
                               min_wait_time=new_earliest_pu - travel_request.rq_time,
                               max_wait_time=new_latest_pu - travel_request.rq_time,
                               max_detour_time_factor=self.max_dtf,
                               max_constant_detour_time=self.max_cdt, add_constant_detour_time=self.add_cdt,
                               min_detour_time_window=self.min_dtw, boarding_time=self.const_bt,
                               pickup_pos=base_start_pos, dropoff_pos=base_end_pos,
                               walking_time_start=base_start_walking / self.walking_speed,
                               walking_time_end=base_end_walking / self.walking_speed,
                               sub_rid_id=0)
        base_prq.set_new_pickup_time_constraint(new_latest_pu, new_earliest_pu_time=new_earliest_pu)
        created[(base_start_pos, base_end_pos)] = 1
        created_prqs = [base_prq]

        LOG.debug("create {}".format(base_prq))
        n = 1

        # other prqs based on base_prq
        for other_start_pos, start_walking_dis in other_starts:
            new_earliest_pu = base_prq.t_pu_earliest + start_walking_dis / self.walking_speed
            if new_earliest_pu > base_prq.t_pu_latest:
                LOG.debug("walking too much start {}".format(start_walking_dis / self.walking_speed))
                continue
            new_latest_pu = base_prq.t_pu_latest  # TODO # ?
            for other_end_pos, end_walking_dis in other_ends:
                if created.get((other_start_pos, other_end_pos)):
                    continue
                if other_start_pos == other_end_pos:
                    continue
                if start_walking_dis + end_walking_dis > base_prq.init_direct_td:
                    LOG.debug("walking too much end {}".format(start_walking_dis + end_walking_dis))
                    continue
                new_prq = PlanRequest(travel_request, self.routing_engine,
                                      min_wait_time=new_earliest_pu - travel_request.rq_time,
                                      max_wait_time=self.max_wait_time, max_detour_time_factor=self.max_dtf,
                                      max_constant_detour_time=self.max_cdt, add_constant_detour_time=self.add_cdt,
                                      min_detour_time_window=self.min_dtw, boarding_time=self.const_bt,
                                      pickup_pos=other_start_pos, dropoff_pos=other_end_pos,
                                      walking_time_start=start_walking_dis / self.walking_speed,
                                      walking_time_end=end_walking_dis / self.walking_speed,
                                      sub_rid_id=n)
                new_prq.set_new_pickup_time_constraint(new_latest_pu, new_earliest_pu_time=new_earliest_pu)
                created_prqs.append(new_prq)
                LOG.debug("create {}".format(new_prq))
                n += 1
        return created_prqs

    def set_new_assignments(self):
        """ this function sets the new assignments computed in the alonso-mora-module
        this function is overwritten because in case of a reassignment of a subrid, this has to be updated in self.rid_to_assigned vid
        """
        for vid, veh_obj in enumerate(self.sim_vehicles):
            assigned_plan = self.AM_Module.get_optimisation_solution(vid)
            LOG.debug("vid: {} {}".format(vid, assigned_plan))
            rids = get_assigned_rids_from_vehplan(assigned_plan)
            if len(rids) == 0 and len(get_assigned_rids_from_vehplan(self.veh_plans[vid])) == 0:
                LOG.debug("ignore assignment")
                self.AM_Module.set_assignment(vid, None)
                continue
            if assigned_plan is not None:
                LOG.debug(f"assigning new plan for vid {vid} : {assigned_plan}")
                self.assign_vehicle_plan(veh_obj, assigned_plan, self.sim_time, add_arg=True)
            else:
                LOG.debug(f"removing assignment from {vid}")
                assigned_plan = VehiclePlan(veh_obj, self.sim_time, self.routing_engine, [])
                self.assign_vehicle_plan(veh_obj, assigned_plan, self.sim_time, add_arg=True)

    def lock_rid_vid_assignments(self):
        """ this function locks all assignments of new assigned requests to the corresponding vid
        and prevents them from reassignemnt in the next opt-steps
        """
        for vid, veh_obj in enumerate(self.sim_vehicles):
            assigned_plan = self.AM_Module.get_optimisation_solution(vid)
            # LOG.debug("vid: {} {}".format(vid, assigned_plan))
            rids = get_assigned_rids_from_vehplan(assigned_plan)
            for rid in rids:
                base_rid = self.sub_rid_to_base_request_id[rid]
                if self.new_requests.get(base_rid):
                    LOG.info("lock rid {} to vid {}".format(rid, vid))
                    self.AM_Module.lock_request_to_vehicle(base_rid, vid)

    def assign_vehicle_plan(self, veh_obj, vehicle_plan, sim_time, force_assign=False, add_arg=None):
        """ this method is used to assign a new vehicle plan to a vehicle

        :param veh_obj: vehicle obj to assign vehicle plan to
        :type veh_obj: SimulationVehicle
        :param vehicle_plan: vehicle plan that should be assigned
        :type vehicle_plan: VehiclePlan
        :param sim_time: current simulation time in seconds
        :type sim_time: int
        :param force_assign: this parameter can be used to enforce the assignment, when a plan is (partially) locked
        :type force_assign: bool
        :param add_arg: set to True, if the vehicle plan is assigned internellay by AM-assignment
        :type add_arg: not defined here
        """
        new_VRL = vehicle_plan.build_VRL(veh_obj, self.rq_dict, charging_management=self.charging_management)
        veh_obj.assign_vehicle_plan(new_VRL, sim_time, force_ignore_lock=force_assign)
        self.veh_plans[veh_obj.vid] = vehicle_plan
        for rid in get_assigned_rids_from_vehplan(vehicle_plan):
            pax_info = vehicle_plan.get_pax_info(rid)
            self.rq_dict[rid].set_assigned(pax_info[0], pax_info[1])
            base_rid = self.sub_rid_to_base_request_id[rid]
            for sub_rid in self.base_request_id_to_sub_rids.get(base_rid, {}).keys():
                if self.rid_to_assigned_vid.get(sub_rid) is not None:
                    del self.rid_to_assigned_vid[sub_rid]
            self.rid_to_assigned_vid[rid] = veh_obj.vid
        if add_arg is None:
            self.AM_Module.set_assignment(veh_obj.vid, vehicle_plan, is_external_vehicle_plan=True)
        else:
            self.AM_Module.set_assignment(veh_obj.vid, vehicle_plan)

    def _create_user_offer(self, rq, simulation_time, assigned_vehicle_plan=None, offer_dict_without_plan={}):
        """ this function creates offers for the requests that will be sent to mobitopp
        # TODO # add additional parameters for fare reductions (i.e. moia card)
        # TODO # add this optional parameter to traveler
        :return: TravellerOffer
        """
        rid_struct = rq.get_rid_struct()
        if assigned_vehicle_plan is not None:
            pu_time, do_time = assigned_vehicle_plan.pax_info.get(rid_struct)
            wait_time = pu_time - rq.rq_time
            drive_time = do_time - pu_time
            pu_delay = pu_time - rq.t_pu_earliest
            rest_dist = rq.init_direct_td - 2000.0
            if rest_dist < 0:
                rest_dist = 0.0
            if self.elastic_pricing is None:
                fare = rest_dist * self.dist_fare + self.base_fare * self.fare_per_pax[rq.nr_pax]
            else:
                base_fare_factor, distance_fare_factor, general_factor = self.elastic_pricing.get_elastic_price_factors(
                    simulation_time)
                fare = rest_dist * self.dist_fare * distance_fare_factor + self.base_fare * base_fare_factor * \
                       self.fare_per_pax[rq.nr_pax] * general_factor
            taxi_fare = get_hamburg_taxi_fare(simulation_time, rq.init_direct_td, rq.nr_pax)
            if 0.95 * taxi_fare < fare:
                LOG.info(f"fare exceeds taxi fare for rid {rid_struct} | moia : {fare} taxi : {taxi_fare}")
                fare = taxi_fare * 0.95

            additional_offer_parameters = {}
            additional_offer_parameters[G_OFFER_ACCESS_W] = int(rq.walking_time_start)
            additional_offer_parameters[G_OFFER_EGRESS_W] = int(rq.walking_time_end)
            additional_offer_parameters[G_OFFER_PU_DELAY] = int(pu_delay)
            offer = TravellerOffer(rq.get_rid(), self.op_id, wait_time, drive_time, int(fare),
                                   additional_parameters=additional_offer_parameters)
        else:
            offer = TravellerOffer(rq.get_rid(), self.op_id, None, None, None)
        # add entries to all subrids
        base_rid = self.sub_rid_to_base_request_id[rq.get_rid_struct()]
        for rid in self.base_request_id_to_sub_rids[base_rid].keys():
            self.rq_dict[rid].set_service_offered(offer)
        return offer

    def _get_vehicles_to_consider_for_immediate_request(self, prq, simulation_time):
        """ this function returns a list of vehicles that should be considered for insertion of a plan request
        which should be assigned immediately
        :param prq: corresponding plan request
        :type prq: PlanRequest
        :return: list of vehicle objects considered for assignment, routing_results_dict ( (o_pos, d_pos) -> (cfv, tt, dis))
        :rtype: tuple of list of SimulationVehicle, dict
        """
        rv_vehicles, rv_results_dict = veh_search_for_immediate_request(simulation_time, prq, self)
        LOG.info("found {} for immediate rq".format(len(rv_vehicles)))
        return rv_vehicles, rv_results_dict

    def _immediate_user_insertion(self, sim_time, variable_boarding_point_prqs, rid_struct, calculate_parallel=False):
        """ computes insertion for immediate requests
        :param sim_time: current simulation time
        :param variable_boarding_point_prqs: list of other boarding point prqs
        :param rid_struct: request id of incoming traveler
        :param calculate_parallel: try to run in parallel
        :return: tuple (best prq, assigned vid, assigned plan, obj_value) if insertion possible; (None, None, None, "inf") else
        """
        if self.Parallelization_Manager is None or (len(variable_boarding_point_prqs) == 1 and not calculate_parallel):
            rid_pending_insertions = {}
            for vir_prq in variable_boarding_point_prqs:
                vehicles_to_consider, _ = self._get_vehicles_to_consider_for_immediate_request(vir_prq,
                                                                                                sim_time)
                ass_vid, ass_plan, delta_obj = single_insertion(vehicles_to_consider,
                                                                {veh.vid: self.veh_plans_without_reloc[veh.vid] for veh in
                                                                    vehicles_to_consider}, vir_prq, self.vr_ctrl_f,
                                                                self.routing_engine, self.rq_dict, sim_time,
                                                                self.const_bt, self.add_bt, check_rv=False)
                rid_pending_insertions[vir_prq.get_rid_struct()] = (ass_vid, ass_plan, delta_obj)
            self.pending_insertions[rid_struct] = rid_pending_insertions
        else:
            for vir_prq in variable_boarding_point_prqs:
                self.Parallelization_Manager.immediatePrqInsertion(self.op_id, vir_prq, sim_time)
            self.insertion_fetch_needed[rid_struct] = 1

    def _get_vehicles_to_consider_for_reservation_request(self, prq, simulation_time):
        """ this function returns a list of vehicles that should be considered for insertion of a plan request
        which pick up is far in the future
        :param prq: corresponding plan request
        :type prq: PlanRequest
        :return: list of vehicle objects considered for assignment, routing_results_dict ( (o_pos, d_pos) -> (cfv, tt, dis))
        :rtype: tuple of list of SimulationVehicle, dict
        """
        if self.rv_heuristics.get(G_RH_R_NWS) is not None:
            rv_vehicles_dict = veh_search_for_reservation_request(simulation_time, prq, self)
            LOG.info("found {} for reservation rq".format(len(rv_vehicles_dict.keys())))
            return [self.sim_vehicles[vid] for vid in rv_vehicles_dict.keys()], {}
        else:
            return self._get_vehicles_to_consider_for_immediate_request(prq, simulation_time)

    def _reservation_user_insertion(self, sim_time, variable_boarding_point_prqs, rid_struct, calculate_parallel=False):
        """ computes insertion for immediate requests
        :param sim_time: current simulation time
        :param variable_boarding_point_prqs: list of other boarding point prqs
        :param rid_struct: request id of incoming traveler
        :param calculate_parallel: try to run in parallel
        :return: tuple (best prq, assigned vid, assigned plan, obj_value) if insertion possible; (None, None, None, "inf") else
        """
        if self.Parallelization_Manager is None or (len(variable_boarding_point_prqs) == 1 and not calculate_parallel):
            rid_pending_insertion = {}
            for vir_prq in variable_boarding_point_prqs:
                vehicles_to_consider, _ = self._get_vehicles_to_consider_for_reservation_request(vir_prq,
                                                                                                    sim_time)
                ass_vid, ass_plan, delta_obj = single_insertion(vehicles_to_consider,
                                                                {veh.vid: self.veh_plans_without_reloc[veh.vid] for veh in
                                                                    vehicles_to_consider}, vir_prq, self.vr_ctrl_f,
                                                                self.routing_engine, self.rq_dict, sim_time,
                                                                self.const_bt, self.add_bt, check_rv=False,
                                                                skip_first_position_insertion=True)
                LOG.info("reservation insertion {} {} {}".format(ass_vid, ass_plan, delta_obj))
                rid_pending_insertion[vir_prq.get_rid_struct()] = (ass_vid, ass_plan, delta_obj)
            self.pending_insertions[rid_struct] = rid_pending_insertion
        else:
            for vir_prq in variable_boarding_point_prqs:
                self.Parallelization_Manager.reservationPrqInsertion(self.op_id, vir_prq, sim_time)
            self.insertion_fetch_needed[rid_struct] = 1


    def _update_vehplans_without_reloc(self, sim_time):
        """ updates the dictonary of veh plans by removing non locked reloc planstops
        in case a new time step started
        :param sim_time: simulation time
        """
        if sim_time != self.last_opt_time:
            self.veh_plans_without_reloc = {veh.vid : self.veh_plans[veh.vid].copy_and_remove_empty_planstops(veh, sim_time, self.routing_engine)
                    for veh in self.sim_vehicles}
            self.last_opt_time = sim_time
            if self.Parallelization_Manager is not None:
                self.Parallelization_Manager.updateOfferVehiclePlans(
                    self.op_id,
                    [SimulationVehicleStruct(veh, self.veh_plans_without_reloc.get(veh.vid), sim_time, self.routing_engine) for veh in self.sim_vehicles],
                    self.veh_plans_without_reloc,
                    {}
                )

