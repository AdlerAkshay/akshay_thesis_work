import logging
import time
from src.fleetctrl.FleetControlBase import VehiclePlan, PlanRequest
from src.fleetctrl.RidePoolingBatchOptimizationFleetControlBase import RidePoolingBatchOptimizationFleetControlBase
from dev.fleetctrl.AlonsoMoraFleetcontrolBase import AlonsoMoraFleetControlBase
from src.fleetctrl.pooling.GeneralPoolingFunctions import get_assigned_rids_from_vehplan
from src.infra.BoardingPointInfrastructure import BoardingPointInfrastructure
from src.misc.globals import *
from src.simulation.Offers import TravellerOffer

LOG = logging.getLogger(__name__)
LARGE_INT = 100000


# TODO # check replacement AlonsoMoraFleetControlBase -> RidePoolingBatchOptimizationFleetControlBase
class VariableBoardingPointsFleetcontrol(AlonsoMoraFleetControlBase):
    def __init__(self, op_id, operator_attributes, list_vehicles, routing_engine, zone_system, scenario_parameters, dir_names, charging_management=None):
        """Batch assignment fleet control for the BMW study
        ride pooling optimisation is called after every optimisation_time_step and offers are created in the time_trigger function
        if "user_max_wait_time_2" is given:
            if the user couldnt be assigned in the first try, it will be considered again in the next opt-step with this new max_waiting_time constraint
        if "user_offer_time_window" is given:
            after accepting an offer the pick-up time is constraint around the expected pick-up time with an interval of the size of this parameter

        this class is used for a ride pooling fleetcontrol where the operator looks for close boarding and alighting stops and optimizes
        pickup and dropoff points locally.
        -> only the assigned route is optimized regarding boarding locations
        depending on walking ranges multiple subrids are created that represent different boarding point combinations.
        subrequests time constraints are adopted in a way that maximum waiting and trip time isnt increased by walking
        boarding locations are fixed when a usere books the offere
        alighting locations are not fixed until drop off

        TODO new control structures in AlonsoMora assignment had to be introduced. Im not sure if all options are still feasible (at least the retry stuff shouldnt because they are treated by deleting an creating new plan requests)

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
        """
        super().__init__(op_id, operator_attributes, list_vehicles, routing_engine, zone_system, scenario_parameters, dir_names, charging_management=charging_management)
        self.max_wait_time_2 = None #TODO! but currently not feasible with subrids operator_attributes.get(G_OP_MAX_WT_2, None)
        self.offer_pickup_time_interval = operator_attributes.get(G_OP_OFF_TW, None)

        try:
            self.max_walking_distance = scenario_parameters[G_BP_MAX_DIS] # maximum walking distance to consider for subrids [m]
        except:
            LOG.error("{} not given for VariableBoardingPointsFleetcontrol!".format(G_BP_MAX_DIS))
        try:
            self.walking_speed = scenario_parameters[G_WALKING_SPEED]      # average walkign speed (m/s)
        except:
            LOG.error("{} not given for VariableBoardingPointsFleetcontrol!".format(G_WALKING_SPEED))
        try:
            self.max_other_boarding_stops = scenario_parameters.get(G_BP_MAX_BPS, None)    #maximum other boarding locations to consider at start and end for each requests (the original included)
            if self.max_other_boarding_stops < 1:
                raise EnvironmentError("at least one boarding point should be considered!")
            if self.max_other_boarding_stops is None:
                LOG.warning("max other boarding stops paramter not given!")
        except:
            LOG.error("{} not given for VariableBoardingPointsFleetcontrol!".format(G_BP_MAX_BPS))

        self.boardingpoints = None
        if dir_names.get(G_DIR_INFRA, None) is not None:
            self.boardingpoints = BoardingPointInfrastructure(dir_names[G_DIR_INFRA], self.routing_engine)
        else:
            raise EnvironmentError(f"could find input specifaction for boarding point infrastructure! {G_DIR_INFRA} not given in scenario file!")
        self.unassigned_requests_1 = {}
        self.unassigned_requests_2 = {}

        self.base_request_id_to_sub_rids = {}   # base_rid -> sub_rid -> 1
        self.sub_rid_to_base_request_id = {}    # sub_rid -> base_rid

    def user_request(self, rq, sim_time):
        """ a new customer send a request to the operator.
        the function creates sub request ids as tuple (base_id, sub_id) the original requests (boarding dropoff locations) has the id (rq_id, 0)
        the functions looks for new baording dropoff combinations that are witihin the maximum_walking_distance and inserts them into the algorithm
        these new created subrid are not considered for global optimization (are registered in the pooling alorithm but no trees are built)
        only after the global optimization, a local optimization is called in the pooling algorithm module to optimize the assigned route and pick the corresponding sub_rids
        if a new subrid is chosen, the active requests for global optimization are exchanged
        :param rq: Traveler obj
        :param sim_time: current simulation time
        """
        LOG.debug(f"Incoming request {rq.__dict__} at time {sim_time}")
        # class PlanRequest:
        #     def __init__(self, rq, routing_engine, min_wait_time=0, max_wait_time=LARGE_INT, max_detour_time_factor=None,
        #                  max_constant_detour_time=None, add_constant_detour_time=None, min_detour_time_window=None,
        #                  boarding_time=0):
        variable_boarding_point_prqs = self._get_variable_boarding_point_requests(rq)
        if len(variable_boarding_point_prqs) > 0:
            base_prq = variable_boarding_point_prqs[0]
            LOG.debug("base prqs: {}".format(base_prq))
            base_rid = base_prq.get_rid()
            self.rq_dict[base_prq.get_rid_struct()] = base_prq
            self.AM_Module.addNewRequest(base_prq.get_rid_struct(), base_prq)
            self.base_request_id_to_sub_rids[base_rid] = {base_prq.get_rid_struct() : 1}
            self.sub_rid_to_base_request_id[base_prq.get_rid_struct()] = base_rid

            other_prqs = variable_boarding_point_prqs[1:]    # create other requests in walking range
            LOG.debug("other prqs: {}".format(other_prqs))
            for other_prq in other_prqs:
                self.rq_dict[other_prq.get_rid_struct()] = other_prq    
                self.base_request_id_to_sub_rids[base_rid][other_prq.get_rid_struct()] = 1
                self.sub_rid_to_base_request_id[other_prq.get_rid_struct()] = base_rid
                self.AM_Module.addNewRequest(other_prq.get_rid_struct(), other_prq, consider_for_global_optimisation=False) # optional attribute, because these requests arnt actively part of the global solution algorithm
            self.AM_Module.setMutuallyExclusiveAssignmentConstraint([o_prq.get_rid_struct() for o_prq in other_prqs] + [base_prq.get_rid_struct()], base_rid)
            self.new_requests[base_rid] = 1
            self.unassigned_requests_1[base_rid] = 1
        else:
            LOG.warning("didnt find bp for {}".format(rq.rid))
            base_rid = rq.rid
            self.new_requests[base_rid] = 1
            self.unassigned_requests_1[base_rid] = 1
            self.base_request_id_to_sub_rids[base_rid] = {(base_rid, 0) : 1 }
            self.sub_rid_to_base_request_id[(base_rid, 0)] = base_rid

    def user_cancels_request(self, rid, simulation_time):
        """This method is used to confirm a customer cancellation.
        in the first step the assigned vehicle plan for the rid is updated (rid is removed from plan/v2rb)
        in the second step higher level data base processes are triggered to delete request

        :param rid: request id
        :type rid: int
        :param simulation_time: current simulation time
        :type simulation_time: float
        """
        assigned_sub_rid = None
        assigned_vid =None
        for sub_rid in self.base_request_id_to_sub_rids[rid].keys():
            assigned_vid = self.rid_to_assigned_vid.get(rid, None)
            if assigned_vid is not None:
                assigned_sub_rid = sub_rid
                break
        if assigned_vid is not None:
            veh_obj = self.sim_vehicles[assigned_vid]
            assigned_plan = self.AM_Module.getOptimisationSolution(assigned_vid)
            new_best_plan = self.AM_Module.getVehiclePlanWithoutRid(veh_obj, assigned_plan, assigned_sub_rid, simulation_time)
            if new_best_plan is not None:
                self.assign_vehicle_plan(assigned_vid, new_best_plan, simulation_time, force_assign=True)
            else:
                assigned_plan = VehiclePlan(veh_obj, self.sim_time, self.routing_engine, [])
                self.assign_vehicle_plan(assigned_vid, assigned_plan, simulation_time, force_assign=True)

        try:
            del self.active_request_offers[rid]
        except KeyError:
            pass

        for sub_rid in self.base_request_id_to_sub_rids[rid].keys():
            super().user_cancels_request(sub_rid, simulation_time)
            del self.sub_rid_to_base_request_id[sub_rid]
        del self.base_request_id_to_sub_rids[rid]

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
                    LOG.error("wrong vehicle boarded compared to database! {} -> {}".format(vid, self.rid_to_assigned_vid[sub_rid]))
                    LOG.error("{}".format(self.rid_to_assigned_vid))
                    raise EnvironmentError
                #self.rq_dict[sub_rid].set_pickup(vid, simulation_time)
                self.AM_Module.setDataBaseInCaseOfBoarding(base_request_id, vid)
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
                    LOG.error("wrong vehicle deboarded compared to database! {} -> {}".format(vid, self.rid_to_assigned_vid[sub_rid]))
                    LOG.error("{}".format(self.rid_to_assigned_vid))
                    raise EnvironmentError
                LOG.debug("alighting: rid_to_assigned: {}".format(self.rid_to_assigned_vid))
                self.AM_Module.setDataBaseInCaseOfAlighting(rid, vid)
            del self.rq_dict[sub_rid]
            #self.AM_Module.delRequest(sub_rid)
            try:
                del self.rid_to_assigned_vid[sub_rid]
            except KeyError:
                pass
            del self.sub_rid_to_base_request_id[sub_rid]
        del self.base_request_id_to_sub_rids[rid]

    def user_confirms_booking(self, rid, simulation_time):
        """This method is used to confirm a customer booking.
        in a first step the pick-up time constraints are updated based on the offer mad, if "user_offer_time_window" is given
        in the second step higher level database processes are triggered to fix request

        :param rid: request id
        :type rid: int
        :param simulation_time: current simulation time
        :type simulation_time: float
        """
        assigned_sub_rid = None
        for sub_rid in list(self.base_request_id_to_sub_rids[rid].keys()):
            if self.rid_to_assigned_vid.get(sub_rid) is not None:
                assigned_sub_rid = sub_rid
                break

        try:
            del self.active_request_offers[rid]
        except KeyError:
            pass

        boarded_position = self.rq_dict[assigned_sub_rid].get_o_stop_info()[0]
        for sub_rid in list(self.base_request_id_to_sub_rids[rid].keys()):
            other_boarding_pos = self.rq_dict[sub_rid].get_o_stop_info()[0]
            LOG.debug("user confirms booking {} boarding_pos {} planned boarding pos {} subrid {}".format(rid, boarded_position, other_boarding_pos, sub_rid))
            if boarded_position == other_boarding_pos:
                pu_offer_tuple = self._get_offered_time_interval(sub_rid)
                if pu_offer_tuple is not None:
                    new_earliest_pu, new_latest_pu = pu_offer_tuple
                    self.change_prq_time_constraints(simulation_time, sub_rid, new_latest_pu, new_ept=new_earliest_pu)
                #if sub_rid == assigned_sub_rid:
                super().user_confirms_booking(sub_rid, simulation_time)
            else:   
                try:
                    del self.base_request_id_to_sub_rids[rid][sub_rid]
                except KeyError:
                    pass
                try:
                    del self.sub_rid_to_base_request_id[sub_rid]
                except KeyError:
                    pass
                super().user_cancels_request(sub_rid, simulation_time)

    def _call_time_trigger_request_batch(self, simulation_time):
        """ this function first triggers the upper level batch optimisation
        based on the optimisation solution offers to newly assigned requests are created in the second step with following logic:
        declined requests will recieve an empty dict
        unassigned requests with a new assignment try in the next opt-step dont get an answer
        new assigned request will recieve a non empty offer-dict

        a retry is only made, if "user_max_wait_time_2" is given

        every request as to answer to an (even empty) offer to be deleted from the system!

        :param simulation_time: current time in simulation
        :return: dictionary rid -> offer for each unassigned request, that will recieve an answer. (offer: dictionary with plan specific entries; empty if no offer can be made)
        :rtype: dict
        """
        self.sim_time = simulation_time
        if self.sim_time % self.optimisation_time_step == 0:
            LOG.info(f"time for new optimsation at {simulation_time}")
            t1 = time.time()
            self.AM_Module.computeNewVehicleAssignments(self.sim_time, self.vid_finished_VRLs, build_from_scratch=False)
            t2 = time.time()
            self.AM_Module.optimize_boarding_points_locally()
            t3 = time.time()
            LOG.info(f"new assignements computed")
            LOG.info(f"assignment took {t2 - t1} | bp opt took {t3 - t2} | together {t3 - t1}")
            self.set_new_assignments()
            self.clearDataBases()
            self.AM_Module.clearDataBases()

        rid_to_offers = {}
        if self.sim_time % self.optimisation_time_step == 0:
            new_unassigned_requests_2 = {}
            # rids to be assigned in first try
            for base_rid in self.unassigned_requests_1.keys():
                assigned_sub_rid = None
                assigned_vid = None
                for sub_rid in self.base_request_id_to_sub_rids[base_rid].keys():
                    assigned_vid = self.rid_to_assigned_vid.get(sub_rid, None)
                    if assigned_vid is not None:
                        assigned_sub_rid = sub_rid
                        break
                
                if assigned_vid is None:
                    if self.max_wait_time_2 is not None and self.max_wait_time_2 > 0:    # retry with new waiting time constraint (no offer returned)
                        new_unassigned_requests_2[base_rid] = 1
                        for sub_rid in self.base_request_id_to_sub_rids[base_rid].keys():
                            prq = self.rq_dict[sub_rid]
                            self.AM_Module.delRequest(sub_rid)
                            _, earliest_pu, _ = prq.get_o_stop_info()
                            new_latest_pu = earliest_pu + self.max_wait_time_2
                            self.change_prq_time_constraints(simulation_time, sub_rid, new_latest_pu)
                            self.AM_Module.addNewRequest(sub_rid, prq)
                    else:   # no retry, rid declined
                        offer = TravellerOffer(base_rid, self.op_id, None, None, None)
                        self.active_request_offers[base_rid] = offer
                else:
                    assigned_plan = self.veh_plans[assigned_vid]
                    prq = self.rq_dict[assigned_sub_rid]
                    offer = self._create_user_offer(prq, simulation_time, assigned_plan)
                    self.active_request_offers[base_rid] = offer
            for base_rid in self.unassigned_requests_2.keys():   # check second try rids
                assigned_sub_rid = None
                assigned_vid = None
                for sub_rid in self.base_request_id_to_sub_rids[base_rid].keys():
                    assigned_vid = self.rid_to_assigned_vid.get(sub_rid, None)
                    if assigned_vid is not None:
                        assigned_sub_rid = sub_rid
                        break
                if assigned_vid is None:    # decline
                    offer = TravellerOffer(base_rid, self.op_id, None, None, None)
                    self.active_request_offers[base_rid] = offer
                else:
                    prq = self.rq_dict[assigned_sub_rid]
                    assigned_plan = self.veh_plans[assigned_vid]
                    offer = self._create_user_offer(prq, simulation_time, assigned_plan)
                    self.active_request_offers[base_rid] = offer
            self.unassigned_requests_1 = {}
            self.unassigned_requests_2 = new_unassigned_requests_2  # retry rids
            LOG.debug("end of opt:")
            LOG.debug("unassigned_requests_2 {}".format(self.unassigned_requests_2))
            LOG.debug("offers: {}".format(rid_to_offers))

    def vehicle_information_needed_for_optimisation(self, sim_time):
        """ needed for coupling with aimsun: tests if new vehicle information should be fetched
        better way to to that?
        """
        self.sim_time = sim_time
        if sim_time % self.optimisation_time_step == 0:
            return True
        else:
            return False

    def _get_offered_time_interval(self, rid):
        if self.offer_pickup_time_interval is not None: # set new pickup time constraints based on expected pu-time and offer time interval
            prq = self.rq_dict[rid]
            _, earliest_pu, latest_pu = prq.get_o_stop_info()
            vid = self.rid_to_assigned_vid[rid]
            assigned_plan = self.veh_plans[vid]
            pu_time, _ = assigned_plan.pax_info.get(rid)
            if pu_time - self.offer_pickup_time_interval/2.0 < earliest_pu:
                new_earliest_pu = earliest_pu
                new_latest_pu = earliest_pu + self.offer_pickup_time_interval
            elif pu_time + self.offer_pickup_time_interval/2.0 > latest_pu:
                new_latest_pu = latest_pu
                new_earliest_pu = latest_pu - self.offer_pickup_time_interval
            else:
                new_earliest_pu = pu_time - self.offer_pickup_time_interval/2.0
                new_latest_pu = pu_time + self.offer_pickup_time_interval/2.0
            return new_earliest_pu, new_latest_pu
        else:
            return None

    def _get_variable_boarding_point_requests(self, travel_request):
        # base_plan_request = PlanRequest(travel_request, self.routing_engine, min_wait_time = self.min_wait_time, max_wait_time = self.max_wait_time, max_detour_time_factor = self.max_dtf, 
        #                     max_constant_detour_time = self.max_cdt, add_constant_detour_time = self.max_cdt, boarding_time = self.const_bt, sub_rid_id=0)
        
        other_starts = self.boardingpoints.return_boarding_points_in_walking_range(travel_request.o_pos, max_walking_range=self.max_walking_distance, max_boarding_points=self.max_other_boarding_stops)
        other_ends = self.boardingpoints.return_boarding_points_in_walking_range(travel_request.d_pos, max_walking_range=self.max_walking_distance, max_boarding_points=self.max_other_boarding_stops)
        
        #create base request
        t_pu_earliest = max(travel_request.rq_time + self.min_wait_time, travel_request.earliest_start_time)
        t_pu_latest = t_pu_earliest + self.max_wait_time

        created = {}
        created_prqs = []

        base_start_pos, base_start_walking = other_starts[0]
        base_end_pos, base_end_walking = other_ends[0]
        new_earliest_pu = t_pu_earliest + base_start_walking / self.walking_speed
        if new_earliest_pu > t_pu_latest:
            LOG.debug("walking too much start {}".format(base_start_walking / self.walking_speed))
            return []
        new_latest_pu = t_pu_latest
        if base_start_pos == base_end_pos:
            return []
        base_prq = PlanRequest(travel_request, self.routing_engine, min_wait_time = new_earliest_pu - travel_request.rq_time, max_wait_time = new_latest_pu - travel_request.rq_time, max_detour_time_factor = self.max_dtf, 
                    max_constant_detour_time = self.max_cdt, add_constant_detour_time = self.add_cdt, min_detour_time_window=self.min_dtw, boarding_time = self.const_bt,
                    pickup_pos=base_start_pos, dropoff_pos=base_end_pos, walking_time_start=base_start_walking / self.walking_speed, walking_time_end = base_end_walking/self.walking_speed,
                    sub_rid_id=0)
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
            new_latest_pu = base_prq.t_pu_latest
            for other_end_pos, end_walking_dis in other_ends:
                if created.get( (other_start_pos, other_end_pos) ):
                    continue
                if other_start_pos == other_end_pos:
                    continue
                if start_walking_dis + end_walking_dis > base_prq.init_direct_td:
                    LOG.debug("walking too much end {}".format(start_walking_dis + end_walking_dis))
                    continue
                prev_max_trip_time = base_prq.max_trip_time 
                new_max_trip_time = prev_max_trip_time - end_walking_dis / self.walking_speed
                _, init_direct_tt, init_direct_td = self.routing_engine.return_travel_costs_1to1(other_start_pos, other_end_pos)
                if init_direct_tt + self.const_bt > new_max_trip_time:
                    continue
                else:
                    new_prq = PlanRequest(travel_request, self.routing_engine, min_wait_time = self.min_wait_time, max_wait_time = self.max_wait_time, max_detour_time_factor = self.max_dtf, 
                                max_constant_detour_time = self.max_cdt, add_constant_detour_time = self.add_cdt,min_detour_time_window=self.min_dtw,
                                walking_time_start=start_walking_dis / self.walking_speed, walking_time_end = end_walking_dis/self.walking_speed,
                                boarding_time = self.const_bt, sub_rid_id=n)
                    new_prq.set_new_pickup_and_dropoff_positions(other_start_pos, other_end_pos)
                    new_prq.init_direct_tt = init_direct_tt
                    new_prq.init_direct_td = init_direct_td
                    new_prq.set_new_pickup_time_constraint(new_latest_pu, new_earliest_pu_time=new_earliest_pu)
                    new_prq.set_new_max_trip_time(new_max_trip_time)
                    created_prqs.append(new_prq)
                    LOG.debug("create {}".format(new_prq))
                    n += 1
        return created_prqs    

    def _create_user_offer(self, rq, simulation_time, assigned_vehicle_plan, offer_dict_without_plan={}):
        if assigned_vehicle_plan is None:
            raise EnvironmentError("cant create offer without plan")
        pu_time, do_time = assigned_vehicle_plan.pax_info.get(rq.get_rid_struct())
        if pu_time > do_time:
            LOG.error("pu time later than drop off time!")
            LOG.error("assigned plan: {}".format(assigned_vehicle_plan))
            LOG.error("pax info: {}".format(assigned_vehicle_plan.pax_info))
            raise EnvironmentError

        extended_offer = {}
        pu_offer_tuple = self._get_offered_time_interval(assigned_vehicle_plan)
        if pu_offer_tuple is not None:
            new_earliest_pu, new_latest_pu = pu_offer_tuple
            extended_offer[G_OFFER_PU_INT_START] = new_earliest_pu
            extended_offer[G_OFFER_PU_INT_END] = new_latest_pu
        extended_offer[G_OFFER_PU_POS] = rq.get_o_stop_info()[0]
        extended_offer[G_OFFER_DO_POS] = rq.get_d_stop_info()[0]
        offer = TravellerOffer(rq.get_rid(), self.op_id, pu_time - rq.get_rq_time(), do_time - pu_time, int(rq.init_direct_td * self.dist_fare + self.base_fare),
                additional_parameters=extended_offer)
        # add entries to all subrids
        base_rid = self.sub_rid_to_base_request_id[rq.get_rid_struct()]
        for rid in self.base_request_id_to_sub_rids[base_rid].keys():
            self.rq_dict[rid].set_service_offered(offer)
        return offer

    def _get_other_plan_requests_in_range(self, request, base_plan_request):
        """ this function finds other feasible boarding and alighting points by making use of the boarding point infrastructre module
        new plan requests are created with unique origin destination relation and adopted time constraints
        :param request: traveler obj
        :param base_plan_request: corresponding base plan_request object
        :return: list of new plan requests corresponding to the same traveler but with differen o-d-relations
        """
        other_starts = self.boardingpoints.return_boarding_points_in_walking_range(base_plan_request.get_o_stop_info()[0], max_walking_range=self.max_walking_distance, max_boarding_points=self.max_other_boarding_stops)
        other_ends = self.boardingpoints.return_boarding_points_in_walking_range(base_plan_request.get_d_stop_info()[0], max_walking_range=self.max_walking_distance, max_boarding_points=self.max_other_boarding_stops)

        LOG.debug("other starts {}".format(other_starts))
        LOG.debug("other ends {}".format(other_ends))

        new_prqs = []
        created = { (base_plan_request.get_o_stop_info()[0], base_plan_request.get_d_stop_info()[0]) : 1 }
        n = 1
        for other_start_pos, start_walking_dis in other_starts:
            new_earliest_pu = base_plan_request.t_pu_earliest + start_walking_dis / self.walking_speed
            if new_earliest_pu > base_plan_request.t_pu_latest:
                LOG.debug("walking too much start {}".format(start_walking_dis / self.walking_speed))
                continue
            new_latest_pu = base_plan_request.t_pu_latest
            for other_end_pos, end_walking_dis in other_ends:
                if created.get( (other_start_pos, other_end_pos) ):
                    continue
                if other_start_pos == other_end_pos:
                    continue
                if start_walking_dis + end_walking_dis > base_plan_request.init_direct_td:
                    LOG.debug("walking too much end {}".format(start_walking_dis + end_walking_dis))
                    continue
                prev_max_trip_time = base_plan_request.max_trip_time 
                new_max_trip_time = prev_max_trip_time - end_walking_dis / self.walking_speed
                _, init_direct_tt, init_direct_td = self.routing_engine.return_travel_costs_1to1(other_start_pos, other_end_pos)
                if init_direct_tt + self.const_bt > new_max_trip_time:
                    continue
                else:
                    new_prq = PlanRequest(request, self.routing_engine, min_wait_time = self.min_wait_time, max_wait_time = self.max_wait_time, max_detour_time_factor = self.max_dtf, 
                                max_constant_detour_time = self.max_cdt, add_constant_detour_time = self.add_cdt, min_detour_time_window=self.min_dtw, boarding_time = self.const_bt, sub_rid_id=n)
                    new_prq.set_new_pickup_and_dropoff_positions(other_start_pos, other_end_pos)
                    new_prq.init_direct_tt = init_direct_tt
                    new_prq.init_direct_td = init_direct_td
                    new_prq.set_new_pickup_time_constraint(new_latest_pu, new_earliest_pu_time=new_earliest_pu)
                    new_prq.set_new_max_trip_time(new_max_trip_time)
                    new_prqs.append(new_prq)
                    LOG.debug("create {}".format(new_prq))
                    n += 1
        return new_prqs    

    def assign_vehicle_plan(self, veh_obj, vehicle_plan, sim_time, force_assign = False, add_arg = None):
        """ this method is used to assign a new vehicle plan to a vehicle

        :param veh_obj: vehicle obj to assign vehicle plan to
        :type veh_obj: SimulationVehicle
        :param vehicle_plan: vehicle plan that should be assigned
        :type vehicle_plan: VehiclePlan
        :param sim_time: current simulation time in seconds
        :type sim_time: int
        :param force_assign: this parameter can be used to enforce the assignment, when a plan is (partially) locked
        :type force_assign: bool
        :param add_arg: possible additional argument if needed
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

    def set_new_assignments(self):
        """ this function sets the new assignments computed in the alonso-mora-module
        this function is overwritten because in case of a reassignment of a subrid, this has to be updated in self.rid_to_assigned vid
        """
        for vid, veh_obj in enumerate(self.sim_vehicles):
            assigned_plan = self.AM_Module.getOptimisationSolution(vid)
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

    # TODO delete if still working!
    # def set_new_assignments(self):
    #     """ this function sets the new assignments computed in the alonso-mora-module
    #     this function is overwritten because in case of a reassignment of a subrid, this has to be updated in self.rid_to_assigned vid
    #     """
    #     for vid, veh_obj in enumerate(self.sim_vehicles):
    #         assigned_plan = self.AM_Module.getOptimisationSolution(vid)
    #         LOG.debug("vid: {} {}".format(vid, assigned_plan))
    #         if assigned_plan is not None:
    #             LOG.debug(f"assigning new plan for vid {vid} : {assigned_plan}")
    #             new_VRL = assigned_plan.build_VRL(veh_obj, self.rq_dict)
    #             veh_obj.assign_vehicle_plan(new_VRL, self.sim_time)
    #             self.veh_plans[vid] = assigned_plan
    #             self.AM_Module.set_assignment(vid, assigned_plan)
    #             rids = get_assigned_rids_from_vehplan(assigned_plan)
    #             for rid in rids:
    #                 base_rid = self.sub_rid_to_base_request_id[rid]
    #                 for sub_rid in self.base_request_id_to_sub_rids.get(base_rid, {}).keys():
    #                     if self.rid_to_assigned_vid.get(sub_rid) is not None:
    #                         del self.rid_to_assigned_vid[sub_rid]
    #                 self.rid_to_assigned_vid[rid] = vid
    #         else:
    #             LOG.debug(f"removing assignment from {vid}")
    #             assigned_plan = VehiclePlan(veh_obj, self.sim_time, self.routing_engine, [])
    #             new_VRL = assigned_plan.build_VRL(veh_obj, self.rq_dict)
    #             veh_obj.assign_vehicle_plan(new_VRL, self.sim_time)
    #             self.veh_plans[vid] = assigned_plan
    #             self.AM_Module.set_assignment(vid, assigned_plan)
        