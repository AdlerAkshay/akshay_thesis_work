import logging
import os
from src.fleetctrl.planning.PlanRequest import PlanRequest
from src.fleetctrl.planning.VehiclePlan import PlanStop
from src.fleetctrl.FleetControlBase import get_assigned_rids_from_vehplan
from dev.fleetctrl.ParallelTemperingFleetcontrolBase import ParallelTemperingFleetControlBase
from src.infra.BoardingPointInfrastructure import BoardingPointInfrastructure
from src.simulation.Offers import TravellerOffer
from src.misc.globals import *

LOG = logging.getLogger(__name__)


class EasyRideBusFltctr(ParallelTemperingFleetControlBase):
    def __init__(self, op_id, operator_attributes, list_vehicles, routing_engine, zone_system, scenario_parameters, dir_names, charging_management=None):
        """This fleetcontrol is used for simulating ODM-Bus scenarios for
        EasyRide AP2330 and AP2340
        Because of large vehicle capacities it uses ParallelTempering Metaheuristic for optimisation
        idle vehicles are relocated to depots refering to public transport rail based stations

        # TODO #
        - multiple boarding locations?
        - batch assignment? -> start with
        - model for boarding process? (addition)
        - scheduling for unserved requests?
        - forced supply for some locations.

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
        super().__init__(op_id, operator_attributes, list_vehicles, routing_engine, zone_system, scenario_parameters, dir_names, charging_management=charging_management)
        self.offer_pickup_time_interval = operator_attributes.get(G_OP_OFF_TW, None)
        self.unassigned_requests = {}

        self.boardingpoints = None
        if dir_names.get(G_DIR_INFRA, None) is not None:
            if os.path.isfile(os.path.join(dir_names[G_DIR_INFRA], "boarding_points.csv")):
                self.boardingpoints = BoardingPointInfrastructure(dir_names[G_DIR_INFRA], self.routing_engine)
        self.max_other_boarding_stops = scenario_parameters.get(G_BP_MAX_BPS, 0)
        self.max_walking_distance = scenario_parameters.get(G_BP_MAX_DIS, 0)
        self.walking_speed = scenario_parameters.get(G_WALKING_SPEED, 1.0)
        self.base_request_id_to_sub_rids = {}
        self.sub_rid_to_base_request_id = {}

    def user_request(self, rq, sim_time):
        if self.boardingpoints is not None:
            other_rqs = self._get_variable_boarding_point_requests(rq)
            if len(other_rqs) > 0:
                base_prq = other_rqs[0]
                LOG.debug("base prqs: {}".format(base_prq))
                base_rid = base_prq.get_rid()
                self.rq_dict[base_prq.get_rid_struct()] = base_prq
                self.PT_Module.addNewRequest(base_prq.get_rid_struct(), base_prq)
                self.base_request_id_to_sub_rids[base_rid] = {base_prq.get_rid_struct() : 1}
                self.sub_rid_to_base_request_id[base_prq.get_rid_struct()] = base_rid

                other_prqs = other_rqs[1:]    # create other requests in walking range
                LOG.debug("other prqs: {}".format(other_prqs))
                for other_prq in other_prqs:
                    self.rq_dict[other_prq.get_rid_struct()] = other_prq    
                    self.base_request_id_to_sub_rids[base_rid][other_prq.get_rid_struct()] = 1
                    self.sub_rid_to_base_request_id[other_prq.get_rid_struct()] = base_rid
                    self.PT_Module.addNewRequest(other_prq.get_rid_struct(), other_prq, consider_for_global_optimisation=True) # optional attribute, because these requests arnt actively part of the global solution algorithm
                self.PT_Module.setMutuallyExclusiveAssignmentConstraint([o_prq.get_rid_struct() for o_prq in other_prqs] + [base_prq.get_rid_struct()], base_rid)
                self.new_requests[base_rid] = 1
                self.unassigned_requests[base_rid] = 1
            else:
                LOG.warning("didnt find bp for {}".format(rq.rid))
                base_rid = rq.rid
                self.new_requests[base_rid] = 1
                self.unassigned_requests[base_rid] = 1
                self.base_request_id_to_sub_rids[base_rid] = {(base_rid, 0) : 1 }
                self.sub_rid_to_base_request_id[(base_rid, 0)] = base_rid
        else:
            self.unassigned_requests[rq.get_rid_struct()] = 1
            self.base_request_id_to_sub_rids[rq.get_rid_struct()] = {rq.get_rid_struct() : 1}
            self.sub_rid_to_base_request_id[rq.get_rid_struct()] = rq.get_rid_struct()
            super().user_request(rq, sim_time)

    def user_cancels_request(self, rid, simulation_time):
        try:
            del self.unassigned_requests[rid]
        except:
            pass
        for sub_rid in self.base_request_id_to_sub_rids[rid].keys():
            super().user_cancels_request(sub_rid, simulation_time)
            try:
                del self.sub_rid_to_base_request_id[sub_rid]
            except:
                pass
            try:
                del self.rid_to_assigned_vid[sub_rid]
            except:
                pass
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
                try:
                    del self.rid_to_assigned_vid[sub_rid]
                except:
                    pass
                super().user_cancels_request(sub_rid, simulation_time)

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
        self.PT_Module.setDataBaseInCaseOfBoarding(base_request_id, vid)
        for sub_rid in self.base_request_id_to_sub_rids[base_request_id].keys():
            if self.rid_to_assigned_vid.get(sub_rid) is not None:
                if self.rid_to_assigned_vid[sub_rid] != vid:
                    LOG.error("wrong vehicle boarded compared to database! {} -> {}".format(vid, self.rid_to_assigned_vid[sub_rid]))
                    LOG.error("{}".format(self.rid_to_assigned_vid))
                    raise EnvironmentError
                #self.rq_dict[sub_rid].set_pickup(vid, simulation_time)
            self.rq_dict[sub_rid].set_pickup(vid, simulation_time)

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
        self.PT_Module.setDataBaseInCaseOfAlighting(rid, vid)
        for sub_rid in list(self.base_request_id_to_sub_rids[rid].keys()):
            if self.rid_to_assigned_vid.get(sub_rid) is not None:
                if self.rid_to_assigned_vid[sub_rid] != vid:
                    LOG.error("wrong vehicle deboarded compared to database! {} -> {}".format(vid, self.rid_to_assigned_vid[sub_rid]))
                    LOG.error("{}".format(self.rid_to_assigned_vid))
                    raise EnvironmentError
                #LOG.debug("alighting: rid_to_assigned: {}".format(self.rid_to_assigned_vid))
            del self.rq_dict[sub_rid]
            #self.AM_Module.delRequest(sub_rid)
            try:
                del self.rid_to_assigned_vid[sub_rid]
            except KeyError:
                pass
            del self.sub_rid_to_base_request_id[sub_rid]
        del self.base_request_id_to_sub_rids[rid]

    def time_trigger(self, simulation_time):
        super().time_trigger(simulation_time)
        if self.sim_time % self.optimisation_time_step == 0:
            # rids to be assigned in first try
            for rid in self.unassigned_requests.keys():
                assigned_subrid = None
                assigned_vid = None
                for sub_rid in self.base_request_id_to_sub_rids[rid].keys():
                    assigned_vid = self.rid_to_assigned_vid.get(sub_rid, None)
                    if assigned_vid is not None:
                        assigned_subrid = sub_rid
                        break
                if assigned_vid is None:
                    prq = self.rq_dict[list(self.base_request_id_to_sub_rids[rid].keys())[0]]
                    self._create_user_offer(prq, simulation_time)
                else:
                    prq = self.rq_dict[assigned_subrid]
                    assigned_plan = self.veh_plans[assigned_vid]
                    self._create_user_offer(prq, simulation_time, assigned_vehicle_plan=assigned_plan)
            self.unassigned_requests = {}

    def get_current_offer(self, rid):
        return super().get_current_offer(rid)

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
            self.PT_Module.set_assignment(veh_obj.vid, vehicle_plan, is_external_vehicle_plan=True)
        else:
            self.PT_Module.set_assignment(veh_obj.vid, vehicle_plan)

    def receive_status_update(self, vid, simulation_time, list_finished_VRL, force_update):
        """ this function is overwritten to trigger return to depot if plan is finished """
        super().receive_status_update(vid, simulation_time, list_finished_VRL, force_update=force_update)
        if len(list_finished_VRL) > 0 and len(self.veh_plans[vid].list_plan_stops) == 0:
            #return to depot
            if self.charging_management is not None:
                nearest_depot = self.charging_management.find_nearest_free_depot(self.sim_vehicles[vid].pos, self.op_id)
                LOG.debug("send vehicle {} to depot {}".format(vid, nearest_depot))
                ps = PlanStop(nearest_depot.pos, {}, {}, {}, {}, 0)
                ass_plan = self.veh_plans[vid]
                ass_plan.add_plan_stop(ps, self.sim_vehicles[vid], simulation_time, self.routing_engine)
                self.assign_vehicle_plan(self.sim_vehicles[vid], ass_plan, simulation_time)

    def _create_user_offer(self, prq, simulation_time, assigned_vehicle_plan=None, offer_dict_without_plan={}):
        """ creating the offer for a requests

        :param prq: plan request
        :type prq: PlanRequest obj
        :param simulation_time: current simulation time
        :type simulation_time: int
        :param assigned_vehicle_plan: vehicle plan of initial solution to serve this request
        :type assigned_vehicle_plan: VehiclePlan None
        :param offer_dict_without_plan: can be used to create an offer that is not derived from a vehicle plan
                    entries will be used to create/extend offer
        :type offer_dict_without_plan: dict or None
        :return: offer for request
        :rtype: TravellerOffer
        """
        if assigned_vehicle_plan is not None:
            pu_time, do_time = assigned_vehicle_plan.pax_info.get(prq.get_rid_struct())
            offer = {G_OFFER_WAIT: pu_time - simulation_time, G_OFFER_DRIVE: do_time - pu_time,
                     G_OFFER_FARE: int(prq.init_direct_td * self.dist_fare + self.base_fare)}
            offer = TravellerOffer(prq.get_rid(), self.op_id, pu_time - prq.rq_time, do_time - pu_time, int(prq.init_direct_td * self.dist_fare + self.base_fare))
            prq.set_service_offered(offer)  # has to be called
            self.active_request_offers[prq.get_rid()] = offer
        else:
            offer = TravellerOffer(prq.get_rid(), self.op_id, None, None, None)
            self.active_request_offers[prq.get_rid()] = offer
        return offer

    def _get_variable_boarding_point_requests(self, travel_request):
        """ other boarding points correspond to bushaltestellen on the other side, so no walking distance is assumed
        :param travel_request: traveller object
        :return: list of plan request objects
        """
        # search for close boarding locations
        other_starts = self.boardingpoints.return_boarding_points_in_walking_range(travel_request.o_pos, max_walking_range=0, max_boarding_points=self.max_other_boarding_stops, return_nearest_else=False)
        other_ends = self.boardingpoints.return_boarding_points_in_walking_range(travel_request.d_pos, max_walking_range=0, max_boarding_points=self.max_other_boarding_stops, return_nearest_else=False)
        if len(other_starts) == 0 or len(other_ends) == 0:
            LOG.warning("no boarding points found for {} or {}".format(travel_request.o_pos, travel_request.d_pos))
            return []
        # sort them
        base_start_pos, base_start_walking = other_starts[0]
        base_end_pos, base_end_walking = other_ends[0]

        #create base request
        t_pu_earliest = max(travel_request.rq_time + self.min_wait_time, travel_request.earliest_start_time)
        t_pu_latest = t_pu_earliest + self.max_wait_time

        created = {}
        created_prqs = []

        new_earliest_pu = t_pu_earliest
        new_latest_pu = t_pu_latest
        if base_start_pos == base_end_pos:
            return []
        base_prq = PlanRequest(travel_request, self.routing_engine, min_wait_time = new_earliest_pu - travel_request.rq_time, max_wait_time = new_latest_pu - travel_request.rq_time, max_detour_time_factor = self.max_dtf, 
                    max_constant_detour_time = self.max_cdt, add_constant_detour_time = self.add_cdt, min_detour_time_window=self.min_dtw, boarding_time = self.const_bt,
                    pickup_pos=base_start_pos, dropoff_pos=base_end_pos, walking_time_start=base_start_walking / self.walking_speed, walking_time_end = base_end_walking/self.walking_speed,
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
            new_latest_pu = base_prq.t_pu_latest    # TODO # ?
            for other_end_pos, end_walking_dis in other_ends:
                if created.get( (other_start_pos, other_end_pos) ):
                    continue
                if other_start_pos == other_end_pos:
                    continue
                if start_walking_dis + end_walking_dis > base_prq.init_direct_td:
                    LOG.debug("walking too much end {}".format(start_walking_dis + end_walking_dis))
                    continue
                new_prq = PlanRequest(travel_request, self.routing_engine, min_wait_time = new_earliest_pu - travel_request.rq_time, max_wait_time = self.max_wait_time, max_detour_time_factor = self.max_dtf, 
                            max_constant_detour_time = self.max_cdt, add_constant_detour_time = self.add_cdt,min_detour_time_window=self.min_dtw, boarding_time = self.const_bt, 
                            pickup_pos=other_start_pos, dropoff_pos=other_end_pos, walking_time_start=start_walking_dis / self.walking_speed, walking_time_end = end_walking_dis/self.walking_speed,
                            sub_rid_id=n)
                new_prq.set_new_pickup_time_constraint(new_latest_pu, new_earliest_pu_time=new_earliest_pu)
                created_prqs.append(new_prq)
                LOG.debug("create {}".format(new_prq))
                n += 1
        return created_prqs  

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