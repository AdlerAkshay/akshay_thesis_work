import importlib
import os
from src.FleetSimulationBase import DEFAULT_LOG_LEVEL
import traceback
from multiprocessing import Process, Queue, Pipe
import time
import numpy as np
import dill as pickle

from src.misc.globals import *
from src.misc.init_modules import load_routing_engine
from dev.fleetctrl.pooling.batch.ParallelTempering.HeatBath import HeatBath


import logging 
LOG = logging.getLogger(__name__)

#======================================================================
#COMMUNICATION CODES
KILL = 0
INIT_OP = 1
LOAD_NEW_NETWORK_TRAVEL_TIMES = 2
ADD_NEW_REQUEST = 3
SET_REQUEST_ASSIGNED = 4
SET_DB_BOARDING = 5
SET_DB_ALIGHTING = 6
DEL_REQUEST = 7
SET_MUT_EX_CONSTR = 8
LOCK_REQUEST_VEH = 9
TEMP_MOVEMENT = 10
SET_NEW_SOL = 11
SWITCH_TEMP = 12
#=====================================================================

def startProcess(q_in, q_out, process_id, scenario_parameters, dir_names):
    PP = ParallelProcess(q_in, q_out, process_id, scenario_parameters, dir_names)
    LOG.info(f"time to run PP {process_id}")
    PP.run()

class ParallelizationManager():
    def __init__(self, number_cores, scenario_parameters, dir_names):   #TODO define data_staff (for loading all the data (nw e.g. cant be transmitted directly))
        """ this class is used to manage the parallelization of functions from the AlonsoMoraAssignment module
        function calls are batched and sent to parallel processes via multiprocessing.Queue() objects
        on each multiprocessing.Process() a ParallelProcess class is initialized which is active during the whole time of simulation
            therefore the network must not be loaded during each optimisation step but is initialized on the parallel cores if initialization for the simulation
            -> this class has to be initialized before(!) the AlonsoMoraAssignment-Module since it may be inptu in its initialization
        
        communication is made with two Queues():
            q_in : this class puts tasks to be computed on the parallel processes
                    a task is defined by the tuple (communication_code, (function_arguments)) the communication codes defined as globals above define the function to be called on the parallel cores
            q_out : here all the outputs from the parallel cores are collected

        :param number_cores: number of parallel processes
        :param scenario_parameters: dictionary initialized in the beginning of the simulation for all simulation parameters
        :param dir_names: dictionary of input/ouput directories initialzied in the beginning of the simulation
        """
        self.number_cores = number_cores

        self.q_in = Queue() #communication queues
        self.q_out = Queue()

        self.processes = [Process(target = startProcess, args = (self.q_in, self.q_out, i, scenario_parameters, dir_names)) for i in range(self.number_cores)]    # start processes
        for p in self.processes:
            p.daemon = True    
            p.start()

        self.last_function_call = None  #last task sent to the parallel cores
        self.number_currently_pending_results = 0   #number of results that are expected to come from the parallel cores
        self.fetched_result_list = []       # buffer for fetched results from parallel cores

        self.current_task = -1
        self.current_task_buffer = []


    def killProcesses(self):
        """ this function is supposed to kill all parallel processes """
        for i in range(self.N_cores):
            self.q_in.put( (KILL,) )
            
        self.q_in.close()
        self.q_in.join_thread()
        self.q_out.close()
        self.q_out.join_thread()

        
        for p in self.processes:
            p.join()

    def _checkFunctionCall(self, function_id = -1):
        """ check if this function call is feasible respective to the last function call and if all results are fetched
        raises error if not
        :param function_id: corresponding communication code
        """
        if function_id != self.last_function_call:
            if self.number_currently_pending_results > 0 or len(self.fetched_result_list) > 0:
                print("results or computations from parallel processes are not cleared! use fetch functions to retrieve results!")
                print("not retrieved results from : {}".format(self.last_function_call))
                raise SyntaxError
    
    def _checkFetchCall(self, function_id):
        """ checks if all results have been fetched in case of a new function call
        raises error if not
        :param function_id: corresponding communcation code
        """
        if function_id != self.last_function_call:
            if not self.number_currently_pending_results == 0:
                LOG.error("results from former computation not fetched! use fetch-functions!")
                LOG.error("not retrieved results from : {} | current call {}".format(self.last_function_call, function_id))
                raise SyntaxError

    def initOp(self, fo_id, obj_function, operator_attributes, list_hb_tmps):
        """ initializes an operator with its specific attributes on the parallel processes
        :param fo_id: fleet control id
        :param obj_function: objective function to rate vehicle plans
        :param operator_attributes: dictionary of operator attributes
        :param list_hb_tmps: list of temperatures for heatbaths
        """
        c = 0
        if len(list_hb_tmps) < self.number_cores:
            raise EnvironmentError("Number of heatbaths should be at least the same as number of cores!")
        hbs_for_core = [[] for i in range(self.number_cores)] # tuple with hb id and temperature (init)
        for i, T in enumerate(list_hb_tmps):
            hbs_for_core[i%self.number_cores].append( (i, T) )
        for i in range(self.number_cores):
            LOG.debug(f"Queue put {INIT_OP}")
            obj_function_pickle = pickle.dumps(obj_function)
            hbs_for_this_core = hbs_for_core[i]
            self.q_in.put( (INIT_OP, (fo_id, obj_function_pickle, operator_attributes, hbs_for_this_core) ) )
            c += 1
        self.last_function_call = INIT_OP
        while c > 0:
            x = self.q_out.get()
            c -= 1
            LOG.debug(f"initOp got {x} rest {c}")

    def release_buffer(self, task_key):
        """ this function sends the accumulated tasks to the parallel processes
        in case a new task is called. this functionality should be used for methods that can be called multiple times
        without calling ohter methods in between"""
        c = 0
        if self.current_task is not None and len(self.current_task_buffer) > 0:
            for i in range(self.number_cores):
                LOG.debug(f"Queue put {self.current_task}")
                self.q_in.put((self.current_task, self.current_task_buffer) )
                c += 1
            self.last_function_call = self.current_task
            while c > 0:
                x = self.q_out.get()
                c -= 1
                LOG.debug(f"release_buffer got {x} rest {c}")
            self.current_task_buffer = []
        self.current_task = task_key

    def schedule_for_buffer(self, task_key, arg_tuple):
        """ this function checks the current buffer and manages the distribution to parallel processes
        for functions that can be called multiple times"""
        if self.current_task != task_key:
            self.release_buffer(task_key)
        self.current_task_buffer.append(arg_tuple)

    def update_network(self, sim_time):
        """ this method communicates the parallel processes to update their network
        """
        self.release_buffer(LOAD_NEW_NETWORK_TRAVEL_TIMES)
        c = 0
        for i in range(self.number_cores):
            LOG.debug(f"Queue put {LOAD_NEW_NETWORK_TRAVEL_TIMES}")
            self.q_in.put((LOAD_NEW_NETWORK_TRAVEL_TIMES, (sim_time, )) )
            c += 1
        self.last_function_call = LOAD_NEW_NETWORK_TRAVEL_TIMES
        while c > 0:
            x = self.q_out.get()
            c -= 1
            LOG.debug(f"update_network got {x} rest {c}")

    def add_new_request(self, fo_id, rid, prq, consider_for_global_optimisation=True, is_allready_assigned=False):
        """ adds request for fo_id to all subprocesses """
        self.schedule_for_buffer(ADD_NEW_REQUEST, (fo_id, rid, prq, consider_for_global_optimisation, is_allready_assigned))

    def set_request_assigned(self, fo_id, rid):
        """ set request assigned for fo_id to all subprocesses """
        self.schedule_for_buffer(SET_REQUEST_ASSIGNED, (fo_id, rid))  

    def set_database_in_case_of_boarding(self, fo_id, rid, vid):
        """ set request assigned for fo_id to all subprocesses """
        self.schedule_for_buffer(SET_DB_BOARDING, (fo_id, rid, vid))

    def set_database_in_case_of_alighting(self, fo_id, rid, vid):
        """ set request assigned for fo_id to all subprocesses """
        self.schedule_for_buffer(SET_DB_ALIGHTING, (fo_id, rid, vid)) 

    def delete_request(self, fo_id, rid):
        """ this function deletes the request from the algorithms database and solution
        :param rid: request id 
        """
        self.schedule_for_buffer(DEL_REQUEST, (fo_id, rid)) 

    def set_mutually_exclusive_assignment_constraint(self, fo_id, list_sub_rids, base_rid):
        self.schedule_for_buffer(SET_MUT_EX_CONSTR, (fo_id, list_sub_rids, base_rid)) 

    def lock_request_to_vehicle(self, fo_id, rid, vid):
        self.schedule_for_buffer(LOCK_REQUEST_VEH, (fo_id, rid, vid))

    def _setNewSol(self, fo_id, vid_to_veh_plans, vid_to_simvid, sim_time):
        """ set new init solution for heatbaths
        :param fo_id: fleet operator id
        :param vid_to_veh_plans: vid -> veh_plan dict
        :param vid_to_simvid: vid -> sim vehicle struct
        :param sim_time: current simulation time
        """
        self.release_buffer(SET_NEW_SOL)
        c = 0
        for i in range(self.number_cores):
            LOG.debug(f"Queue put {SET_NEW_SOL}")
            self.q_in.put((SET_NEW_SOL, (fo_id, vid_to_simvid, vid_to_veh_plans, sim_time)) )
            c += 1
        self.last_function_call = SET_NEW_SOL
        while c > 0:
            x = self.q_out.get()
            c -= 1
            LOG.debug(f"setNewSol got {x} rest {c}")  

    def _switchTemp(self, fo_id, temp_to_new_temp):
        """ sets new temperatures on heatbaths for this core
        :param fo_id: fleet operator id
        :param temp_to_new_temp: dict old temp -> new temp
        """
        self.release_buffer(SWITCH_TEMP)
        c = 0
        for i in range(self.number_cores):
            LOG.debug(f"Queue put {SWITCH_TEMP}")
            self.q_in.put((SWITCH_TEMP, (fo_id, temp_to_new_temp)) )
            c += 1
        self.last_function_call = SWITCH_TEMP
        while c > 0:
            x = self.q_out.get()
            c -= 1
            LOG.debug(f"switchTemp got {x} rest {c}")  

    def _temperature_movement(self, fo_id, number_iterations):
        """ starts temperature movements in all heatbaths of fleetoperator
        :param fo_id: fleet operator id
        :param number_iterations: number of iterations
        :return: dict temperature -> best sol, best cfv, current sol, current cfv
        """
        self.release_buffer(TEMP_MOVEMENT)
        c = 0
        for i in range(self.number_cores):
            LOG.debug(f"Queue put {TEMP_MOVEMENT}")
            self.q_in.put((TEMP_MOVEMENT, (fo_id, number_iterations)) )
            c += 1
        self.last_function_call = TEMP_MOVEMENT
        results = {}
        while c > 0:
            x = self.q_out.get()
            c -= 1
            LOG.debug(f"TEMP_MOVEMENT got {x} rest {c}")  
            results.update(x)
        return x


#===============================================================================================================#

class ParallelProcess():
    def __init__(self, q_in, q_out, process_id, scenario_parameters, dir_names):
        """ this class carries out the computation tasks distributed from the parallelization manager
        communication is made via two multiprocessing.Queue() objects; the functions to be excuted are communcated via the global communication codes
        this process mimics AlonsoMoraAssignment-classes with only a subset of vehicles (mostly one); 
        preprocessing steps and tree building are carried out in parallel thereby
        :param q_in: mulitprocessing.Queue() -> only inputs from the main core are put here
        :param q_out: multiprocessing.Queue() -> only outputs to the main core are put here
        :param process_id: id defined in the manager class of this process class
        :param scenario_parameters: scenario parameter entries to set up the process
        :param dir_names: dir_name paths from the simulation environment to set up the process
        """
        # routing engine
        self.sleep_time = 0.1   # short waiting time in case another process is still busy

        self.process_id = process_id
        self.q_in = q_in
        self.q_out = q_out

        self.scenario_parameters = scenario_parameters
        self.dir_names = dir_names

        # start log file
        logging.VERBOSE = 5
        logging.addLevelName(logging.VERBOSE, "VERBOSE")
        logging.Logger.verbose = lambda inst, msg, *args, **kwargs: inst.log(logging.VERBOSE, msg, *args, **kwargs)
        logging.LoggerAdapter.verbose = lambda inst, msg, *args, **kwargs: inst.log(logging.VERBOSE, msg, *args, **kwargs)
        logging.verbose = lambda msg, *args, **kwargs: logging.log(logging.VERBOSE, msg, *args, **kwargs)
        if self.scenario_parameters["log_level"]:
            level_str = self.scenario_parameters["log_level"]
            if level_str == "verbose":
                log_level = logging.VERBOSE
            elif level_str == "debug":
                log_level = logging.DEBUG
            elif level_str == "info":
                log_level = logging.INFO
            elif level_str == "warning":
                log_level = logging.WARNING
            else:
                log_level = DEFAULT_LOG_LEVEL
        else:
            log_level = DEFAULT_LOG_LEVEL
        self.log_file = os.path.join(self.dir_names[G_DIR_OUTPUT], f"00_simulation_par_{self.process_id}.log")  # logging from this process into an extra file
        if log_level < logging.INFO:
            streams = [logging.FileHandler(self.log_file), logging.StreamHandler()]
        else: # TODO # progress bar 
            print("Only minimum output to console -> see log-file")
            streams = [logging.FileHandler(self.log_file)]
        logging.basicConfig(handlers=streams,
                            level=log_level, format='%(process)d-%(name)s-%(levelname)s-%(message)s')

        LOG.info(f"Initialization of network and routing engine... on {self.process_id}")   # load the network TODO this should be communicated in a better fashion since this is allready defined
        network_type = self.scenario_parameters[G_NETWORK_TYPE]
        self.routing_engine = load_routing_engine(network_type, self.dir_names[G_DIR_NETWORK])

        self.new_routing_data_loaded = False    # flag to tell if network changed

        self.fo_data = {}   #fo_id -> parameters= (see self.initOp())

        self.sim_time = -1  # simulation time

        self.heatbaths = {} # fo_id -> tempterature -> heatbath-obj

        LOG.debug("_____________________________________")
        LOG.debug(f"PARALLEL PROCESS INITIALIZED! on {self.process_id}")
        LOG.debug("_____________________________________")
        self.time_meassure = time.time()

    def run(self):
        """ this is the main function of the parallel process which is supposed to run until the simulation terminates
        here communcatios via the Queue() objects are treated and computation tasks are carried out
        tasks from the main process have the form (function_code, (function_arguments)); the function codes are defined as globals at the top of the file
        """
        try:
            LOG.info("Process {} started work!".format(self.process_id))
            last_order = None   # some tasks might be recieved double but shouldnt; its checked with this parameter
            last_order_fo_id = None
            while True:
                x = self.q_in.get() # recieve new task from main core
                LOG.debug("Queue got {}".format(x[0]))
                if x[0] == KILL:
                    LOG.warning("Process {} got killed!".format(self.process_id))
                    return
                elif x[0] == LOAD_NEW_NETWORK_TRAVEL_TIMES: # update travel times
                    if last_order == x[0]:  # information allready here, but missing on another process -> put it back into the queue and wait for small time
                        self.q_in.put(x)
                        time.sleep(self.sleep_time)
                    else:
                        self.new_routing_data_loaded = True
                        self.routing_engine.update_network(*x[1], update_state = True)
                        self.q_out.put(LOAD_NEW_NETWORK_TRAVEL_TIMES)   # show that message is processed
                elif x[0] == INIT_OP: # initialize operator
                    if last_order == x[0] and last_order_fo_id == x[1][0]:# information allready here, but missing on another process -> put it back into the queue and wait for small time TODO still feasible for multiple operators?
                        LOG.debug(f"pid {self.process_id} : {INIT_OP}")
                        self.q_in.put(x)
                        time.sleep(self.sleep_time)
                    else:
                        # if x[0] != INIT_OP:
                        #     LOG.error("wrong input for {}! : {}".format(INIT_OP, x))
                        LOG.verbose(f"first recieved {x}")
                        self._initOp(*x[1])
                        last_order_fo_id = x[1][0]
                        self.q_out.put(INIT_OP) # show that message is processed
                elif x[0] == ADD_NEW_REQUEST: 
                    if last_order == x[0]: # information allready here, but missing on another process
                        self.q_in.put(x)
                        time.sleep(self.sleep_time)
                    else:
                        LOG.verbose(f"first recieved {x}")
                        for y in x[1]:
                            self._add_new_request(*y)
                        self.q_out.put(ADD_NEW_REQUEST)
                elif x[0] == SET_REQUEST_ASSIGNED: 
                    if last_order == x[0]: # information allready here, but missing on another process
                        self.q_in.put(x)
                        time.sleep(self.sleep_time)
                    else:
                        LOG.verbose(f"first recieved {x}")
                        for y in x[1]:
                            self._set_request_assigned(*y)
                        self.q_out.put(SET_REQUEST_ASSIGNED)
                elif x[0] == SET_DB_BOARDING: 
                    if last_order == x[0]: # information allready here, but missing on another process
                        self.q_in.put(x)
                        time.sleep(self.sleep_time)
                    else:
                        LOG.verbose(f"first recieved {x}")
                        for y in x[1]:
                            self._set_database_in_case_of_boarding(*y)
                        self.q_out.put(SET_DB_BOARDING)
                elif x[0] == SET_DB_ALIGHTING: 
                    if last_order == x[0]: # information allready here, but missing on another process
                        self.q_in.put(x)
                        time.sleep(self.sleep_time)
                    else:
                        LOG.verbose(f"first recieved {x}")
                        for y in x[1]:
                            self._set_database_in_case_of_alighting(*y)
                        self.q_out.put(SET_DB_ALIGHTING)
                elif x[0] == DEL_REQUEST: 
                    if last_order == x[0]: # information allready here, but missing on another process
                        self.q_in.put(x)
                        time.sleep(self.sleep_time)
                    else:
                        LOG.verbose(f"first recieved {x}")
                        for y in x[1]:
                            self._delete_request(*y)
                        self.q_out.put(DEL_REQUEST)
                elif x[0] == SET_MUT_EX_CONSTR: 
                    if last_order == x[0]: # information allready here, but missing on another process
                        self.q_in.put(x)
                        time.sleep(self.sleep_time)
                    else:
                        LOG.verbose(f"first recieved {x}")
                        for y in x[1]:
                            self._set_mutually_exclusive_assignment_constraint(*y)
                        self.q_out.put(SET_MUT_EX_CONSTR)
                elif x[0] == LOCK_REQUEST_VEH: 
                    if last_order == x[0]: # information allready here, but missing on another process
                        self.q_in.put(x)
                        time.sleep(self.sleep_time)
                    else:
                        LOG.verbose(f"first recieved {x}")
                        for y in x[1]:
                            self._lock_request_to_vehicle(*y)
                        self.q_out.put(LOCK_REQUEST_VEH)
                elif x[0] == SET_NEW_SOL: 
                    if last_order == x[0]: # information allready here, but missing on another process
                        self.q_in.put(x)
                        time.sleep(self.sleep_time)
                    else:
                        LOG.verbose(f"first recieved {x}")
                        self._setNewSol(*x[1])
                        self.q_out.put(SET_NEW_SOL)
                elif x[0] == SWITCH_TEMP: 
                    if last_order == x[0]: # information allready here, but missing on another process
                        self.q_in.put(x)
                        time.sleep(self.sleep_time)
                    else:
                        LOG.verbose(f"first recieved {x}")
                        self._switchTemp(*x[1])
                        self.q_out.put(SWITCH_TEMP)
                elif x[0] == TEMP_MOVEMENT: 
                    if last_order == x[0]: # information allready here, but missing on another process
                        self.q_in.put(x)
                        time.sleep(self.sleep_time)
                    else:
                        res = self._temperature_movement(*x[1])
                        self.q_out.put(res)
                else:
                    LOG.warning("I dont know what this means!")
                    LOG.warning(str(x))
                    LOG.warning("Oh boy, here I go killing myself again, {}".format(self.process_id))
                    return
                last_order = x[0]
        except:
            try:
                exc_info = os.sys.exc_info()
                try:
                    for x in range(100):
                        self.q_in.put("kill")
                except:
                    LOG.warning("break breaked!")
            finally:
                traceback.print_exception(*exc_info)
                del exc_info

                return

    def _initOp(self, fo_id, obj_function_pickle, operator_attributes, list_bh_tmps_for_this_core):
        """  this function initialize operator attributes
        :param fo_id: fleet control id
        :param obj_function_pickle: with pickle serialized objective function for rating vehicle plans
        :param operator_attributes: dictionary of operator attributes
        :param list_bh_tmps_for_this_core: list of temperature with heatbaths for this core
        """
        obj_function = pickle.loads(obj_function_pickle)
        LOG.verbose(f"unpickeld obj func {obj_function}")
        self.fo_data[fo_id] = {"obj_function" : obj_function, "std_bt" : operator_attributes[G_OP_CONST_BT], "add_bt" : operator_attributes[G_OP_ADD_BT], "operator_attributes" : operator_attributes}
        self.heatbaths[fo_id] = {}
        for hb_id, t in list_bh_tmps_for_this_core:
            heat = HeatBath(None, self.routing_engine, self.sim_time, obj_function, operator_attributes, Temperature=t, hbid=hb_id)
            try:
                self.heatbaths[fo_id][hb_id] = heat
            except:
                self.heatbaths[fo_id] = {hb_id : heat}
        LOG.debug(f"init op heatbaths {self.heatbaths}")
        LOG.debug(f"temperature list {list_bh_tmps_for_this_core}")

    def _add_new_request(self, fo_id, rid, prq, consider_for_global_optimisation = True, is_allready_assigned = False):
        for hb in self.heatbaths[fo_id].values():
            hb.add_new_request(rid, prq, consider_for_global_optimisation=consider_for_global_optimisation, is_allready_assigned=is_allready_assigned)

    def _set_request_assigned(self, fo_id, rid):
        for hb in self.heatbaths[fo_id].values():
            hb.set_request_assigned(rid)

    def _set_database_in_case_of_boarding(self, fo_id, rid, vid):
        """ this function updates the database if a boarding occurs
        not needed in this algorithm, therefore nothing happens """
        for hb in self.heatbaths[fo_id].values():
            hb.set_database_in_case_of_boarding(rid, vid)

    def _set_database_in_case_of_alighting(self, fo_id, rid, vid):
        """ this function updates the database if an alighting process occurs
        -> deletes this request from the algorithm database
        :param rid: request id that alights the vehicle
        :param vid: vehicle id the request alights from
        """
        for hb in self.heatbaths[fo_id].values():
            hb.set_database_in_case_of_alighting(rid, vid)

    def _delete_request(self, fo_id, rid):
        """ this function deletes the request from the algorithms database and solution
        :param rid: request id 
        """
        for hb in self.heatbaths[fo_id].values():
            hb.delete_request(rid)

    def _set_mutually_exclusive_assignment_constraint(self, fo_id, list_sub_rids, base_rid):
        for hb in self.heatbaths[fo_id].values():
            hb.set_mutually_exclusive_assignment_constraint(list_sub_rids, base_rid)

    def _lock_request_to_vehicle(self, fo_id, rid, vid):
        for hb in self.heatbaths[fo_id].values():
            hb.lock_request_to_vehicle(rid, vid)

    def _setNewSol(self, fo_id, vid_to_veh_plans, vid_to_simvid, sim_time):
        """ set new init solution for heatbaths
        :param fo_id: fleet operator id
        :param vid_to_veh_plans: vid -> veh_plan dict
        :param vid_to_simvid: vid -> sim vehicle struct
        :param sim_time: current simulation time
        """
        for hb in self.heatbaths[fo_id].values():
            hb.set_new_sol(vid_to_simvid, vid_to_veh_plans, sim_time)

    def _switchTemp(self, fo_id, hb_id_to_new_temp):
        """ sets new temperatures on heatbaths for this core
        :param fo_id: fleet operator id
        :param hb_id_to_new_temp: dict hb_id -> new temp
        """
        old_ts = {hb_id : hb.get_temperature() for hb_id, hb in self.heatbaths[fo_id].items()}
        LOG.verbose("switiching for hbs {}".format(old_ts))
        LOG.verbose("with {}".format(hb_id_to_new_temp))
        for hb_id, hb in self.heatbaths[fo_id].items():
            new_temp = hb_id_to_new_temp.get(hb_id)
            if new_temp is not None:
                hb.set_temperature(new_temp)
        new_ts = {hb_id : hb.get_temperature() for hb_id, hb in self.heatbaths[fo_id].items()}
        LOG.verbose("-> new hb temps: {}".format(new_ts.keys()))
        LOG.verbose("current heatbaths after switch: {}".format(self.heatbaths))

    def _temperature_movement(self, fo_id, number_iterations):
        """ starts temperature movements in all heatbaths of fleetoperator
        :param fo_id: fleet operator id
        :param number_iterations: number of iterations
        :return: dict hb_id -> temperature, best sol, best cfv, current sol, current cfv
        """
        hb_id_to_sol = {}
        LOG.debug("temp movements for hbs {}".format(self.heatbaths))
        for hb in self.heatbaths[fo_id].values():
            hb.temperature_movements(number_iterations)
            hb_best_sol, hb_best_cfv = hb.get_best_sol_found()
            hb_cur_sol, hb_cur_cfv = hb.get_current_sol_found()
            hb_id_to_sol[hb.hb_id] = (hb.get_temperature(), hb_best_sol, hb_best_cfv, hb_cur_sol, hb_cur_cfv)
            LOG.debug("para temp movement: {} -> {} | {}".format(hb.get_temperature(), hb_best_cfv, hb_cur_cfv))
        return hb_id_to_sol

