"""
The following classes and methods can be used to solve Assignment Problems of the form
min. f(x_ij) + sum_{ij} c(x_ij) x_ij
s.t. BC_x
where i is the vehicle index and j is the assignment index.

The objective function should contain penalty terms for not assigning customers (with extra penalty for previously
assigned customers in case re-assignments are allowed). The objective should already be evaluated for each
possible x_ij before calling these classes.
In contrast, f(x_ij) can be any general objective term, which depends on a complete set of assignments.
Additional boundary conditions BC_x could be set depending on the problem type as well.

Mind: imports should be function dependent (gurobi, cplex, tensorflow, ...) in order to guarantee compatibility on
      different computer systems.
"""
from abc import ABC, abstractmethod


class OptimizedAssignment(ABC):
    @abstractmethod
    def __init__(self, v_ass_obj_dict, sim_time, fctrl, n_cpu=1, write_model=False, write_output=False):
        """This method is used to initialize the optimization problem, so that it can be solved in the next step.

        :param v_ass_obj_dict: {}: vid -> list of assignments; assignment contains costs directly related to assignment
                                c(x_ij)
        :param sim_time: current simulation time
        :param fctrl: Hailing fleet control class; can offer further information, e.g. about vehicle positions or zone
                      forecasts, or access to the routing engine
        :param n_cpu: number of cpus the optimization process can use
        :param write_model: saves optimization model to file
        :param write_output: saves optimization solution to file
        """
        self.v_ass_obj_dict = v_ass_obj_dict
        self.sim_time = sim_time
        self.fctrl = fctrl
        self.write_output = write_output

    @abstractmethod
    def solve(self):
        """This method solves the optimization problem posed in the init method.

        :return: {}: vid -> new assignment
        """
        return []
