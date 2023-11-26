from libcpp.string cimport string

cdef extern from "../contraction_hierarchies_router/ChEdge.cpp":
    pass
cdef extern from "../contraction_hierarchies_router/utils.cpp":
    pass
cdef extern from "../contraction_hierarchies_router/ChGraph.cpp":
    pass
cdef extern from "CCHGraph.cpp":
    pass
cdef extern from "CchNetwork.cpp":
    pass

cdef extern from "CchNetwork.h":

    cdef cppclass CchNetwork:
        CchNetwork(string) except +
        void updateEdgeTravelTimes(string) except +
        int computeTravelCosts1ToXpy(int start_node_index, int number_targets, int* targets, int* reached_targets, double* reached_target_tts, double* reached_target_dis, double time_range, int max_targets) except +
        int computeTravelCostsXTo1py(int start_node_index, int number_targets, int* targets, int* reached_targets, double* reached_target_tts, double* reached_target_dis, double time_range, int max_targets) except +
        void computeTravelCosts1To1py(int start_node_index, int end_node_index, double* tt, double* dis) except +
        int computeRouteSize1to1(int start_node_index, int end_node_index) except +
        void writeRoute(int* output_array) except +
