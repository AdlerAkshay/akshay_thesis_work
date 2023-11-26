from libcpp.string cimport string

cdef extern from "ChEdge.cpp":
    pass
cdef extern from "utils.cpp":
    pass
cdef extern from "ChGraph.cpp":
    pass
cdef extern from "ChNetwork.cpp":
    pass

cdef extern from "ChNetwork.h":

    cdef cppclass ChNetwork:
        ChNetwork(string) except +
        void updateEdgeTravelTimes(string) except +
        int computeTravelCosts1ToXpy(int start_node_index, int number_targets, int* targets, int* reached_targets, double* reached_target_tts, double* reached_target_dis, double time_range, int max_targets) except +
        int computeTravelCostsXTo1py(int start_node_index, int number_targets, int* targets, int* reached_targets, double* reached_target_tts, double* reached_target_dis, double time_range, int max_targets) except +
        void computeTravelCosts1To1py(int start_node_index, int end_node_index, double* tt, double* dis) except +
        int computeRouteSize1to1(int start_node_index, int end_node_index) except +
        void writeRoute(int* output_array) except +

cdef extern from "ChGraph.cpp":

    cdef cppclass ChGraph:
        ChGraph(string, bool) except +
        void build_contraction_hierarchy() except +
        void save_contraction_hierarchy() except +
