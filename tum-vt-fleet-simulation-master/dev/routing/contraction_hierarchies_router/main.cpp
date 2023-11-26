#include <vector>
#include <chrono>
#include <math.h>
#include <iostream>
#include <cassert>
#include <fstream>
#include <cfloat>

#include "ChGraph.h"

#define START_TIMER begin = std::chrono::steady_clock::now();
#define STOP_TIMER end = std::chrono::steady_clock::now();
#define TIME_SPENT std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count()

/*
Contraction times:
   - Small network: ~1min
   - Manhatten network: ~11min
   - Munich network: ~428min
*/

std::chrono::steady_clock::time_point begin, end;

//const std::string SMALL_NETWORK_PATH = "/home/vinzothex/idp/tum-vt-fleet-simulation/data/networks/EasyRideLine196_Example";
//const std::string MUNICH_NETWORK_PATH = "/home/vinzothex/idp/tum-vt-fleet-simulation/data/networks/Aimsun_Munich_2020_04_15_Majid_reduced_ER_BP_all";
//const std::string MANHATTEN_NETWORK_PATH = "/home/vinzothex/idp/tum-vt-fleet-simulation/data/networks/Manhattan_2019_corrected";

const std::string SMALL_NETWORK_PATH = "C:\\Users\\malek\\Desktop\\uni\\idp-routing\\tum-vt-fleet-simulation\\data\\networks\\EasyRideLine196_Example";
const std::string MUNICH_NETWORK_PATH = "C:\\Users\\malek\\Desktop\\uni\\idp-routing\\tum-vt-fleet-simulation\\data\\networks\\Aimsun_Munich_2020_04_15_Majid_reduced_ER_BP_all";
const std::string MANHATTEN_NETWORK_PATH = "C:\\Users\\malek\\Desktop\\uni\\idp-routing\\tum-vt-fleet-simulation\\data\\networks\\Manhattan_2019_corrected";
const std::string HAMBURG_NETWORK_PATH = "C:\\Users\\malek\\Desktop\\uni\\idp-routing\\tum-vt-fleet-simulation\\data\\networks\\MOIA_HH_12052021_raw";

int main(int argc, char** argv) {

    if (argc != 2) {
        std::cout << "unexpected number of parameters given. Expected path to network as a single argument." << std::endl;
        return 1;
    }

    std::string network_path = argv[1];

    std::cout << "Network path is: " << network_path << std::endl << std::endl;

    ChGraph graph = ChGraph(network_path, false);
    
    START_TIMER
    std::cout << std::endl << "Building contraction hierarchy ..." << std::endl;
    graph.build_contraction_hierarchy();  
    STOP_TIMER
    int64_t time_spent = TIME_SPENT; 
    std::cout << "Time spent = " << time_spent / 60000000000 << " minutes " << (time_spent % 60000000000) / 1000000000 << " seconds" << std::endl;
    std::cout << std::endl;

    std::cout << "Saving contracted graph ... ";
    graph.save_contraction_hierarchy();
    std::cout << "Done.\n";
    
    return 0;
}