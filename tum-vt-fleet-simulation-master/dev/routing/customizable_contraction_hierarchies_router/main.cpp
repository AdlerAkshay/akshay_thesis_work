#include <iostream>
#include <cassert>
#include <math.h>
#include <vector>
#include <chrono>

#include <random>

#include "CchGraph.h"

#define START_TIMER begin = std::chrono::steady_clock::now();
#define STOP_TIMER end = std::chrono::steady_clock::now();
#define TIME_SPENT std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count()

/*

KaHIP nested-dissection ordering times:
  - Small network: 7 sec
  - Manhattan network: 7 sec
  - Munich network: 60 sec
  - Hamburg network: 13 min

Contraction times:
  - Small network: 1 sec
  - Manhattan network: 2 sec
  - Munich network: 4 sec
  - Hamburg network: 1 min
*/

std::chrono::steady_clock::time_point begin, end;

const std::string SMALL_NETWORK_PATH = "C:\\Users\\malek\\Desktop\\uni\\idp-routing\\tum-vt-fleet-simulation\\data\\networks\\EasyRideLine196_Example";
const std::string MUNICH_NETWORK_PATH = "C:\\Users\\malek\\Desktop\\uni\\idp-routing\\tum-vt-fleet-simulation\\data\\networks\\Aimsun_Munich_2020_04_15_Majid_reduced_ER_BP_all";
const std::string MANHATTEN_NETWORK_PATH = "C:\\Users\\malek\\Desktop\\uni\\idp-routing\\tum-vt-fleet-simulation\\data\\networks\\Manhattan_2019_corrected";


int main(int argc, char** argv) {

    if (argc != 2) {
        std::cout << "unexpected number of parameters given. Expected path to network as a single argument." << std::endl;
        return 1;
    }

    std::string network_path = argv[1];

    std::cout << "Network path is: " << network_path << std::endl << std::endl;

    CCHGraph cchGraph = CCHGraph(network_path, false, false);

    START_TIMER
    std::cout << std::endl << "Contracting nodes in order ..." << std::endl;
    cchGraph.contract_nodes_in_order();  
    STOP_TIMER
    int64_t time_spent = TIME_SPENT; 
    std::cout << "Time spent = " << time_spent / 60000000000 << " minutes " << (time_spent % 60000000000) / 1000000000 << " seconds" << std::endl;
    std::cout << std::endl;

    cchGraph.save_phase_one_graph();

    
    START_TIMER
    std::cout << std::endl << "Running triangle enumeration and updating weights ..." << std::endl;
    cchGraph.run_triangle_enumeration_and_update_weights();  
    STOP_TIMER
    time_spent = TIME_SPENT; 
    std::cout << "Time spent = " << time_spent / 60000000000 << " minutes " << (time_spent % 60000000000) / 1000000000 << " seconds" << std::endl;
    std::cout << std::endl;

    cchGraph.save_phase_two_graph();
}


/*
int main() {

  

  std::random_device rd;     // only used once to initialise (seed) engine
  std::mt19937 rng(rd());    // random-number engine used (Mersenne-Twister in this case)
  std::uniform_int_distribution<int> uni(0,30000); // guaranteed unbiased

  auto random_integer = uni(rng);

  CCHGraph graph = CCHGraph(MUNICH_NETWORK_PATH, true, true);

  int Ns[4] = {10, 50, 100, 500};
  long long NtoN_times[4];
  long long Nto1_times[4];

  for (int i = 0; i < 4; i ++) {
      int N = Ns[i];

      std::vector<int> sources;
      std::vector<int> targets;

      for (int i = 0; i < N; i++)
        sources.push_back(uni(rng));
      for (int i = 0; i < N; i++)
        targets.push_back(uni(rng));
      
      START_TIMER
      graph.shortest_path_N_to_N(sources, targets);
      STOP_TIMER
      NtoN_times[i] = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();

      START_TIMER
      for(auto target: targets)
        graph.shortest_path_N_to_1(sources, target);
      STOP_TIMER
      Nto1_times[i] = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
  }

  for(int i = 0; i < 4; i ++) {
    std::cout << Ns[i] << " " << NtoN_times[i] << " " << Nto1_times[i] << " " << (double) Nto1_times[i] / NtoN_times[i] << std::endl;
  }
}
*/