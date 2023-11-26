import sys
import cProfile, pstats, io

from run_scenarios_from_csv_and_config import *


""" this script can be used to profile the simulation
it can be used the same way "run_scenarios_from_csv_and_config.py" is used
it prints the computational time spent in different methods during the run
"str+c" can be used to observe only parts of the simulation
note the computations within parallel processes are not tracked
"""

if __name__ == "__main__":
    profiler = cProfile.Profile()
    profiler.enable()

    try:
        mp.freeze_support()
        if len(sys.argv) > 2:
            #constant_config_file, scenario_file, n_parallel_sim=1, n_cpu_per_sim=1
            x = sys.argv[1:]
            cont_conf = x[0]
            sc_conf = x[1]
            n_parallel_sim = 1
            if len(x) >= 3:
                n_parallel_sim = int(x[2])
            n_cpu_per_sim = 1
            if len(x) >= 4:
                n_cpu_per_sim = int(x[3])
            evaluate = 1
            if len(x) >= 5:
                evaluate = int(x[4])
            log_mode = "info"
            if len(x) >= 6:
                log_mode = x[5]
            run_scenarios(cont_conf, sc_conf, n_parallel_sim=n_parallel_sim, n_cpu_per_sim=n_cpu_per_sim, evaluate=evaluate, log_level=log_mode)
        else:
            raise IOError("Not enough arguments!")
    finally:
        profiler.disable()
        s = io.StringIO()
        #sortby = SortKey.CUMULATIVE
        ps = pstats.Stats(profiler, stream=s).sort_stats('cumtime') #tottime cumtime
        ps.print_stats(150)
        print(s.getvalue())