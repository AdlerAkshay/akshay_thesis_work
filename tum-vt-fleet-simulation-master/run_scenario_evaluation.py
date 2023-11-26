from src.evaluation.standard import *
import src.evaluation.temporal as temporal
import os

if __name__ == "__main__":
    # scs = ["test_mod_400", "test_mod_800", "test_mod_1600"]
    # scs = ["bugfix3"]

    # # base = r'results/ISTTT_Test_MoD'
    # base = r'results/ISTTT_bugfix'
    # for sc in scs:
    #     scenario_dir = os.path.join(base, sc)
    #     standard_evaluation(scenario_dir)

    # sc_ints = [16] #[2, 3, 4, 5, 17, 18, 19]

    study = r'C:\Users\ge37ser\Documents\Coding\TUM_VT_FleetSimulation\tum-vt-fleet-simulation\results\FabianRPPsc01'
    for sc in os.listdir(study):
        if os.path.isdir(os.path.join(study, sc)):
            standard_evaluation(os.path.join(study, sc))
            temporal.run_complete_temporal_evaluation(os.path.join(study, sc))

    # for sc_int in sc_ints:
    #     sc_dir=r'C:\Users\ge37ser\Documents\Paper_me\TRB20\Simulationen_KIT\TRB2020_fleetsimMobitopp\scenario{}'.format(sc_int)
    #     try:
    #         standard_evaluation(sc_dir)
    #         #temporal.run_complete_temporal_evaluation(sc_dir)
    #     except:
    #         print("sc_int {} could not be evaluated".format(sc_int))
    #         print(sc_dir)
    # sc_dir = r'C:\Users\ge37ser\Downloads\mobitopp-fleet-example-simulation-100p-scenario-5\results\simulation\scenario5'
    # standard_evaluation(sc_dir)
    # temporal.run_complete_temporal_evaluation(sc_dir)

