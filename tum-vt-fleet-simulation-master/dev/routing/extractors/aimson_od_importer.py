import os
import pickle
import pandas as pd
import sys
SITEPACKAGES = r"C:\ProgramData\Anaconda3\Lib\site-packages"
TUM_FLEET_SIMULATION_ABS_PATH = r'C:\Users\ge37ser\Documents\Coding\TUM_VT_FleetSimulation\tum-vt-fleet-simulation'

if SITEPACKAGES not in sys.path:
    sys.path.append(SITEPACKAGES)
if TUM_FLEET_SIMULATION_ABS_PATH not in sys.path:
    sys.path.append(TUM_FLEET_SIMULATION_ABS_PATH)

# --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- #
# global Variables
system = GKSystem.getSystem()
model = GKSystem.getSystem().getActiveModel()

# this file needs a pickle file created from "adjust_aimsun_network.py"

def importMatrix( matrix_line, centroidConf ):
    mid = matrix_line['matrix_id']
    name = matrix_line['matrix_name']
    start_time = matrix_line['start_time']
    duration = matrix_line['duration']
    factor = matrix_line['factor']
    matrix = matrix_line['matrix']

    if mid < 0:
        aim_matrix = GKSystem.getSystem().newObject( "GKODMatrix", model )
        aim_matrix.setName( name )
        aim_matrix.setFrom( QTime.fromString( start_time, Qt.ISODate ) )
        aim_matrix.setDuration( GKTimeDuration.fromString( duration ) )
        centroidConf.addODMatrix( aim_matrix )

        for o_cen_id, columns in matrix.iterrows():
            fromCentroid = centroidConf.getModel().getCatalog().find( o_cen_id )
            if fromCentroid is None:
                print("error for centroid {}".format(o_cen_id))
                continue
            for d_cen_id, trips in columns.items():
                toCentroid = centroidConf.getModel().getCatalog().find( d_cen_id )
                if toCentroid is None:
                    print("error for centroid {}".format(d_cen_id))
                    continue
                aim_matrix.setTrips( fromCentroid, toCentroid, trips )

# Entry code, the script starts here:
centroidConfId = 115688
centroidConf = model.getCatalog().find( int(centroidConfId) )
file = r'C:\Users\ge37ser\Desktop\tmp\reduced_matrix_10.0.pickle'
f = open(file, "rb")
matrix_data = pickle.load(f)
f.close()
#importMatrix(matrix_data[0], centroidConf)
for matrix_line in matrix_data:
    importMatrix(matrix_line, centroidConf)
print("file {} imported!".format(file))