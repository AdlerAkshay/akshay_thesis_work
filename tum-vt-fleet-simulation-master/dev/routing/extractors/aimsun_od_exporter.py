import traceback
import os
import sys
SITEPACKAGES = r"C:\ProgramData\Anaconda3\Lib\site-packages"
TUM_FLEET_SIMULATION_ABS_PATH = r'C:\Users\ge37ser\Documents\Coding\TUM_VT_FleetSimulation\tum-vt-fleet-simulation'

if SITEPACKAGES not in sys.path:
    sys.path.append(SITEPACKAGES)
if TUM_FLEET_SIMULATION_ABS_PATH not in sys.path:
    sys.path.append(TUM_FLEET_SIMULATION_ABS_PATH)
	
import json
import inspect
import pandas as pd

# import Network

# --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- #
# global Variables
system = GKSystem.getSystem()
model = GKSystem.getSystem().getActiveModel()

# Write the matrix to the currently open file
# Change this method if you want to change the format of the file (ie from a list of trips
# to a matrix)
def getMatrix(matrix, factor = 100.0):
	centroids = matrix.getCentroidConfiguration().getCentroidsInOrder()
	matrix_id = matrix.getId()
	matrix_name = matrix.getName()
	vehicle_id = None
	vehicle_name = None
	if matrix.getVehicle() != None:
		vehicle_id = matrix.getVehicle().getId()
		vehicle_name = matrix.getVehicle().getName()
		
	start_time = matrix.getFrom().toString()
	duration = matrix.getDuration().toString()
	
	columns = [c.getId() for c in centroids]
	indices = [c.getId() for c in centroids]
	matrix_list = []
	for origin in centroids:
		entry_dict = {}
		for destination in centroids:
			entry_dict[destination.getId()] = matrix.getTrips( origin, destination )
		matrix_list.append(entry_dict)
	od_df = pd.DataFrame(matrix_list, index = indices)
	return {"matrix_id" : matrix_id, "matrix_name" : matrix_name, "vehicle_id" : vehicle_id, "vehicle_name": vehicle_name,
				"start_time" : start_time, "duration" : duration, "factor" : factor, "matrix" : od_df}

# Export all the matrices in a centroid configuration
def exportMatricesConf( model, centroidConf ):
	matrices = []
	odMats = centroidConf.getODMatrices()
	if odMats != None:
		for matrix in odMats:
			if matrix.isA( "GKODMatrix" ):
				matrices.append( getMatrix( matrix ) )
	return matrices

# Export all the matrices in a traffic demand
def exportMatricesDemand( model, trafficDemand ):
	matrices = []
	for schedule in trafficDemand.getSchedule():
		try:
			factor = schedule.getFactor()
		except:
			factor = 100.0
		if schedule.getTrafficDemandItem().isA( "GKODMatrix" ):
			matrices.append( getMatrix( schedule.getTrafficDemandItem(), factor=factor ) )
	return matrices

# Export matrices from object "entry". It can be either a traffic demand or a centroid configuration.
# Change here the file name and path if required.
def get( model, entry):
	container = model.getCatalog().find( int(entry) )
	if container != None:
		if container.isA( "GKCentroidConfiguration" ):
			print('Centroid configuration matrices exported')
			return exportMatricesConf( model,container )
		elif container.isA( "GKTrafficDemand" ):
			print('Traffic demand matrices exported')
			return exportMatricesDemand( model,  container )
		else:
			print('Object is neither a centroid configuration not a traffic demand')
	else:
		print('No object to export')
	return None

def getCentroidConnections(model):
	centroidconnectionType = model.getType( "GKCenConnection" )
	centroid_connection_list = []
	for cenconnection in model.getCatalog().getObjectsByType( centroidconnectionType ).values():
		con_obj = cenconnection.getConnectionObject()
		owner_obj = cenconnection.getOwner()
		#print("connection {} | {} -> {} ? ".format(cenconnection.getId(), con_obj.getId(), owner_obj.getId()) )
		centroid_connection_list.append( {"connection_id" : cenconnection.getId(), "origin_id" : owner_obj.getId(), "destination_id" : con_obj.getId(), "origin_type" : owner_obj.getType().getName() , "destination_type" : con_obj.getType().getName()})
	return pd.DataFrame(centroid_connection_list)

def getCentroids(model):
	centroidType = model.getType( "GKCentroid" )
	centroid_list = []
	for cen in model.getCatalog().getObjectsByType( centroidType ).values():
		cen_id = cen.getId()
		cen_point = cen.getPosition()
		cen_x = cen_point.x
		cen_y = cen_point.y
		centroid_list.append({"centroid id" : cen_id, "x" : cen_x, "y" : cen_y} )
	return pd.DataFrame(centroid_list)

# Entry code, the script starts here
# Export the matrices. Set the right ID before using this script.

# Identifier of the object that holds the matrices to export (either a traffic demand
# or a centroid configuration)
objectToExport = 6703532
output_folder = r'C:\Users\ge37ser\Documents\Coding\TUM_VT_FleetSimulation\tum-vt-fleet-simulation\data\networks\Aimsun_Munich_2020_04_15\aimsun_exports\data'
objects_to_export = [6703597,6703598,6703599,6703600,6703601]
for objectToExport in objects_to_export:
	matrices = get( model, objectToExport )
	cen_connection_df = getCentroidConnections(model)
	centroid_df = getCentroids(model)
	pickle_obj = {"matrices" : matrices, "cen_connections" : cen_connection_df, "centroids" : centroid_df}
	# Full path to the file in where matrices will be written
	import pickle
	with open(os.path.join(output_folder, "aimsun_od_matrices_obj_{}.pickle".format(objectToExport)), "wb") as f:
		pickle.dump(pickle_obj, f)
	print("Done")