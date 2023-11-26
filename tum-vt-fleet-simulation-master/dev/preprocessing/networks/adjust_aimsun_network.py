import os
import pickle
import numpy as np
import pandas as pd
from shapely.geometry import Point, LineString
import geopandas as gpd 
import matplotlib.pyplot as plt
try:
    from dev.preprocessing.networks.network_manipulation import FullNetwork, NetworkNode, NetworkEdge
except:
    from network_manipulation import FullNetwork, NetworkNode, NetworkEdge
os.sys.path.append(r'C:\Users\ge37ser\Documents\Projekte\EasyRide\AP2300\AP2320\scripts')
from loadOperatingAreas import *

""" this script can be used to create reduced aimsun od matrices which can be loaded again into aimsun while
creating demand-files for the fleet simulation
# TODO # comment!
"""

def convert_time_string_to_seconds(time_str):
    hours, minutes, seconds = time_str.split(":")
    return int(hours)*3600 + int(minutes)*60 + int(seconds)

def poissonEvents(start_time, end_time, rate):
    events = []
    if rate == 0:
        return []
    t = start_time
    while t < end_time:
        r = np.random.random()
        t = t - np.math.log(r)/rate
        if t > end_time:
            break
        events.append(t)
    return events

class AimsunODloader():
    def __init__(self, filepath_list):
        """ filepath: absolute path to pickle file created by aimsun_od_exporter.py (.../routing/extractors)
        """
        self.od_matrix_list = []
        self.centroid_connector_df = None
        self.centroid_df = None
        for filepath in filepath_list:
            f = open(filepath, "rb")
            data = pickle.load(f)
            f.close()
            if self.centroid_connector_df is None:
                self.centroid_connector_df = data.get("cen_connections")
                if self.centroid_connector_df is not None:
                    print(" -> centroid connector data frame loaded")
                    print(self.centroid_connector_df.head())
            if self.centroid_df is None:
                self.centroid_df = data.get("centroids")
                self.centroids_within_oa = None
                if self.centroid_df is not None:
                    print(" -> centroids loaded!")
                    print(self.centroid_df.head())
            self.od_matrix_list += data.get("matrices", [])
            if self.od_matrix_list is not None:
                print(" -> following {} od-matrices found: ".format(len(self.od_matrix_list)))
                for entry in self.od_matrix_list:
                    print(f"aimsun id {entry['matrix_id']} | aimsun name {entry['matrix_name']} | start_time {entry['start_time']} | duration {entry['duration']}")
        self.active_od_matrix = None
        self.active_od_infos = None

        self.full_network = None
        self.centroid_to_stop_node_indeces = None

        self.frac_seed_to_request_df = {}
        self.frac_to_reduced_matrix_list = {}

    def selectODmatrix(self, aimsun_id):
        """ sets the specifc od matrix as active to create demand """
        for entry in self.od_matrix_list:
            if entry['matrix_id'] == aimsun_id:
                self.active_od_matrix = entry['matrix']
                self.active_od_infos = {key : val for key, val in entry.items() if key != 'matrix'}
                print("active od matrix: {} | {} | from {} duration {} | aimsun fraction {}".format(self.active_od_infos['matrix_id'], self.active_od_infos['matrix_name'], self.active_od_infos["start_time"], self.active_od_infos["duration"], self.active_od_infos["factor"]))
                #print(self.active_od_matrix.head())
                return
        raise KeyError(aimsun_id)

    def loadBMWoperatingAreaAndSetCentroidsWithinOA(self):
        bmw_oa = loadBMWArea()
        #ax = bmw_oa.plot()
        centroid_locations_list = []
        for key, entries in self.centroid_df.iterrows():
            centroid_locations_list.append( {"id" : entries["centroid id"], "geometry" : Point(entries["x"], entries["y"])})
        cen_gdf = gpd.GeoDataFrame(centroid_locations_list)
        #cen_gdf.plot(ax = ax, color = "r")
        def isIn(centroid_gdf_row):
            for key, entries in bmw_oa.iterrows():
                if centroid_gdf_row['geometry'].within(entries['geometry']):
                    return True
            return False
        cen_gdf_within = cen_gdf[cen_gdf.apply(isIn, axis=1)]
        #cen_gdf_within.plot(ax = ax, color = 'g')
        #plt.show()

        cen_within = {}
        for key, entries in cen_gdf_within.iterrows():
            cen_within[entries['id']] = 1
        print("{}/{} are within the operating area!".format(cen_gdf_within.shape[0], cen_gdf.shape[0]))
        self.centroids_within_oa = cen_within

    def load_aimsun_exported_full_network(self, network_path):
        self.full_network = FullNetwork(network_path)

    def _create_stop_node_at_super_node(self, aimsun_node_id):
        """ creates a is_stop_only-node at a specific super_node (aimsun_node)
        the stop node is connected to all incoming and outgoing aimsun sections
        the edges are defined as "turns"
        """
        current_node_number = self.full_network.getCurrentNodeNumber()
        super_node_id = self.full_network.getSuperNodeId(aimsun_node_id)
        super_node_outgoing_nodes = self.full_network.getSuperNodeOutgoingStartNodes(super_node_id)
        super_node_incoming_nodes = self.full_network.getSuperNodeIncomingEndNodes(super_node_id)
        super_node_x = np.mean([self.full_network.getNodeCoordinates(n_id)[0] for n_id in super_node_incoming_nodes + super_node_outgoing_nodes])
        super_node_y = np.mean([self.full_network.getNodeCoordinates(n_id)[1] for n_id in super_node_incoming_nodes + super_node_outgoing_nodes])
        new_node = NetworkNode(current_node_number, super_node_x, super_node_y, is_stop_only=True)
        self.full_network.addNode(new_node)
        outgoing_edges = []
        for node_id in super_node_outgoing_nodes:
            outgoing_edge = NetworkEdge(new_node.index, node_id, 0, 0, source_edge_id=aimsun_node_id, polyline=[self.full_network.getNodeCoordinates(new_node.index), self.full_network.getNodeCoordinates(node_id)], roadtype="Stop_Connector")
            outgoing_edges.append(outgoing_edge)
            self.full_network.addEdge(outgoing_edge)
        incoming_edges = []
        for node_id in super_node_incoming_nodes:
            incoming_edge = NetworkEdge(node_id, new_node.index, 0, 0, source_edge_id=aimsun_node_id, polyline=[self.full_network.getNodeCoordinates(node_id), self.full_network.getNodeCoordinates(new_node.index)], roadtype="Stop_Connector")
            incoming_edges.append(incoming_edge)
            self.full_network.addEdge(incoming_edge)
        return new_node.index

    def find_same_centroids(self):
        centroid_to_connector_ids = {}
        for i, entries in self.centroid_connector_df.iterrows():
            supernode_id = entries['destination_id']
            centroid_id = entries['origin_id']
            try:
                centroid_to_connector_ids[centroid_id][supernode_id] = 1
            except:
                centroid_to_connector_ids[centroid_id] = {supernode_id : 1}
        connector_tuple_to_centroids = {}
        for cen, con_dict in centroid_to_connector_ids.items():
            con_tuple = tuple(sorted(con_dict.keys()))
            try:
                connector_tuple_to_centroids[con_tuple].append(cen)
            except:
                connector_tuple_to_centroids[con_tuple] = [cen]
        c = 0
        for con_tuple, cens in connector_tuple_to_centroids.items():
            if len(cens) > 1:
                print("double centroid: {} -> {}".format(con_tuple, cens))
                c += 1
        print("{} at least double pairs found out of {} centroids".format(c, len(centroid_to_connector_ids.keys())))


    def create_centroid_connector_stop_nodes(self):
        """ TODO aimsun distinguish between FROM and TO connection ? """
        created = {}
        centroid_to_stop_node_indices = {}
        for i, entries in self.centroid_connector_df.iterrows():
            if entries['destination_type'] == "GKNode":
                supernode_id = entries['destination_id']
                centroid_id = entries['origin_id']
                if self.centroids_within_oa is None or self.centroids_within_oa.get(centroid_id) is not None:
                    if created.get(supernode_id) is None:
                        node_index = self._create_stop_node_at_super_node(supernode_id)
                        created[supernode_id] = node_index
                    else:
                        node_index = created[supernode_id]
                    try:
                        centroid_to_stop_node_indices[centroid_id].append(node_index)
                    except:
                        centroid_to_stop_node_indices[centroid_id] = [node_index]
            if entries['destination_type'] == "GKSection":
                centroid_id = entries['origin_id']
                if self.centroids_within_oa.get(centroid_id) is not None:
                    print(" -> section connection")
                    print(f"{entries}")
            if entries['origin_type'] != "GKCentroid":
                print(entries)
        print("created {} stop nodes for {} centroids".format(len(created.keys()), len(centroid_to_stop_node_indices.keys())))
        self.full_network.addNodeAttributesToClasses()
        self.centroid_to_stop_node_indeces = centroid_to_stop_node_indices
        print("following centroids not created but within oa:")
        for cen in self.centroids_within_oa.keys():
            if self.centroid_to_stop_node_indeces.get(cen) is None:
                print(f"-> {cen}")

    def create_od_requests_from_active_od(self, demand_fractions_in_per = [100], number_seeds = 1):
        fraction_seed_to_request_list = {}
        od_dict = {}
        s = 0
        for o, d_series in self.active_od_matrix.iterrows():
            for d, val in d_series.items():
                s += val
                if val > 0.0:
                    try:
                        od_dict[o][d] = val
                    except:
                        od_dict[o] = {d: val}
        print("sum", s)
        start_time = convert_time_string_to_seconds( self.active_od_infos["start_time"] )
        duration = convert_time_string_to_seconds( self.active_od_infos["duration"] )
        aimsun_fraction = float(self.active_od_infos["factor"])/100.0
        for frac in demand_fractions_in_per:
            r = frac/100.0
            for s in range(number_seeds):
                od_request_list = []
                sr = 0
                sa = 0
                for o, d_dict in od_dict.items():
                    for d, val in d_dict.items():
                        sa += val
                        if self.centroids_within_oa is None or (self.centroids_within_oa.get(o) is not None and self.centroids_within_oa.get(d) is not None):
                            val = val*aimsun_fraction
                            sr += val * r
                            events = poissonEvents(start_time, start_time + duration, val * r / duration)
                            for t in events:
                                od_request_list.append( (int(np.round(t)), o, d) )
                od_request_list = sorted(od_request_list, key = lambda x:x[0] )
                print("{}/{} of global {} requests created with frac {}".format(len(od_request_list), sr, sa, frac))
                fraction_seed_to_request_list[ (frac, s) ] = [ (i, x[0], x[1], x[2] ) for i, x in enumerate(od_request_list) ]
        return fraction_seed_to_request_list

    def create_request_dfs_from_centroid_to_stop_nodes(self, demand_fractions_in_per = [100], number_seeds = 1):
        fraction_seed_to_od_request_list = self.create_od_requests_from_active_od(demand_fractions_in_per=demand_fractions_in_per, number_seeds=number_seeds)
        frac_seed_to_request_df = {}
        for key, request_od_list in fraction_seed_to_od_request_list.items():
            rq_df_list = []
            for i, time, o_cen, d_cen in request_od_list:
                o_node_id = np.random.choice(self.centroid_to_stop_node_indeces[o_cen])
                d_node_id = np.random.choice(self.centroid_to_stop_node_indeces[d_cen])
                #request_id,rq_time,start,end
                rq_df_list.append( {"request_id" : i, "rq_time" : time, "start" : o_node_id, "end" : d_node_id})
            rq_df = pd.DataFrame(rq_df_list)
            frac_seed_to_request_df[key] = rq_df
        self.frac_seed_to_request_df = frac_seed_to_request_df

    def create_request_dfs_from_centroid_to_stop_nodes_allMatrices(self, demand_fractions_in_per = [100], number_seeds = 1):
        frac_seed_to_od_request_list = {}
        for frac in demand_fractions_in_per:
            for s in range(number_seeds):
                frac_seed_to_od_request_list[(frac, s)] = []
        for matrix in self.od_matrix_list:
            mid = matrix['matrix_id']
            self.selectODmatrix(mid)
            single_matrix_fraction_seed_to_od_request_list = self.create_od_requests_from_active_od(demand_fractions_in_per=demand_fractions_in_per, number_seeds=number_seeds)
            for key, rq_list in single_matrix_fraction_seed_to_od_request_list.items():
                frac_seed_to_od_request_list[key].extend(rq_list)

        frac_seed_to_request_df = {}
        for key, request_od_list in frac_seed_to_od_request_list.items():
            rq_df_list = []
            c = 0
            for i, time, o_cen, d_cen in sorted(request_od_list, key = lambda x : x[1]):
                o_node_id = np.random.choice(self.centroid_to_stop_node_indeces[o_cen])
                d_node_id = np.random.choice(self.centroid_to_stop_node_indeces[d_cen])
                #request_id,rq_time,start,end
                rq_df_list.append( {"request_id" : c, "rq_time" : time, "start" : o_node_id, "end" : d_node_id})
                c += 1
            print(" number of rqs: {} for {}".format(len(rq_df_list), key))
            rq_df = pd.DataFrame(rq_df_list)
            frac_seed_to_request_df[key] = rq_df
        self.frac_seed_to_request_df = frac_seed_to_request_df

    def createReducedAimsunODMatrix(self, demand_fractions_in_per = [100.0]):
        frac_to_matrix_list = {}
        for frac in demand_fractions_in_per:
            rel_frac = frac/100.0
            def reducer(x):
                if self.centroids_within_oa is None or self.centroids_within_oa.get(x.name) is not None:
                    for o, val in x.items():
                        if self.centroids_within_oa is None or self.centroids_within_oa.get(o) is not None:
                            x[o] = val * (1.0 - rel_frac)
                return x
            before = 0.0
            after = 0.0
            before_factor = 0.0
            after_factor = 0.0
            frac_to_matrix_list[frac] = []
            for matrix_data in self.od_matrix_list:
                old_id = matrix_data['matrix_id']
                old_name = matrix_data['matrix_name']
                new_name = "{}_reduced_{}".format(old_name, frac)
                start_time = matrix_data['start_time']
                duration = matrix_data['duration']
                factor = matrix_data['factor']
                od_df = matrix_data['matrix']
                #print(od_df.head())
                new_od_df = od_df.copy()
                new_od_df.apply(reducer)
                #print(new_od_df.head())
                before += od_df.values.sum()
                b_fac = od_df.values.sum() * float(factor)/100.0
                before_factor += b_fac
                after += new_od_df.values.sum()
                a_fac = new_od_df.values.sum() * float(factor)/100.0
                after_factor += a_fac
                print("MoD requests at time {} : {} | sum {}".format(start_time, b_fac - a_fac, od_df.values.sum()))
                frac_to_matrix_list[frac].append( {'matrix_id' : -1, 'matrix_name' : new_name, 'start_time' : start_time, 'duration' : duration, 'factor' : factor, 'matrix' : new_od_df} )
            print("reduced od matrices by {} trips! (before: {} | after: {})".format(before - after, before, after))
            print("reduced od matrices with factor by {} trips! (before: {} | after: {})".format(before_factor - after_factor, before_factor, after_factor))
        self.frac_to_reduced_matrix_list = frac_to_matrix_list

    def store_data(self, new_network_name, base_path):
        network_path = os.path.join(base_path, "network")
        if not os.path.isdir(network_path):
            os.mkdir(network_path)
        rq_files_path = os.path.join(base_path, "request_files")
        if not os.path.isdir(rq_files_path):
            os.mkdir(rq_files_path)
        rq_files_path = os.path.join(rq_files_path, new_name)
        if not os.path.isdir(rq_files_path):
            os.mkdir(rq_files_path)
        rq_files_path = os.path.join(rq_files_path, "matched")
        if not os.path.isdir(rq_files_path):
            os.mkdir(rq_files_path)
        rq_files_path = os.path.join(rq_files_path, new_name)
        if not os.path.isdir(rq_files_path):
            os.mkdir(rq_files_path)
        self.full_network.storeNewFullNetwork(network_path, new_network_name)
        rq_base_name = "aimsun_matrix_id_{}".format(self.active_od_infos["matrix_id"])
        for frac_seed, rq_df in self.frac_seed_to_request_df.items():
            frac, seed = frac_seed
            file_name = rq_base_name + "_{}_{}.csv".format(frac, seed)
            rq_df.to_csv(os.path.join(rq_files_path, file_name), index = False)
        import pickle
        for frac, matrix_data in self.frac_to_reduced_matrix_list.items():
            file_name = os.path.join(base_path, "reduced_matrix_{}.pickle".format(frac))
            f = open(file_name, "wb")
            pickle.dump(matrix_data, f)
            f.close()
        

if __name__ == "__main__":
    aimsun_od_base = r'C:\Users\ge37ser\Documents\Coding\TUM_VT_FleetSimulation\tum-vt-fleet-simulation\data\networks\Aimsun_Munich_2020_04_15\aimsun_exports\data'
    files = ["aimsun_od_matrices_obj_6703597.pickle", "aimsun_od_matrices_obj_6703598.pickle", "aimsun_od_matrices_obj_6703599.pickle", "aimsun_od_matrices_obj_6703600.pickle", "aimsun_od_matrices_obj_6703601.pickle"]
    #aimsun_od_export = r"C:\Users\ge37ser\Documents\Coding\TUM_VT_FleetSimulation\tum-vt-fleet-simulation\data\demand\Aimsun_Munich_2020_04_15\aimsun_od_matrices_obj_6703532.pickle"
    aimsun_od_exp_list = [os.path.join(aimsun_od_base, f) for f in files]
    AL = AimsunODloader(aimsun_od_exp_list)
    AL.loadBMWoperatingAreaAndSetCentroidsWithinOA()
    nw_path = r'C:\Users\ge37ser\Documents\Coding\TUM_VT_FleetSimulation\tum-vt-fleet-simulation\data\networks\Aimsun_Munich_2020_04_15'
    AL.load_aimsun_exported_full_network(nw_path)
    AL.find_same_centroids()
    AL.create_centroid_connector_stop_nodes()

    demand_fractions = [1.0, 5.0, 10.0, 15.0, 25.0, 50.0, 75.0, 100.0]
    AL.createReducedAimsunODMatrix(demand_fractions_in_per=demand_fractions)

    AL.create_request_dfs_from_centroid_to_stop_nodes_allMatrices(demand_fractions_in_per=demand_fractions, number_seeds=5)
    # exit()
    # AL.selectODmatrix(2598)
    # AL.create_request_dfs_from_centroid_to_stop_nodes(demand_fractions_in_per=[1.0, 5.0, 10.0, 20.0, 30.0], number_seeds=3)

    output_path = r'C:\Users\ge37ser\Desktop\tmp'
    new_name = 'Aimsun_Munich_2020_04_15_not_preprocessed'
    AL.store_data(new_name, output_path)