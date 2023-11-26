import os
from shapely.geometry import Point, LineString, Polygon
import numpy as np
import pandas as pd
import geopandas as gpd


#NETWORK CLASSES FOR EXPORT

def convertCoordListToString(coord_list):
    l = []
    for coord in coord_list:
        l.append("{};{}".format(coord[0], coord[1]))
    return "|".join(l)

def convertStringToCoordList(string):
    coord_list = []
    for e in string.split("|"):
        x,y = e.split(";")
        coord_list.append( (float(x), float(y)) )
    return coord_list

def createDir(dirname):
    if os.path.isdir(dirname):
        print("{} allready here! Overwriting basic network files!".format(dirname))
    else:
        os.makedirs(dirname)

class NetworkAimsunConvertor():
    def __init__(self):
        self.nodes = []
        self.edge_id_to_edge = {}
        self.source_edge_id_to_edge = {}
        self.super_nodes = {}

    def addNode(self, node):
        self.nodes.append(node)
        if node.index != len(self.nodes) - 1:
            print("wrong node order!")
            exit()

    def addSuperNode(self, super_node):
        self.super_nodes[super_node.id] = super_node

    def getCurrentNodeNumber(self):
        return len(self.nodes)

    def addEdge(self, edge):
        start_index = edge.start_node_index
        end_index = edge.end_node_index
        print("edge {} -> {} nodes {}".format(start_index, end_index, len(self.nodes)))
        self.nodes[start_index].addOutgoingEdge(edge)
        self.nodes[end_index].addIncomingEdge(edge)
        self.edge_id_to_edge[edge.id] = edge
        if edge.source_edge_id is not None:
            self.source_edge_id_to_edge[edge.source_edge_id] = edge

    def exportNetworkFull(self, destination_folder, name):
        node_list = []
        for node in self.nodes:
            node_list.append(str(node))
        f = open(os.path.join(destination_folder, "{}_nodes.csv".format(name)), "w")
        f.write("\n".join(node_list))
        f.close()

        edge_list = []
        for edge in self.edge_id_to_edge.values():
            edge_list.append(str(edge))
        f = open(os.path.join(destination_folder, "{}_edges.csv".format(name)), "w")
        f.write("\n".join(edge_list))
        f.close()

        super_node_list = []
        for super_node in self.super_nodes.values():
            super_node_list.append(str(super_node))

        f = open(os.path.join(destination_folder, "{}_supernodes.csv".format(name)), "w")
        f.write("\n".join(super_node_list))
        f.close()

    def loadNetworkFromAimsunExport(self, base_folder, name):
        """ loads network files created by aimsun_converter.py
        same base_folder and name should be used in both scripts
        base_folder corresponds to the folder of aimsun_converter.py export files
        name defines the name of network later
        """
        node_file = open(os.path.join(base_folder, "{}_nodes.csv".format(name)), "r")
        ls = node_file.read()
        node_file.close()
        for l in ls.split("\n"):
            if not l:
                continue
            index, x, y, source_edge_id, is_stop_only, ch_val = l.split(",")
            if is_stop_only:
                is_stop_only = True
            else:
                is_stop_only = False
            node = NetworkNode(int(index), float(x), float(y), source_edge_id=source_edge_id, is_stop_only=is_stop_only, ch_value=int(ch_val))
            self.addNode(node)
        edge_file = open(os.path.join(base_folder, "{}_edges.csv".format(name)), "r")
        ls = edge_file.read()
        edge_file.close()
        for l in ls.split("\n"):
            if not l:
                continue
            start_node_index, end_node_index, travel_time, travel_distance, edge_id, source_edge_id, roadtype, polyline = l.split(",")
            polyline = convertStringToCoordList(polyline)
            edge = NetworkEdge(int(start_node_index), int(end_node_index), float(travel_time), float(travel_distance), edge_index=edge_id, source_edge_id=source_edge_id, roadtype=roadtype, polyline=polyline)
            self.addEdge(edge)

        supernode_file = open(os.path.join(base_folder, "{}_supernodes.csv".format(name)), "r")
        ls = supernode_file.read()
        for l in ls.split("\n"):
            if not l:
                continue
            sn_id, source_edge_id, node_collection, polygon = l.split(",")
            node_collection = [int(n) for n in node_collection.split(";")]
            polygon = convertStringToCoordList(polygon)
            supernode = NetworkSuperNode(sn_id, node_collection, polygon=polygon, source_edge_id=source_edge_id)
            self.addSuperNode(supernode)

    def plotNetwork(self):
        from matplotlib import pyplot as plt

        x = [node.coordinates[0] for node in self.nodes]
        y = [node.coordinates[1] for node in self.nodes]

        plt.plot(x, y, "rx")

        for arc in self.edge_id_to_edge.values():
            plt.plot([x[0] for x in arc.polyline], [y[1] for y in arc.polyline], "b-")

        for supernode in self.super_nodes.values():
            x = [supernode.polygon[i%len(supernode.polygon)][0] for i in range(len(supernode.polygon) + 1)]
            y = [supernode.polygon[i%len(supernode.polygon)][1] for i in range(len(supernode.polygon) + 1)]
            plt.plot(x, y, "g-")

        plt.show()

    def convertFullInformationToGeoJSON(self, path):
        node_gpd_list = []
        for node in self.nodes:
            node_gpd_list.append(node.getAttributeDictGEOJSON())
        node_gpd = gpd.GeoDataFrame(node_gpd_list)
        node_gpd.to_file(os.path.join(path, "nodes_all_infos.geojson"), driver="GeoJSON")

        edge_gpd_list = []
        for edge in self.edge_id_to_edge.values():
            edge_gpd_list.append(edge.getAttributeDictGEOJSON())
        edge_gpd = gpd.GeoDataFrame(edge_gpd_list)
        edge_gpd.to_file(os.path.join(path, "edges_all_infos.geojson"), driver="GeoJSON")

        supernode_gpd_list = []
        for supernode in self.super_nodes.values():
            supernode_gpd_list.append(supernode.getAttributeDictGEOJSON())
        supernode_gpd = gpd.GeoDataFrame(supernode_gpd_list)
        # print(supernode_gpd.head())
        # print(supernode_gpd.to_json())
        supernode_gpd.to_file(os.path.join(path, "supernodes_all_infos.geojson"), driver="GeoJSON")

    def convertBaseInformationToCSV(self, path):
        node_pd_list = []
        for node in self.nodes:
            node_pd_list.append(node.getAttributeDictBaseFile())
        node_pd = pd.DataFrame(node_pd_list)
        node_pd.to_csv(os.path.join(path, "nodes.csv"), index=False)

        edge_pd_list = []
        for edge in self.edge_id_to_edge.values():
            edge_pd_list.append( edge.getAttributeDictBaseFile() )
        edge_pd = pd.DataFrame(edge_pd_list)
        edge_pd.to_csv(os.path.join(path, "edges.csv"), index=False)
        


class NetworkNode():
    def __init__(self, node_index, pos_x, pos_y, source_edge_id = None, is_stop_only = False, ch_value = -1):
        self.index = node_index
        self.coordinates = (pos_x, pos_y)
        self.is_stop_only = is_stop_only
        self.ch_value = ch_value
        self.source_edge_id = source_edge_id

        self.outgoing_edges = {}
        self.incoming_edges = {}

        self.outgoing_ch_edges = {}
        self.incoming_ch_edges = {}

    def addIncomingEdge(self, edge):
        self.incoming_edges[edge.id] = edge

    def addOutgoingEdge(self, edge):
        self.outgoing_edges[edge.id] = edge

    def __str__(self):
        s = "{},{},{}".format(self.index, self.coordinates[0], self.coordinates[1])
        if self.is_stop:
            s += ",{}".format(1)
        else:
            s += ","
        s += ",{}".format(self.ch_value)
        return s

    def getAttributeDictGEOJSON(self):
        att_dict = {"node_index" : self.index, "is_stop_only" : self.is_stop_only, "source_edge_id" : self.source_edge_id, "geometry" : Point(self.coordinates)}
        return att_dict

    def getAttributeDictBaseFile(self):
        att_dict = {"node_index" : self.index, "is_stop_only" : self.is_stop_only, "pos_x" : self.coordinates[0], "pos_y" : self.coordinates[1] }
        return att_dict

class NetworkSuperNode():
    def __init__(self, id, node_id_list, polygon = None, source_edge_id = None):
        self.node_collection = node_id_list[:]
        self.id = id
        self.polygon = polygon
        self.source_edge_id = source_edge_id

    def __str__(self):
        return "{},{},{},{}".format(self.id, self.source_edge_id, ";".join([str(x) for x in self.node_collection]), convertCoordListToString(self.polygon))

    def getAttributeDictGEOJSON(self):
        att_dict = {"index" : self.id, "node_collection" : ";".join([str(x) for x in self.node_collection]), "source_edge_id" : self.source_edge_id, "geometry" : Polygon(self.polygon) }
        #print(type(np.array(self.node_collection, dtype='int64')))
        return att_dict

class NetworkEdge():
    def __init__(self, start_node_index, end_node_index, travel_time, travel_distance, edge_index = None, source_edge_id = None, shortcut_def = None, customized_cost_val = None, polyline = None, roadtype = None):
        self.start_node_index = start_node_index
        self.start_node = None
        self.end_node_index = end_node_index
        self.end_node = None

        self.travel_time = travel_time
        self.travel_distance = travel_distance

        self.customized_cost_val = customized_cost_val

        self.id = edge_index
        if self.id is None:
            self.id = "{};{}".format(self.start_node_index, self.end_node_index)
        
        self.source_edge_id = source_edge_id
        self.shortcut_def = shortcut_def

        self.polyline = polyline
        #print(self.polyline)
        self.roadtype = roadtype

    def getStartNodeIndex(self):
        return self.start_node_index

    def getEndNodeIndex(self):
        return self.end_node_index

    def __str__(self):
        s = "{},{},{},{},{},{},{},{}".format(self.start_node_index, self.end_node_index, self.travel_time, self.travel_distance, self.id, self.source_edge_id,self.roadtype,convertCoordListToString(self.polyline))
        return s

    def getAttributeDictGEOJSON(self):
        att_dict = {"from_node" : self.start_node_index, "to_node" : self.end_node_index, "distance" : self.travel_distance, "travel_time" : self.travel_time, "source_edge_id" : self.source_edge_id, "road_type" : self.roadtype, "geometry" : LineString(self.polyline)}
        return att_dict

    def getAttributeDictBaseFile(self):
        att_dict = {"from_node" : self.start_node_index, "to_node" : self.end_node_index, "distance" : self.travel_distance, "travel_time" : self.travel_time, "source_edge_id" : self.source_edge_id}
        return att_dict



#------------------------------------------------------------------------------------------------------------#
# MAIN FUNCTION #
#------------------------------------------------------------------------------------------------------------#
def createNetwork(aimsun_files_folder, target_network_path, network_name):
    ''' 
    reads export files from aimsun_files_folder and looks for files "{}_nodes/edges/supernodes.csv".format(network_name)
    network_name should allready be defined in the aimsun extraction script aimsun_export_network.py
    creates a unified network structure in target_network_path with name network_name
    only creates the base folder (static network with free flow)
    '''
    nw = NetworkAimsunConvertor()
    nw.loadNetworkFromAimsunExport(aimsun_files_folder, network_name)
    new_network_folder = os.path.join(target_network_path, network_name)
    createDir(new_network_folder)
    base_folder = os.path.join(new_network_folder, "base")
    createDir(base_folder)
    nw.convertBaseInformationToCSV(base_folder)
    nw.convertFullInformationToGeoJSON(base_folder)




if __name__ == "__main__":
    output_folder = r"C:\Users\ge37ser\Documents\Coding\TUM_VT_FleetSimulation\tum-vt-fleet-simulation\data\networks\Aimsun_Munich_2020_04_15"
    name = "aimsun_exports"
    createNetwork(output_folder, r"C:\Users\ge37ser\Documents\Coding\TUM_VT_FleetSimulation\tum-vt-fleet-simulation\data\networks\Aimsun_Munich_2020_04_15", name)