import os
from shapely.geometry import Point, LineString, Polygon
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import geopandas as gpd


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

class NetworkNode():
    def __init__(self, node_index, pos_x, pos_y, is_stop_only = False, source_edge_id=None, ch_value = -1):
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
        att_dict = {"node_index" : self.index, "is_stop_only" : self.is_stop_only, "geometry" : Point(self.coordinates)}
        return att_dict

    def getAttributeDictBaseFile(self):
        att_dict = {"node_index" : self.index, "is_stop_only" : self.is_stop_only, "pos_x" : self.coordinates[0], "pos_y" : self.coordinates[1], "source_edge_id" : self.source_edge_id }
        return att_dict

class NetworkEdge():
    def __init__(self, start_node_index, end_node_index, travel_time, travel_distance, edge_index = None, source_edge_id = None, shortcut_def = None, customized_cost_val = None, polyline = None, roadtype = None, time_dependent_travel_times = {}):
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

        self.time_dependent_travel_times = {}   # start_time -> travel time
        for t, tt in time_dependent_travel_times.items():
            self.time_dependent_travel_times[t] = tt

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


class NetworkCreator():
    def __init__(self, crs = None):
        self.nodes = []
        self.edge_id_to_edge = {}
        self.source_edge_id_to_edge = {}
        self.super_nodes = {}
        self.registered_travel_times = []
        self.crs = crs

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
        #print("edge {} -> {} nodes {}".format(start_index, end_index, len(self.nodes)))
        self.nodes[start_index].addOutgoingEdge(edge)
        self.nodes[end_index].addIncomingEdge(edge)
        self.edge_id_to_edge[edge.id] = edge
        if edge.source_edge_id is not None:
            self.source_edge_id_to_edge[edge.source_edge_id] = edge

    def registerTravelTimes(self, start_time_list):
        self.registered_travel_times = start_time_list

    def convertFullInformationToGeoJSON(self, path):
        node_gpd_list = []
        for node in self.nodes:
            node_gpd_list.append(node.getAttributeDictGEOJSON())
        if self.crs is None:
            node_gpd = gpd.GeoDataFrame(node_gpd_list)
        else:
            node_gpd = gpd.GeoDataFrame(node_gpd_list, crs = self.crs)
        node_gpd.to_file(os.path.join(path, "nodes_all_infos.geojson"), driver="GeoJSON")

        edge_gpd_list = []
        for edge in self.edge_id_to_edge.values():
            edge_gpd_list.append(edge.getAttributeDictGEOJSON())
        if self.crs is None:
            edge_gpd = gpd.GeoDataFrame(edge_gpd_list)
        else:
            edge_gpd = gpd.GeoDataFrame(edge_gpd_list, crs = self.crs)
        edge_gpd.to_file(os.path.join(path, "edges_all_infos.geojson"), driver="GeoJSON")

        supernode_gpd_list = []
        for supernode in self.super_nodes.values():
            supernode_gpd_list.append(supernode.getAttributeDictGEOJSON())
        if len(supernode_gpd_list) > 0:
            if self.crs is None:
                supernode_gpd = gpd.GeoDataFrame(supernode_gpd_list)
            else:
                supernode_gpd = gpd.GeoDataFrame(supernode_gpd_list, crs = self.crs)
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

    def plotNetwork(self):
        node_gpd_list = []
        for node in self.nodes:
            node_gpd_list.append(node.getAttributeDictGEOJSON())
        node_gpd = gpd.GeoDataFrame(node_gpd_list)

        edge_gpd_list = []
        for edge in self.edge_id_to_edge.values():
            edge_gpd_list.append(edge.getAttributeDictGEOJSON())
        edge_gpd = gpd.GeoDataFrame(edge_gpd_list)

        ax = node_gpd.plot(color = "r")
        edge_gpd.plot(ax = ax)
        plt.show()

    def createNetworkFiles(self, path, network_name):
        network_folder = os.path.join(path, network_name)
        createDir(network_folder)
        base_dir = os.path.join(network_folder, "base")
        createDir(base_dir)
        print(base_dir)
        self.convertBaseInformationToCSV(base_dir)
        self.convertFullInformationToGeoJSON(base_dir)
        for tt in self.registered_travel_times:
            print("create traveltime files", tt)
            self.createTTFiles(network_folder, tt)

    def createTTFiles(self, network_folder, tt):
        if not os.path.isdir(os.path.join(network_folder, str(tt))):
            os.mkdir(os.path.join(network_folder, str(tt)))
        edge_tt_df_list = []
        tt_folder = os.path.join(network_folder, str(tt))
        for edge in self.edge_id_to_edge.values():
            tt_tt = edge.time_dependent_travel_times.get(tt)
            if tt is None:
                tt_tt = edge.travel_time
            edge_tt_df_list.append({"from_node" : edge.start_node_index, "to_node" : edge.end_node_index, "edge_tt" : tt_tt})
        edge_tt_df = pd.DataFrame(edge_tt_df_list)
        edge_tt_df.to_csv(os.path.join(tt_folder, "edges_td_att.csv"), index=False)

def readVissimNetworkFileAndCreateNetwork(path, outpath, network_name):
    moia_allowed_links = {}
    visum_nodes = {}    # node_id -> (x, y)
    visum_links = {}    # (start_node_id, end_node_id) -> (source_edge_id, length, tt)
    visum_turns = {}    # (start_node_id, mid_node_id, end_node_id) -> tt
    with open(path, "r") as f:
        l = f.readline()
        c = 0
        current_infos = 0
        current_info = None
        current_info_entries = None
        current_info_entry_dict = {}   #entry id -> index
        while l:
            c+=1
            if l[0] == "\n" or l.startswith("*"):
                pass
            else:
                if l[0] == '$':
                    print("new infos!")
                    print(l)
                    current_infos += 1
                    current_info = l.split(":")[0]
                    current_info = current_info[1:]
                    print(current_info)
                    try:
                        current_info_entries = l.split(":")[1]
                        current_info_entries = current_info_entries[:-1]
                        current_info_entries = current_info_entries.split(";")
                    except:
                        current_info_entries = None
                    if current_info_entries is not None:
                        current_info_entry_dict = {x : y for y, x in enumerate(current_info_entries)}
                        if current_info == 'LINK':
                            if current_info_entry_dict.get('V0_PRTSYS(C)'):
                                current_info_entry_dict['FreeFlowVel'] = current_info_entry_dict['V0_PRTSYS(C)']
                            elif current_info_entry_dict.get('V0PRT'):
                                current_info_entry_dict['FreeFlowVel'] = current_info_entry_dict['V0PRT']
                            else:
                                print("free flow attribute for LINK is not defined for this file!")
                                print(current_info_entry_dict)
                                exit()
                    else:
                        current_info_entry_dict = {}
                    print(current_info_entry_dict)
                elif current_info == "NODE":
                    l = l[:-1]
                    entries = l.split(";")
                    id_entry = current_info_entry_dict['NO'] # current_info_entries.index('NO')
                    x_entry = current_info_entry_dict['XCOORD']# current_info_entries.index('XCOORD')
                    y_entry = current_info_entry_dict['YCOORD'] # current_info_entries.index('YCOORD')
                    #print(id_entry, x_entry, y_entry)
                    source_id = int(entries[id_entry])
                    x = float(entries[x_entry])
                    y = float(entries[y_entry])
                    visum_nodes[source_id] = (x, y)
                elif current_info == "LINK":
                    #print(l)
                    l = l[:-1]
                    entries = l.split(";")
                    # print(current_info_entry_dict)
                    # print(entries)
                    # print(l)
                    # print(len(current_info_entry_dict.keys()))
                    # print(len(entries))
                    # exit()
                    #print(entries)
                    type_no = entries[current_info_entry_dict['TYPENO']]
                    source_id = entries[current_info_entry_dict['NO']]
                    from_node = int(entries[current_info_entry_dict['FROMNODENO']])
                    to_node = int(entries[current_info_entry_dict['TONODENO']])
                    dis = entries[current_info_entry_dict['LENGTH']]
                    dis = float(dis.split("km")[0])*1000.0
                    v = entries[current_info_entry_dict['FreeFlowVel']]
                    v = float(v.split("km/h")[0])/3.6
                    try:
                        tt0 = dis/v
                    except:
                        #print("Zero Length!")
                        tt0 = 0.0
                    append_feasible = True
                    if not moia_allowed_links[type_no]:
                        print("not allowed for moia", from_node, to_node)
                        append_feasible = False
                    if from_node == to_node:
                        #print("self connected! ")
                        append_feasible = False

                    # tt0 = entries[12]
                    # tt0 = float(tt0.split("s")[0])
                    # try:
                    #     v = dis/tt0 * 3.6
                    # except:
                    #     print("Zero Length")
                    if append_feasible:
                        visum_links[(from_node, to_node)] = (source_id, dis, tt0)
                    #break
                elif current_info == "LINKTYPE":
                    l = l[:-1]
                    entries = l.split(";")
                    type_no = entries[current_info_entry_dict['NO']]
                    allowed_veh_types = entries[current_info_entry_dict['TSYSSET']].split(",")
                    moia_allowed = False
                    if "C" in allowed_veh_types:
                        moia_allowed = True
                    moia_allowed_links[type_no] = moia_allowed
                elif current_info == "TURN":
                    l = l[:-1]
                    entries = l.split(";")
                    from_node = int(entries[current_info_entry_dict['FROMNODENO']])
                    via_node = int(entries[current_info_entry_dict['VIANODENO']])
                    to_node = int(entries[current_info_entry_dict['TONODENO']])
                    tt = 0
                    allowed_veh_types = entries[current_info_entry_dict['TSYSSET']].split(",")
                    if "C" in allowed_veh_types:
                        visum_turns[(from_node, via_node, to_node)] = tt

                    
            l = f.readline()

    print("number visum nodes: ", len(visum_nodes.keys()))
    print("number visum links: ", len(visum_links.keys()))
    print("number visum turn: ", len(visum_turns.keys()))

    nw = NetworkCreator()
    nodes = {}
    base_edges = {}
    node_source_id_to_index = {}
    node_source_id_to_index_list = []
    # node_id -> (x, y)
    c = 0
    for source_node_id, coords in visum_nodes.items():
        node = NetworkNode(c, coords[0], coords[1], is_stop_only=True, source_edge_id=source_node_id)
        node_source_id_to_index[source_node_id] = c
        node_source_id_to_index_list.append( {"visum_node_id" : source_node_id, "node_index" : c})
        c += 1
        nodes[source_node_id] = node
        nw.addNode(node)

    edge_source_id_to_index_tuple = {}
    edge_source_id_to_index_tuple_list = []
    # (start_node_id, end_node_id) -> (source_edge_id, length, tt)
    for key, vals in visum_links.items():
        start_node, end_node = key
        source_edge_id, length, tt = vals
        created_start_node = nodes[start_node]
        created_end_node = nodes[end_node]
        s_node = NetworkNode(c, created_start_node.coordinates[0], created_start_node.coordinates[1], source_edge_id="{}_s".format(source_edge_id))
        nw.addNode(s_node)
        c += 1
        e_node = NetworkNode(c, created_end_node.coordinates[0], created_end_node.coordinates[1], source_edge_id="{}_e".format(source_edge_id))
        nw.addNode(e_node)
        c += 1
        nodes["{}_s".format(source_edge_id)] = s_node
        nodes["{}_e".format(source_edge_id)] = e_node

        base_edge = NetworkEdge(s_node.index, e_node.index, tt, length, source_edge_id=source_edge_id, polyline=LineString( [s_node.coordinates, e_node.coordinates]))
        base_edges[(start_node, end_node)] = base_edge
        con_edge_start = NetworkEdge(created_start_node.index, s_node.index, 0, 0, polyline=LineString( [created_start_node.coordinates, s_node.coordinates]) )
        con_edge_end = NetworkEdge(e_node.index, created_end_node.index, 0, 0, polyline=LineString( [e_node.coordinates, created_end_node.coordinates]) )
        nw.addEdge(base_edge)
        nw.addEdge(con_edge_start)
        nw.addEdge(con_edge_end)
        edge_source_id_to_index_tuple[source_edge_id] = base_edge.id
        edge_source_id_to_index_tuple_list.append( {"visum_edge_id" : source_edge_id, "edge_id" : base_edge.id})

    # (start_node_id, mid_node_id, end_node_id) -> tt
    not_found = 0
    for turn_key, tt in visum_turns.items():
        start, via, end = turn_key
        try:
            start_edge = base_edges[(start, via)]
        except KeyError:
            not_found += 1
            continue
        try:
            end_edge = base_edges[(via, end)]
        except KeyError:
            not_found += 1
            continue
        start_node = nw.nodes[start_edge.end_node_index]
        end_node = nw.nodes[end_edge.start_node_index]
        turn_edge = NetworkEdge(start_node.index, end_node.index, tt, 0, source_edge_id=";".join([str(x) for x in turn_key]), polyline=LineString( [start_node.coordinates, end_node.coordinates]))
        nw.addEdge(turn_edge)
    print("not found turn combinations: {}/{}".format(not_found, len(visum_turns.keys())))
    print("number nodes found: {}".format(len(nw.nodes)))
    print("number edges found: {}".format(len(nw.edge_id_to_edge.keys())))

    nw.createNetworkFiles(outpath, network_name)
    node_source_id_to_index_df = pd.DataFrame(node_source_id_to_index_list)
    node_source_id_to_index_df.to_csv(os.path.join(outpath,network_name, "visum_node_id_to_node_index.csv"), index=False)
    edge_source_id_to_index_tuple_df = pd.DataFrame(edge_source_id_to_index_tuple_list)
    edge_source_id_to_index_tuple_df.to_csv(os.path.join(outpath,network_name, "visum_edge_id_to_edge_index.csv"), index=False)

    nw.plotNetwork()

def readVissimNetworkFileAndCreateNetworkWithUmlegung(path, outpath, network_name, travel_time_interval_translator):
    #travel time translator: travel time column name -> tt folder name
    MOIA_MARKER  = "M"  # straßen für moia
    moia_allowed_links = {}
    #TCUR_IV_M_NACHM;TCUR_IV_M_NACHT;TCUR_IV_M_REST;TCUR_IV_M_TAG;TCUR_IV_M_VORM;VCUR_IV_M_NACHM;VCUR_IV_M_NACHT;VCUR_IV_M_REST;VCUR_IV_M_TAG;VCUR_IV_M_VORM
    visum_nodes = {}    # node_id -> (x, y)
    visum_links = {}    # (start_node_id, end_node_id) -> (source_edge_id, length, tt_wholeday)
    visum_turns = {}    # (start_node_id, mid_node_id, end_node_id) -> tt_wholeday
    visum_links_tt = {} # (start_node_id, end_node_id) -> start_time -> tt
    visum_turns_tt = {} # (start_node_id, mid_node_id, end_node_id) -> start_time -> tt
    with open(path, "r") as f:
        l = f.readline()
        c = 0
        current_infos = 0
        current_info = None
        current_info_entries = None
        current_info_entry_dict = {}   #entry id -> index
        while l:
            c+=1
            if l[0] == "\n" or l.startswith("*"):
                pass
            else:
                if l[0] == '$':
                    print("new infos!")
                    print(l)
                    current_infos += 1
                    current_info = l.split(":")[0]
                    current_info = current_info[1:]
                    print(current_info)
                    try:
                        current_info_entries = l.split(":")[1]
                        current_info_entries = current_info_entries[:-1]
                        current_info_entries = current_info_entries.split(";")
                    except:
                        current_info_entries = None
                    if current_info_entries is not None:
                        current_info_entry_dict = {x : y for y, x in enumerate(current_info_entries)}
                        # if current_info == 'LINK':
                        #     if current_info_entry_dict.get('V0_PRTSYS(C)'):
                        #         current_info_entry_dict['FreeFlowVel'] = current_info_entry_dict['V0_PRTSYS(C)']
                        #     elif current_info_entry_dict.get('V0PRT'):
                        #         current_info_entry_dict['FreeFlowVel'] = current_info_entry_dict['V0PRT']
                        #     else:
                        #         print("free flow attribute for LINK is not defined for this file!")
                        #         print(current_info_entry_dict)
                        #         exit()
                    else:
                        current_info_entry_dict = {}
                    print(current_info_entry_dict)
                elif current_info == "NODE":
                    l = l[:-1]
                    entries = l.split(";")
                    id_entry = current_info_entry_dict['NO'] # current_info_entries.index('NO')
                    x_entry = current_info_entry_dict['XCOORD']# current_info_entries.index('XCOORD')
                    y_entry = current_info_entry_dict['YCOORD'] # current_info_entries.index('YCOORD')
                    #print(id_entry, x_entry, y_entry)
                    source_id = int(entries[id_entry])
                    x = float(entries[x_entry])
                    y = float(entries[y_entry])
                    visum_nodes[source_id] = (x, y)
                elif current_info == "LINK":
                    #print(l)
                    l = l[:-1]
                    entries = l.split(";")

                    allowed_veh_types = entries[current_info_entry_dict['TSYSSET']].split(",")
                    if not MOIA_MARKER in allowed_veh_types:
                        pass
                        #print("not allowed for moia", entries)
                    else:
                        source_id = entries[current_info_entry_dict['NO']]
                        from_node = int(entries[current_info_entry_dict['FROMNODENO']])
                        to_node = int(entries[current_info_entry_dict['TONODENO']])
                        dis = entries[current_info_entry_dict['LENGTHPOLY']]
                        dis = float(dis.split("km")[0])*1000.0
                        try:
                            tt0 = float(entries[current_info_entry_dict["TCUR_IV_M_TAG"]])
                        except:
                            tt0 = float(entries[current_info_entry_dict["TCUR_IV_M_WD_0_1"]])
                        append_feasible = True
                        if from_node == to_node:
                            print("self connected! ")
                            #append_feasible = False
                        if append_feasible:
                            visum_links[(from_node, to_node)] = (source_id, dis, tt0)
                            for key, start_times in travel_time_interval_translator.items():
                                tt = float(entries[current_info_entry_dict[key]])
                                for t in start_times:
                                    try:
                                        visum_links_tt[(from_node, to_node)][t] = tt
                                    except KeyError:
                                        visum_links_tt[(from_node, to_node)] = {t : tt}
                elif current_info == "TURN":
                    l = l[:-1]
                    #FROMNODENO;VIANODENO;TONODENO;TSYSSET;TCUR_IV_M_NACHM;TCUR_IV_M_NACHT;TCUR_IV_M_REST;TCUR_IV_M_TAG;TCUR_IV_M_VORM
                    entries = l.split(";")
                    allowed_veh_types = entries[current_info_entry_dict['TSYSSET']].split(",")
                    if MOIA_MARKER in allowed_veh_types:
                        from_node = int(entries[current_info_entry_dict['FROMNODENO']])
                        via_node = int(entries[current_info_entry_dict['VIANODENO']])
                        to_node = int(entries[current_info_entry_dict['TONODENO']])
                        try:
                            tt0 = float(entries[current_info_entry_dict["TCUR_IV_M_TAG"]])
                        except:
                            tt0 = float(entries[current_info_entry_dict["TCUR_IV_M_WD_0_1"]])
                        if tt0 == 0:
                            tt0 = 0.1
                        visum_turns[(from_node, via_node, to_node)] = tt0
                        for key, start_times in travel_time_interval_translator.items():
                            tt = float(entries[current_info_entry_dict[key]])
                            if tt == 0:
                                tt = 0.1
                            for t in start_times:
                                try:
                                    visum_turns_tt[(from_node, via_node, to_node)][t] = tt
                                except KeyError:
                                    visum_turns_tt[(from_node, via_node, to_node)] = {t : tt}

            l = f.readline()

    print("number visum nodes: ", len(visum_nodes.keys()))
    print("number visum links: ", len(visum_links.keys()))
    print("number visum turn: ", len(visum_turns.keys()))

    nw = NetworkCreator(crs="EPSG:32632")
    travel_times = []
    for tt_list in travel_time_interval_translator.values():
        travel_times += tt_list
    nw.registerTravelTimes(travel_times)
    nodes = {}
    base_edges = {}
    node_source_id_to_index = {}
    node_source_id_to_index_list = []
    # node_id -> (x, y)
    c = 0
    for source_node_id, coords in visum_nodes.items():
        node = NetworkNode(c, coords[0], coords[1], is_stop_only=True, source_edge_id=source_node_id)
        node_source_id_to_index[source_node_id] = c
        node_source_id_to_index_list.append( {"visum_node_id" : source_node_id, "node_index" : c})
        c += 1
        nodes[source_node_id] = node
        nw.addNode(node)

    edge_source_id_to_index_tuple = {}
    edge_source_id_to_index_tuple_list = []
    # (start_node_id, end_node_id) -> (source_edge_id, length, tt)
    for key, vals in visum_links.items():
        start_node, end_node = key
        source_edge_id, length, tt = vals
        created_start_node = nodes[start_node]
        created_end_node = nodes[end_node]
        s_node = NetworkNode(c, created_start_node.coordinates[0], created_start_node.coordinates[1], source_edge_id="{}_s".format(source_edge_id))
        nw.addNode(s_node)
        c += 1
        e_node = NetworkNode(c, created_end_node.coordinates[0], created_end_node.coordinates[1], source_edge_id="{}_e".format(source_edge_id))
        nw.addNode(e_node)
        c += 1
        nodes["{}_s".format(source_edge_id)] = s_node
        nodes["{}_e".format(source_edge_id)] = e_node

        time_dependent_tts = visum_links_tt[key].copy()
        base_edge = NetworkEdge(s_node.index, e_node.index, tt, length, source_edge_id=source_edge_id, polyline=LineString( [s_node.coordinates, e_node.coordinates]), roadtype="LINK", time_dependent_travel_times=time_dependent_tts)
        base_edges[(start_node, end_node)] = base_edge
        con_time_dependent_tts = {t : 0 for t in time_dependent_tts.keys()}
        con_edge_start = NetworkEdge(created_start_node.index, s_node.index, 0, 0, polyline=LineString( [created_start_node.coordinates, s_node.coordinates]), roadtype="STOPCONNECTOR", time_dependent_travel_times=con_time_dependent_tts )
        con_edge_end = NetworkEdge(e_node.index, created_end_node.index, 0, 0, polyline=LineString( [e_node.coordinates, created_end_node.coordinates]), roadtype="STOPCONNECTOR", time_dependent_travel_times=con_time_dependent_tts )
        nw.addEdge(base_edge)
        nw.addEdge(con_edge_start)
        nw.addEdge(con_edge_end)
        edge_source_id_to_index_tuple[source_edge_id] = base_edge.id
        edge_source_id_to_index_tuple_list.append( {"visum_edge_id" : source_edge_id, "edge_id" : base_edge.id, "visum_start_node_id" : start_node, "visum_end_node_id" : end_node})

    # (start_node_id, mid_node_id, end_node_id) -> tt
    not_found = 0
    for turn_key, tt in visum_turns.items():
        start, via, end = turn_key
        try:
            start_edge = base_edges[(start, via)]
        except KeyError:
            not_found += 1
            continue
        try:
            end_edge = base_edges[(via, end)]
        except KeyError:
            not_found += 1
            # print("b", start, via, end)
            # exit()
            continue
        start_node = nw.nodes[start_edge.end_node_index]
        end_node = nw.nodes[end_edge.start_node_index]
        time_dependent_tts = visum_turns_tt[turn_key].copy()
        turn_edge = NetworkEdge(start_node.index, end_node.index, tt, 0, source_edge_id=";".join([str(x) for x in turn_key]), polyline=LineString( [start_node.coordinates, end_node.coordinates]), roadtype="TURN", time_dependent_travel_times=time_dependent_tts)
        nw.addEdge(turn_edge)
    print("not found turn combinations: {}/{}".format(not_found, len(visum_turns.keys())))
    print("number nodes found: {}".format(len(nw.nodes)))
    print("number edges found: {}".format(len(nw.edge_id_to_edge.keys())))

    nw.createNetworkFiles(outpath, network_name)
    node_source_id_to_index_df = pd.DataFrame(node_source_id_to_index_list)
    node_source_id_to_index_df.to_csv(os.path.join(outpath,network_name, "visum_node_id_to_node_index.csv"), index=False)
    edge_source_id_to_index_tuple_df = pd.DataFrame(edge_source_id_to_index_tuple_list)
    edge_source_id_to_index_tuple_df.to_csv(os.path.join(outpath,network_name, "visum_edge_id_to_edge_index.csv"), index=False)

    #nw.plotNetwork()   


def readZones(path, outpath, zone_name):
    #travel time translator: travel time column name -> tt folder name
    zone_id_to_attribute_list = []
    with open(path, "r") as f:
        l = f.readline()
        c = 0
        current_infos = 0
        current_info = None
        current_info_entries = None
        current_info_entry_dict = {}   #entry id -> index
        while l:
            c+=1
            if l[0] == "\n" or l.startswith("*"):
                pass
            else:
                if l[0] == '$':
                    print("new infos!")
                    print(l)
                    current_infos += 1
                    current_info = l.split(":")[0]
                    current_info = current_info[1:]
                    print(current_info)
                    try:
                        current_info_entries = l.split(":")[1]
                        current_info_entries = current_info_entries[:-1]
                        current_info_entries = current_info_entries.split(";")
                    except:
                        current_info_entries = None
                    if current_info_entries is not None:
                        current_info_entry_dict = {x : y for y, x in enumerate(current_info_entries)}
                    else:
                        current_info_entry_dict = {}
                    print(current_info_entry_dict)
                elif current_info == "ZONE":
                    l = l[:-1]
                    entries = l.split(";")
                    print(entries)
                    id_entry = entries[current_info_entry_dict['NO']] # current_info_entries.index('NO')
                    x_entry = entries[current_info_entry_dict['XCOORD']]# current_info_entries.index('XCOORD')
                    y_entry = entries[current_info_entry_dict['YCOORD']] # current_info_entries.index('YCOORD')
                    #print(id_entry, x_entry, y_entry)
                    surface_id = entries[current_info_entry_dict['SURFACEID']]
                    zone_dict = {
                        "zone_id" : int(id_entry),
                        "geometry" : Point(float(x_entry), float(y_entry))
                    }
                    try:
                        zone_dict["surface_id"] = int(surface_id)
                    except:
                        pass
                    zone_id_to_attribute_list.append(zone_dict)
            l = f.readline()
    zone_centroid_gpd = gpd.GeoDataFrame(zone_id_to_attribute_list, crs="EPSG:32632")
    if not os.path.isdir(os.path.join(outpath, zone_name)):
        os.mkdir(os.path.join(outpath, zone_name))
    zone_centroid_gpd.to_file(os.path.join(outpath, zone_name, "zone_centroid_information.geojson"), driver="GeoJSON")


if __name__ == "__main__":
    # p = r'C:\Users\ge37ser\Documents\Projekte\MOIA\Code\Netzwerk\26032020\medium_network.net'
    # outpath = r'C:\Users\ge37ser\Documents\Coding\commonNetworkStructure\networks_git_rep\networks_git\networks'
    # readVissimNetworkFileAndCreateNetwork(p, outpath, "MoiaHamburg08042020")

    import os
    dirname = os.path.dirname(__file__)

    #os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..', 'templates'))
    base_data_folder = os.path.abspath(os.path.join('.'))
    nw_data_folder = os.path.join(base_data_folder, 'data','networks')
    zone_data_folder = os.path.join(base_data_folder, "data", "zones")
    print(nw_data_folder)

    if False:
        travel_time_interval_translator = {
            "TCUR_IV_M_NACHM" : [15*3600],
            "TCUR_IV_M_NACHT" : [0*3600, 22*3600],
            "TCUR_IV_M_REST" : [10*3600, 19*3600],
            "TCUR_IV_M_VORM" : [6*3600]
        }

        visum_net_p = r'C:\Users\ge37ser\Documents\Coding\TUM_VT_FleetSimulation\tum-vt-fleet-simulation\data\networks\MOIA_HH_21122020_raw\netz.net'
        readVissimNetworkFileAndCreateNetworkWithUmlegung(visum_net_p, nw_data_folder, "MOIA_HH_21122020_raw")

    if True:
        x = "TCUR_IV_M_WD_0_1;BEL_IV_C_WD_0_1;TCUR_IV_M_WD_2_3;BEL_IV_C_WD_2_3;TCUR_IV_M_WD_4_5;BEL_IV_C_WD_4_5;TCUR_IV_M_WD_6_7;BEL_IV_C_WD_6_7;TCUR_IV_M_WD_8_9;BEL_IV_C_WD_8_9;TCUR_IV_M_WD_10_11;BEL_IV_C_WD_10_11;TCUR_IV_M_WD_12_13;BEL_IV_C_WD_12_13;TCUR_IV_M_WD_14_15;BEL_IV_C_WD_14_15;TCUR_IV_M_WD_16_17;BEL_IV_C_WD_16_17;TCUR_IV_M_WD_18_19;BEL_IV_C_WD_18_19;TCUR_IV_M_WD_20_21;BEL_IV_C_WD_20_21;TCUR_IV_M_WD_22_23;BEL_IV_C_WD_22_23;TCUR_IV_M_SA_0_1;BEL_IV_C_SA_0_1;TCUR_IV_M_SA_2_3;BEL_IV_C_SA_2_3;TCUR_IV_M_SA_4_5;BEL_IV_C_SA_4_5;TCUR_IV_M_SA_6_7;BEL_IV_C_SA_6_7;TCUR_IV_M_SA_8_9;BEL_IV_C_SA_8_9;TCUR_IV_M_SA_10_11;BEL_IV_C_SA_10_11;TCUR_IV_M_SA_12_13;BEL_IV_C_SA_12_13;TCUR_IV_M_SA_14_15;BEL_IV_C_SA_14_15;TCUR_IV_M_SA_16_17;BEL_IV_C_SA_16_17;TCUR_IV_M_SA_18_19;BEL_IV_C_SA_18_19;TCUR_IV_M_SA_20_21;BEL_IV_C_SA_20_21;TCUR_IV_M_SA_22_23;BEL_IV_C_SA_22_23;TCUR_IV_M_SU_0_1;BEL_IV_C_SU_0_1;TCUR_IV_M_SU_2_3;BEL_IV_C_SU_2_3;TCUR_IV_M_SU_4_5;BEL_IV_C_SU_4_5;TCUR_IV_M_SU_6_7;BEL_IV_C_SU_6_7;TCUR_IV_M_SU_8_9;BEL_IV_C_SU_8_9;TCUR_IV_M_SU_10_11;BEL_IV_C_SU_10_11;TCUR_IV_M_SU_12_13;BEL_IV_C_SU_12_13;TCUR_IV_M_SU_14_15;BEL_IV_C_SU_14_15;TCUR_IV_M_SU_16_17;BEL_IV_C_SU_16_17;TCUR_IV_M_SU_18_19;BEL_IV_C_SU_18_19;TCUR_IV_M_SU_20_21;BEL_IV_C_SU_20_21;TCUR_IV_M_SU_22_23;BEL_IV_C_SU_22_23"
        travel_time_interval_translator = {}
        for y in x.split(";"):
            ps = y.split("_")
            if ps[0] == "TCUR":
                if ps[2] == "M":
                    travel_time_interval_translator[y] = ["_".join(ps[3:])]

        visum_net_p = r'C:\Users\ge37ser\Documents\Coding\TUM_VT_FleetSimulation\tum-vt-fleet-simulation\data\networks\MOIA_HH_12052021_raw\210512_network_fuerFlotte_basisszenario.net'
        readVissimNetworkFileAndCreateNetworkWithUmlegung(visum_net_p, nw_data_folder, "MOIA_HH_12052021_raw", travel_time_interval_translator)

    if True:
        visum_net_p = r'C:\Users\ge37ser\Documents\Coding\TUM_VT_FleetSimulation\tum-vt-fleet-simulation\data\networks\MOIA_HH_12052021_raw\210512_network_fuerFlotte_basisszenario.net'
        readZones(visum_net_p, zone_data_folder, "MOIA_HH_12052021")