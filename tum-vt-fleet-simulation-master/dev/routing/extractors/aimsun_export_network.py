import traceback
import os
import json
import inspect

# import Network

# --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- #
# global Variables
system = GKSystem.getSystem()
model = GKSystem.getSystem().getActiveModel()


# --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- #
# help functions
def returnListIds(list_aimsun_objs):
    return [x.getId() for x in list_aimsun_objs]

def createPointList(list_points):
    return [(pt.x, pt.y) for pt in list_points]

def returnDictIds(list_aimsun_objs):
    return { x.getId() : 1 for x in list_aimsun_objs}

def hasPriorityOver(aimsun_node_obj):
    priority_list = []
    for turn1 in aimsun_node_obj.getTurnings():
        for turn2 in aimsun_node_obj.getTurnings():
            if turn1.getId() == turn2.getId():
                continue
            if aimsun_node_obj.hasPriority(turn1, turn2):
                priority_list.append( (turn1.getId(), turn2.getId() ) )
    return priority_list

def convertCoordListToString(coord_list):
    l = []
    for coord in coord_list:
        l.append("{};{}".format(coord[0], coord[1]))
    return "|".join(l)

# --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- #
# class definitions
class Lane():
    def __init__(self, aimsun_lane_obj):
        self.init_offset = aimsun_lane_obj.getInitialOffset()
        self.final_offset = aimsun_lane_obj.getFinalOffset()

    def returnDict(self):
        return self.__dict__


class Section():
    def __init__(self, aimsun_section_obj):
        #self.att = {}
        # ID and connectivity
        self.id = aimsun_section_obj.getId()
        self.entrance_turns = returnDictIds(aimsun_section_obj.getOrigTurnings())
        self.entrance_edges = returnDictIds(aimsun_section_obj.getEntranceSections())
        try:
            self.entrance_node = aimsun_section_obj.getOrigin().getId()
        except:
            self.entrance_node = None
        self.exit_turns = returnDictIds(aimsun_section_obj.getDestTurnings())
        self.exit_edges = returnDictIds(aimsun_section_obj.getExitSections())
        try:
            self.exit_node = aimsun_section_obj.getDestination().getId()
        except:
            self.exit_node = None
        # geometric attributes
        self.polyline = createPointList(aimsun_section_obj.getPoints())
        self.lanes = [Lane(x).returnDict() for x in aimsun_section_obj.getLanes()]
        self.lane_widths = [aimsun_section_obj.getLaneWidth(i) for i in range(len(self.lanes)) ]
        # get additional information
        self.speed_limit = aimsun_section_obj.getSpeed() # speed in km/h
        self.roadtype = aimsun_section_obj.getRoadType().getName()
        self.length = aimsun_section_obj.length2D()
        # debug: print info
        #print(self.__dict__)

    def __str__(self):
        try:
            return json.dumps(self.__dict__)
        except:
            traceback.print_exc()
            raise IOError()

# --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- #
# class definitions
class StopLine():
    def __init__(self, aimsun_stopline_obj):
        self.length = aimsun_stopline_obj.getLength()
        self.position = aimsun_stopline_obj.getPosition()
    
    def returnDict(self):
        return self.__dict__

class Turn():
    def __init__(self, aimsun_turn_obj):
        # ID and connectivity
        self.id = aimsun_turn_obj.getId()
        self.destination_section = aimsun_turn_obj.getDestination().getId()
        self.origin_section = aimsun_turn_obj.getOrigin().getId()
        self.node = aimsun_turn_obj.getNode().getId()
        self.destination_lanes = list(range(aimsun_turn_obj.getDestinationFromLane(), aimsun_turn_obj.getDestinationToLane() + 1))
        self.origin_lanes = list(range(aimsun_turn_obj.getOriginFromLane(), aimsun_turn_obj.getOriginToLane() + 1))
        self.stop_lines = [StopLine(stop).returnDict() for stop in aimsun_turn_obj.getStopLines()]
        self.is_internal_roundabout_turning = aimsun_turn_obj.isInternalRoundaboutTurning()
        self.angle_between_origin_destination_section = aimsun_turn_obj.calcAngleSections()
        self.speed = aimsun_turn_obj.getSpeed()
        self.polyline = createPointList(aimsun_turn_obj.getPoints())
        self.length = aimsun_turn_obj.length2D()


        #print(self.__dict__)

    def __str__(self):
        try:
            return json.dumps(self.__dict__)
        except:
            traceback.print_exc()
            raise IOError()

class Node():
    def __init__(self, aimsun_node_obj):
        # ID and connectivity
        self.id = aimsun_node_obj.getId()
        self.entrance_sections = returnDictIds(aimsun_node_obj.getEntranceSections())
        self.exit_sections = returnDictIds(aimsun_node_obj.getExitSections())
        self.turns = returnDictIds(aimsun_node_obj.getTurnings())
        self.signal_groups = returnDictIds(aimsun_node_obj.getSignals())
        self.polygon = createPointList(aimsun_node_obj.getPolygon())
        self.first_turn_has_priority_over_second = hasPriorityOver(aimsun_node_obj)

        #print(self.__dict__)

    def __str__(self):
        try:
            return json.dumps(self.__dict__)
        except:
            traceback.print_exc()
            raise IOError()

# #NETWORK CLASSES FOR EXPORT

class Network():
    def __init__(self):
        self.nodes = []
        self.edge_id_to_edge = {}
        self.source_edge_id_to_edge = {}
        self.super_nodes = {}

    def addNode(self, node):
        self.nodes.append(node)

    def getCurrentNodeNumber(self):
        return len(self.nodes)

    def addEdge(self, edge):
        start_index = edge.start_node_index
        end_index = edge.end_node_index
        print("edge {} -> {} nodes {}".format(start_index, end_index, len(self.nodes)))
        self.nodes[start_index].addOutgoingEdge(edge)
        self.nodes[end_index].addIncomingEdge(edge)
        if self.edge_id_to_edge.get(edge.id):
            print("double edge: {} <-> {}".format(edge, self.edge_id_to_edge[edge.id]))
        self.edge_id_to_edge[edge.id] = edge
        if edge.source_edge_id is not None:
            self.source_edge_id_to_edge[edge.source_edge_id] = edge

    def addSuperNode(self, super_node):
        self.super_nodes[super_node.id] = super_node

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



class NetworkNode():
    def __init__(self, node_index, pos_x, pos_y, is_stop_only = False, ch_value = -1, source_edge_id = None):
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
        s = "{},{},{},{}".format(self.index, self.coordinates[0], self.coordinates[1], self.source_edge_id)
        if self.is_stop_only:
            s += ",{}".format(1)
        else:
            s += ","
        s += ",{}".format(self.ch_value)
        return s

class NetworkSuperNode():
    def __init__(self, id, node_id_list, polygon = None, source_edge_id = None):
        self.node_collection = node_id_list[:]
        self.id = id
        self.polygon = polygon
        self.source_edge_id = source_edge_id

    def __str__(self):
        return "{},{},{},{}".format(self.id, self.source_edge_id, ";".join([str(x) for x in self.node_collection]), convertCoordListToString(self.polygon))

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
        s = "{},{},{},{},{},{},{}".format(self.start_node_index, self.end_node_index, self.travel_time, self.travel_distance, self.id, self.source_edge_id,self.roadtype)
        return s




# --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- #
# function definitions
def getTurnInformation():
    """ collects information of aimsun turn objects (these objects are also treated as edges in the fleet simulation network) """
    cat_type = model.getType("GKTurning")
    aimsun_obj_dict = model.getCatalog().getObjectsByType(cat_type)
    turns = {}
    for oid, aimsun_obj in aimsun_obj_dict.items():
        tmp_turn = Turn(aimsun_obj)
        turns[tmp_turn.id] = tmp_turn
    return turns

def getSectionInformation():
    """ collects information of aimsun section objects (this objects are treated as edges in the fleet simulation network) """
    cat_type = model.getType("GKSection")
    aimsun_obj_dict = model.getCatalog().getObjectsByType(cat_type)
    sections = {}
    for oid, aimsun_obj in aimsun_obj_dict.items():
        tmp_section = Section(aimsun_obj)
        sections[tmp_section.id] = tmp_section
    return sections

def getNodeInformation():
    """ collects information of aimsun node objects (this objects are NO nodes in the fleet simulation network!) """
    cat_type = model.getType("GKNode")
    aimsun_obj_dict = model.getCatalog().getObjectsByType(cat_type)
    nodes = {}
    for oid, aimsun_obj in aimsun_obj_dict.items():
        tmp_node = Node(aimsun_obj)
        nodes[tmp_node.id] = tmp_node
    return nodes


def createNetwork(section_dict, turn_dict, node_dict, output_folder, name):
    nw = Network()
    print("assuming speed in km/h")
    for sec_id, section in section_dict.items():
        #print("s ", sec_id)
        start_coord = section.polyline[0]
        end_coord = section.polyline[-1]
        #print(start_coord, end_coord)
        start_node = NetworkNode(nw.getCurrentNodeNumber(), start_coord[0], start_coord[1])
        nw.addNode(start_node)
        end_node = NetworkNode(nw.getCurrentNodeNumber(), end_coord[0], end_coord[1])
        nw.addNode(end_node)
        dis = section.length
        speed = section.speed_limit / 3.6
        tt = dis/speed
        edge = NetworkEdge(start_node.index, end_node.index, tt, dis, source_edge_id=sec_id, polyline=section.polyline, roadtype=section.roadtype)
        print("edge: {}".format(edge))
        nw.addEdge(edge)

    for turn_id, turn in turn_dict.items():
        target_edge = nw.source_edge_id_to_edge[turn.destination_section]
        origin_edge = nw.source_edge_id_to_edge[turn.origin_section]
        start_node = origin_edge.getEndNodeIndex()
        end_node = target_edge.getStartNodeIndex()
        dis = turn.length
        speed = turn.speed / 3.6
        tt = dis/speed
        edge = NetworkEdge(start_node, end_node, tt, dis, source_edge_id=turn_id, roadtype="Turn", polyline=turn.polyline)
        nw.addEdge(edge)

    c = 0
    for node_id, aimsun_node in node_dict.items():
        node_id_collection = []
        for turn_id in aimsun_node.turns.keys():
            print("   ", turn_id)
            edge = nw.source_edge_id_to_edge[turn_id]
            node_id_collection.append(edge.start_node_index)
            node_id_collection.append(edge.end_node_index)
        polygon = aimsun_node.polygon
        super_node= NetworkSuperNode(c, list(set(node_id_collection)), polygon = polygon, source_edge_id=node_id)
        nw.addSuperNode(super_node)
        c+=1

    nw.exportNetworkFull(output_folder, name)

#==========MAIN======================#
"""
writes node-file, edge-file and supernode-file to the defined output_folder
network_name corresponds to the newly defined name of the network
this should be consistent with the network name later used in aimsun_network_converter.yp
"""
output_folder = r"C:\Users\ge37ser\Documents\Coding\TUM_VT_FleetSimulation\tum-vt-fleet-simulation\data\networks\Aimsun_Munich_2020_04_15"
name = "aimsun_exports"
createNetwork(getSectionInformation(), getTurnInformation(), getNodeInformation(), output_folder, name)