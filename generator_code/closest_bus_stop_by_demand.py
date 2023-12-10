import csv
import sys
import os
from threading import Thread
import xml.etree.ElementTree as ET

sys.path.append(os.path.join(os.environ["SUMO_HOME"], "tools"))
import sumolib  # noqa

net = sumolib.net.readNet("C:\\Users\\Audi\\Desktop\\thesis_work\\akshay_thesis_work\\tum-vt-fleet-simulation-master"
                          "\\FleetPy\\studies\\SUMO_example_study\\SUMO_model\\ingolstadt_24h.net.xml")

bus_stops_dict = {}
node_id_to_edge_id = {}
edge_id_to_node_id = {}


def read_bus_stops_xml(bus_xml):
    try:
        tree = ET.parse(bus_xml)
        root_node = tree.getroot()
        for bus_stop in root_node.findall('busStop'):
            id = bus_stop.get("id")
            lane_id = bus_stop.get("lane")
            lane_id = lane_id.split("_")
            if not net.hasEdge(lane_id[0]):
                continue
            bus_stops_dict[id] = lane_id[0]
    except NameError:
        print("error in reading xml \n exception thrown read_bus_stops_xml")
        return False
    return True


def fill_in_necessary_data_structures(edge_id_node_id_file_csv):
    try:
        with open(edge_id_node_id_file_csv, newline='') as csv_file:
            reader = csv.DictReader(csv_file)

            # Iterate over the rows in the CSV file
            for row in reader:
                edge_id = row['edge_id']
                node_id = row['node_index']
                edge_id = edge_id.split("_")[1]
                node_id_to_edge_id[node_id] = edge_id
                edge_id_to_node_id[edge_id] = node_id
    except NameError:
        print("fill_in_necessary_data_structures exception thrown")
        return False
    return True


def use_network_shortest_path(from_edge_id):
    shortest_path = sys.maxsize
    bus_stop = None
    try:
        for _, to_edge_id in bus_stops_dict.items():
            from_edge_obj = net.getEdge(from_edge_id)
            to_edge_obj = net.getEdge(to_edge_id)

            tmp = net.getShortestPath(from_edge_obj, to_edge_obj)
            if tmp[1] < shortest_path:
                shortest_path = tmp[1]
                bus_stop = tmp
    except NameError:
        raise TypeError("Net getShortestPath failed")

    if bus_stop is not None:
        bus_tuple = bus_stop[0]
        last_index = len(bus_tuple) - 1
        edge_id_o = bus_tuple[last_index]
        return edge_id_o.getID()
    else:
        return -1

def cal_closest_bus_stop(demand_file_inter_csv, final_demand_file):
    try:
        with open(demand_file_inter_csv, newline='') as csv_file, open(final_demand_file, "a", newline='') as f_output:
            reader = csv.DictReader(csv_file)
            writer = csv.writer(f_output)
            header = next(reader)
            writer.writerow(header)

            # Iterate over the rows in the CSV file
            for row in reader:
                start_edge_id = node_id_to_edge_id[row['start']]
                end_edge_id = node_id_to_edge_id[row['end']]
                req_id = row['request_id']
                req_time = row['rq_time']

                closest_starting_bus_stop_edge_id = edge_id_to_node_id[use_network_shortest_path(start_edge_id)]
                closest_ending_bus_stop_edge_id = edge_id_to_node_id[use_network_shortest_path(end_edge_id)]

                writer.write({closest_starting_bus_stop_edge_id, closest_ending_bus_stop_edge_id, req_time, req_id})

    except NameError:
        print("cal_closest_bus_stop exception thrown")
        return False
    return True


if __name__ == "__main__":
    if len(sys.argv) != 5:
        print("get your necessary files right and try again")
        print("you would need following")
        print("bus stops xml, node_edge_id_csv, demand_file, name_of_final_file")
        exit(-1)

    ret = read_bus_stops_xml(sys.argv[1])
    ret &= fill_in_necessary_data_structures(sys.argv[2])
    ret &= cal_closest_bus_stop(sys.argv[3], sys.argv[4])

    if ret:
        print("successfully executed")
    else:
        print("something went terribly wrong")
