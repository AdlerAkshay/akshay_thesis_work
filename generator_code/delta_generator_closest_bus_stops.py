import csv
import sys
import os
# from threading import Thread
import threading
import xml.etree.ElementTree as ET
import time

sys.path.append(os.path.join(os.environ["SUMO_HOME"], "tools"))
import sumolib  # noqa


class ImprovedGeneratorExt:

    def __init__(self, network_file, bus_xml, bus_xml_600, all_nodes_csv):
        # save the files in member variables
        self.network_file = network_file
        self.bus_xml = bus_xml
        self.edge_id_node_id_file_csv = all_nodes_csv

        self.bus_xml_600 = bus_xml_600

        # dictionaries needed
        self.bus_stops_edge_id_to_id = {}
        self.lane_id_to_bus_stop = {}
        self.edge_id_to_pos = {}

    def start(self):
        self.net = sumolib.net.readNet(self.network_file)
        self._read_bus_stops_xml()
        self.fill_in_necessary_data_structures()
        self._cal_closest_bus_stop()

    def fill_in_necessary_data_structures(self):
        try:
            with open(self.edge_id_node_id_file_csv, newline='') as csv_file:
                reader = csv.DictReader(csv_file)

                # Iterate over the rows in the CSV file
                for row in reader:
                    edge_id = row['edge_id']
                    node_id = row['node_index']
                    edge_id = edge_id.split("_")[1]
                    # self.node_id_to_edge_id[node_id] = edge_id
                    # self.edge_id_to_node_id[edge_id] = node_id
                    self.edge_id_to_pos[edge_id] = (row["pos_x"], row["pos_y"])
        except NameError:
            print("fill_in_necessary_data_structures exception thrown")
            return False
        return True

    def _read_bus_stops_xml(self):
        try:
            tree = ET.parse(self.bus_xml)
            root_node = tree.getroot()
            for bus_stop in root_node.findall('busStop'):
                id = bus_stop.get("id")
                lane_id = bus_stop.get("lane")
                lane_id = lane_id.split("_")
                if not self.net.hasEdge(lane_id[0]):
                    continue
                self.bus_stops_edge_id_to_id[lane_id[0]] = id
                self.lane_id_to_bus_stop[lane_id[0]] = bus_stop
        except NameError:
            print("error in reading xml \n exception thrown read_bus_stops_xml")
            return False
        return True

    def _found_bus_stops(self, from_edge_id, rad=1):
        if rad > 1000000:
            return []

        pos_x_y = self.edge_id_to_pos[from_edge_id]

        neighbours = self.net.getNeighboringEdges(float(pos_x_y[0]), float(pos_x_y[1]), rad)
        list_of_bus_stops = []
        for ngh in neighbours:
            edge_id = ngh[0].getID()
            if edge_id in self.bus_stops_edge_id_to_id:
                list_of_bus_stops.append(edge_id)

        if len(list_of_bus_stops) == 0:
            return self._found_bus_stops(from_edge_id, rad * 2)

        return list_of_bus_stops

    def _cal_closest_bus_stop(self):
        copy = self.bus_stops_edge_id_to_id
        for edge_id, _ in self.bus_stops_edge_id_to_id.items():
            list_of_bus_stops = self._found_bus_stops(edge_id, 0.6)
            for rem_edge_id in list_of_bus_stops:
                del copy[rem_edge_id]
            self.bus_stops_edge_id_to_id = copy

        for edge_id, _ in self.bus_stops_edge_id_to_id.items():
            if edge_id in self.lane_id_to_bus_stop:
                print(self.lane_id_to_bus_stop[edge_id])


if __name__ == "__main__":
    ig = ImprovedGeneratorExt("C:\\Users\\Audi\\Desktop\\thesis_work\\akshay_thesis_work\\generator_code\\data"
                              "\\ingolstadt_24h.net.xml",
                              "C:\\Users\\Audi\\Desktop\\thesis_work\\akshay_thesis_work\\generator_code\\data\\23-07"
                              "-19_pt_stops_gtfs_updated_net.add.xml",
                              "C:\\Users\\Audi\\Desktop\\thesis_work\\akshay_thesis_work\\generator_code\\data"
                              "\\600_23-07-19_pt_stops_gtfs_updated_net.add.xml",
                              "C:\\Users\\Audi\\Desktop\\thesis_work\\akshay_thesis_work\\generator_code\\data"
                              "\\Allnodes.csv",
                              )

    ig.start()
    print("ended")
