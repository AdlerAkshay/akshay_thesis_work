import csv
import sys
import os
#from threading import Thread
import threading
import xml.etree.ElementTree as ET
import time

sys.path.append("/Users/rakeshsadhu/dev/python_env/lib/python3.11/site-packages/sumolib")
import sumolib  # noqa

class ImprovedGenerator:

    def __init__(self, network_file, bus_xml, edge_id_node_id_file_csv, demand_file_inter_csv, final_output_csv):
        # save the files in member variables
        self.network_file = network_file
        self.bus_xml = bus_xml
        self.edge_id_node_id_file_csv = edge_id_node_id_file_csv
        self.demand_file_inter_csv = demand_file_inter_csv
        self.final_demand_file = final_output_csv

        # dictionaries needed
        self.node_id_to_edge_id = {}
        self.edge_id_to_node_id = {}
        self.edge_id_to_pos = {}
        self.bus_stops_dict = {}
        self.bus_stops_edge_id_to_id = {}
        self.count=0


    def start(self):
        self.net = sumolib.net.readNet(self.network_file)
        self.read_bus_stops_th = threading.Thread(target=self.read_bus_stops_xml)
        self.read_neccessary_ds_th = threading.Thread(target=self.fill_in_necessary_data_structures)

        self.read_bus_stops_th.start()
        self.read_neccessary_ds_th.start()

        self.read_bus_stops_th.join()
        self.read_neccessary_ds_th.join()

        self.cal_closest_bus_stop()

    def read_bus_stops_xml(self):
        try:
            tree = ET.parse(self.bus_xml)
            root_node = tree.getroot()
            for bus_stop in root_node.findall('busStop'):
                id = bus_stop.get("id")
                lane_id = bus_stop.get("lane")
                lane_id = lane_id.split("_")
                if not self.net.hasEdge(lane_id[0]):
                    continue
                self.bus_stops_dict[id] = lane_id[0]
                self.bus_stops_edge_id_to_id[lane_id[0]] = id
        except NameError:
            print("error in reading xml \n exception thrown read_bus_stops_xml")
            return False
        return True


    def fill_in_necessary_data_structures(self):
        try:
            with open(self.edge_id_node_id_file_csv, newline='') as csv_file:
                reader = csv.DictReader(csv_file)

                # Iterate over the rows in the CSV file
                for row in reader:
                    edge_id = row['edge_id']
                    node_id = row['node_index']
                    edge_id = edge_id.split("_")[1]
                    self.node_id_to_edge_id[node_id] = edge_id
                    self.edge_id_to_node_id[edge_id] = node_id
                    self.edge_id_to_pos[edge_id] = (row["pos_x"], row["pos_y"])
        except NameError:
            print("fill_in_necessary_data_structures exception thrown")
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
            return self._found_bus_stops(from_edge_id, rad*10)

        return list_of_bus_stops



    def _use_network_shortest_path(self, from_edge_id):

        shortest_path = sys.maxsize
        bus_stop = None

        neighbouring_bus_stops = self._found_bus_stops(from_edge_id)
        self.count+=1
        print(f"iteration   -->  {self.count}")
        if len(neighbouring_bus_stops) == 0:
            return None
        elif len(neighbouring_bus_stops) == 1:
            return neighbouring_bus_stops[0]
        else:
            for to_edge_id in neighbouring_bus_stops:
                from_edge_obj = self.net.getEdge(from_edge_id)
                to_edge_obj = self.net.getEdge(to_edge_id)

                tmp = self.net.getShortestPath(from_edge_obj, to_edge_obj)
                if tmp[1] < shortest_path:
                    shortest_path = tmp[1]
                    bus_stop = tmp

        if bus_stop is not None:
            bus_tuple = bus_stop[0]
            last_index = len(bus_tuple) - 1
            edge_id_o = bus_tuple[last_index]
            return edge_id_o.getID()

    def cal_closest_bus_stop(self):

        start_time = time.time()
        try:
            with open(self.demand_file_inter_csv, newline='') as csv_file, open(self.final_demand_file, "w", newline='') as f_output:
                reader = csv.DictReader(csv_file)
                header_string = ','.join(reader.fieldnames) +"\n"

                f_output.write(header_string)

                # Iterate over the rows in the CSV file
                for row in reader:

                    start_edge_id = self.node_id_to_edge_id[row['start']]
                    end_edge_id = self.node_id_to_edge_id[row['end']]
                    req_id = row['request_id']
                    req_time = row['rq_time']

                    closest_starting_bus_stop_edge_id = self.edge_id_to_node_id[self._use_network_shortest_path(start_edge_id)]
                    closest_ending_bus_stop_edge_id = self.edge_id_to_node_id[self._use_network_shortest_path(end_edge_id)]

                    my_row = closest_starting_bus_stop_edge_id + "," + closest_ending_bus_stop_edge_id +"," + req_time +  "," + req_id +"\n"
                    f_output.write(my_row)

        except NameError:
            print("cal_closest_bus_stop exception thrown")
            return False
        end_time = time.time()
        elapsed_time = end_time - start_time

        print(f"time spent {elapsed_time}")
        return True


if __name__ == "__main__":

    ig = ImprovedGenerator("/Users/rakeshsadhu/dev/akshay_thesis_work/generator_code/data/ingolstadt_24h.net.xml",
                           "/Users/rakeshsadhu/dev/akshay_thesis_work/generator_code/data/23-07-19_pt_stops_gtfs_updated_net.add.xml",
                           "/Users/rakeshsadhu/dev/akshay_thesis_work/generator_code/data/complete_nodes.csv",
                           "/Users/rakeshsadhu/dev/akshay_thesis_work/generator_code/data/final_output.csv",
                           "/Users/rakeshsadhu/dev/akshay_thesis_work/generator_code/data/demand_final_output.csv")

    ret = ig.start()
    if ret:
        print("successfully executed")
    else:
        print("something went terribly wrong")
