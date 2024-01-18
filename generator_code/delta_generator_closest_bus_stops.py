import csv
import sys
import os
import xml
import xml.etree.ElementTree as ET
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

    def start(self, stop_xml_file="output.xml"):
        self.net = sumolib.net.readNet(self.network_file)
        self._read_bus_stops_xml()
        self.fill_in_necessary_data_structures()
        self._remove_and_write_bus_stop(stop_xml_file)

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
            for bus_stop in root_node:
                id = bus_stop.get(str("id"))
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
        if rad > 10000:
            return []

        if from_edge_id not in self.edge_id_to_pos:
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

    def _remove_and_write_bus_stop(self, stop_xml_file):
        root = ET.Element("additional", xmlns="http://sumo.dlr.de/xsd/additional_file.xsd",
                          attrib={"xmlns:xsi": "http://www.w3.org/2001/XMLSchema-instance",
                                  "xsi:noNamespaceSchemaLocation": "http://sumo.dlr.de/xsd/additional_file.xsd"})

        copy = dict(self.bus_stops_edge_id_to_id)

        for edge_id, value_ign in self.bus_stops_edge_id_to_id.items():
            if edge_id in copy:
                list_of_bus_stops_300 = self._found_bus_stops(edge_id, 600)
                list_of_bus_stops_100 = self._found_bus_stops(edge_id, 100)

                delta_diff = list(set(list_of_bus_stops_300) - set(list_of_bus_stops_100))

                for rem_edge_id in delta_diff:
                    if rem_edge_id in copy:
                        del copy[rem_edge_id]

        for edge_id, _ in copy.items():
            if edge_id in self.lane_id_to_bus_stop:
                bus_stop = self.lane_id_to_bus_stop[edge_id]

                stop_type = bus_stop.tag

                stop_element = ET.Element(stop_type, id=bus_stop.get("id"), name=bus_stop.get("name"),
                                          lane=bus_stop.get("lane"), startPos=bus_stop.get("startPos"),
                                          endPos=bus_stop.get("endPos"))

                if bus_stop.get("friendlyPos"):
                    stop_element.set("friendlyPos", bus_stop.get("friendlyPos"))

                root.append(stop_element)

        poly_element = ET.Element("poly", id="001", color="red", fill="1", layer="2.00",
                                  shape="7055.272713,6803.108001 7035.272713,6823.108001 7075.272713,"
                                        "6823.108001 7055.272713,6843.108001")
        root.append(poly_element)

        # Convert the ElementTree to a string
        xml_str = ET.tostring(root, encoding='utf-8', method='xml').decode()

        # Use xml.dom.minidom to format the XML string with indentation
        dom = xml.dom.minidom.parseString(xml_str)
        formatted_xml = dom.toprettyxml(indent="  ")

        # Write the formatted XML to the file
        with open(stop_xml_file, "w", encoding="utf-8") as xml_file:
            xml_file.write(formatted_xml)


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

    ig.start("stops_at_600.add.xml")
    print("ended")
