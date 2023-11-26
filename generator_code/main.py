import csv
import sys
from threading import Thread
import xml.etree.ElementTree as ET

node_dict = {}
demand_request_dict = []


class DemandFileGenerator:

    @staticmethod
    def read_node_csv(csv_node_file):
        try:
            with open(csv_node_file, newline='') as csv_file:
                reader = csv.DictReader(csv_file)

                # Iterate over the rows in the CSV file
                for row in reader:
                    source_node_id = row['edge_id']
                    node_dict[source_node_id] = row
        except NameError:
            print(" exception raised, failed to process csv")

    @staticmethod
    def read_request_file(xml_demand_file):
        tree = ET.parse(xml_demand_file)
        root_node = tree.getroot()
        for person_element in root_node.findall('person'):
            request_id = person_element.get('id')
            request_id = request_id.split("_")
            request_time = person_element.get('depart')

            ride_element = person_element.find('ride')
            ride_from = ride_element.get('from')
            ride_to = ride_element.get('to')
            temp_list = [request_id[1], request_time, ride_from, ride_to]
            demand_request_dict.append(temp_list)

    def create_demand_request_csv(self, demand_request_csv_file):

        with open(demand_request_csv_file, 'w', newline='') as csv_file:

            header = "start,end,rq_time,request_id"
            csv_file.write(header)
            csv_file.write("\n")
            for value in demand_request_dict:
                ride_from = value[2]
                ride_to = value[3]
                request_id = value[0]
                request_time = value[1]

                s_edge_id = "S_" + ride_from
                e_edge_id = "E_" + ride_to

                start = None
                if s_edge_id in node_dict:
                    start = node_dict[s_edge_id]

                end = None

                if e_edge_id in node_dict:
                    end = node_dict[e_edge_id]

                final_str = str(start['node_index']) + "," + str(
                   end['node_index']) + "," + request_time + "," + request_id
                csv_file.write(final_str)
                csv_file.write("\n")


if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("enter correct node csv and demand xml file please")

    node_csv_file = sys.argv[1]
    demand_xml_file = sys.argv[2]

    demand_file_gen = DemandFileGenerator()

    thread_csv_reader = Thread(target=demand_file_gen.read_node_csv, args=[node_csv_file])
    thread_csv_reader.start()
    thread_csv_reader.join()

    thread_demand_xml_reader = Thread(target=demand_file_gen.read_request_file, args=[demand_xml_file])
    thread_demand_xml_reader.start()
    thread_demand_xml_reader.join()

    demand_file_gen.create_demand_request_csv(sys.argv[3])
