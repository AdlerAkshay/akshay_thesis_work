import sys
import xml.etree.ElementTree as ET
import xml.dom.minidom
import threading
import time


def time_calc(foo):
    def wrapper():
        st = time.time()
        foo()
        print(f"{foo.__name__} took {time.time() - st} seconds")
    return wrapper


def randomize_and_complement(input_list, percentage):
    i = 0
    per_list = []
    remaining_list = []
    for item in input_list:
        i += 1
        if i % percentage == 0:
            per_list.append(item)
        else:
            remaining_list.append(item)

    return per_list, remaining_list


def parse_xml(xml_file, vehicle_data=None):
    tree = ET.parse(xml_file)
    root = tree.getroot()

    result = []
    result_remaining = []

    for tag_type in root.findall("vehicle"):
        vehicle_type = tag_type.get('type')
        vehicles = {}
        vehicles_remaining = {}
        depart = float(tag_type.get('depart'))
        route_edges = None
        depart_lane = tag_type.get('departLane')
        depart_speed = tag_type.get('departSpeed')
        id = tag_type.get('id')
        type = tag_type.get('type')
        # now save only opti_driver types
        if vehicle_type is not None and "opti_driver" in vehicle_type:
            children_using_findall = tag_type.findall('*')
            for each_route in children_using_findall:
                route_edges = each_route.get('edges')
                vehicles['depart'] = depart
                vehicles['route_edges'] = route_edges
                vehicles['depart_lane'] = depart_lane
                vehicles['depart_speed'] = depart_speed
                vehicles['id'] = id
                vehicles['type'] = type
                result.append(vehicles)
                break
        else:
            # first save all the nodes
            children_using_findall = tag_type.findall('*')
            for each_route in children_using_findall:
                route_edges = each_route.get('edges')
                vehicles_remaining['depart'] = depart
                vehicles_remaining['route_edges'] = route_edges
                vehicles_remaining['depart_lane'] = depart_lane
                vehicles_remaining['depart_speed'] = depart_speed
                vehicles_remaining['id'] = id
                vehicles_remaining['type'] = type
                result_remaining.append(vehicles_remaining)
                break

    return result, result_remaining


def write_to_remaining_xml(vehicles_remaining, remaining_file_xml_file):
    root = ET.Element("routes")

    try:
        for vehicle_remaining in vehicles_remaining:
            child = ET.SubElement(root, "vehicle")
            child.set("depart", str(vehicle_remaining['depart']))
            child.set("departLane", vehicle_remaining['depart_lane'])
            child.set("departSpeed", vehicle_remaining['depart_speed'])
            child.set("id", vehicle_remaining['id'])
            child.set("type", vehicle_remaining['type'])
            grandchild = ET.SubElement(child, "route")
            grandchild.set("edges", vehicle_remaining['route_edges'])

    except Exception as e:
        print(f" {e} ")
        return

    # Convert the ElementTree to a string
    xml_str = ET.tostring(root, encoding='utf-8', method='xml').decode()

    # Use xml.dom.minidom to format the XML string with indentation
    dom = xml.dom.minidom.parseString(xml_str)
    formatted_xml = dom.toprettyxml(indent="  ")

    # Write the formatted XML to the file
    with open(remaining_file_xml_file, "w", encoding="utf-8") as xml_file:
        xml_file.write(formatted_xml)


def write_to_xml(vehicles, demand_xml_file):
    root = ET.Element("persons")
    id = 1

    try:
        for vehicle in vehicles:
            child = ET.SubElement(root, "person")
            child.set("id", str(id))
            child.set("depart", str(vehicle['depart']))
            child.set("color", "#00008B")
            grandchild = ET.SubElement(child, "ride")
            r_edges = vehicle['route_edges']
            list_route_edges = r_edges.split(" ")
            frm = list_route_edges[0]
            to = list_route_edges[len(list_route_edges) - 1]
            grandchild.set("from", frm)
            grandchild.set("to", to)
            grandchild.set("lines", "taxi")
            id += 1
    except Exception as e:
        print(f" {e} ")
        return

    # Convert the ElementTree to a string
    xml_str = ET.tostring(root, encoding='utf-8', method='xml').decode()

    # Use xml.dom.minidom to format the XML string with indentation
    dom = xml.dom.minidom.parseString(xml_str)
    formatted_xml = dom.toprettyxml(indent="  ")

    # Write the formatted XML to the file
    with open(demand_xml_file, "w", encoding="utf-8") as xml_file:
        xml_file.write(formatted_xml)


@time_calc
def main():
    if len(sys.argv) != 4:
        print("Usage: python script.py input.xml output.csv")
        sys.exit(1)

    route_xml_file = sys.argv[1]
    demand_xml_file = sys.argv[2]
    remaining_file_xml_file = sys.argv[3]

    try:
        vehicles_opti_driver, vehicles_remaining = parse_xml(route_xml_file)

        random_vehicles, random_vehicles_compliment = randomize_and_complement(vehicles_opti_driver, 10)

        vehicles_remaining_filtered = vehicles_remaining + random_vehicles_compliment

        thread_write_xml = threading.Thread(target=write_to_xml, args=[random_vehicles, demand_xml_file])
        thread_write_xml_rem = threading.Thread(target=write_to_remaining_xml,
                                                args=[vehicles_remaining_filtered, remaining_file_xml_file])
        thread_write_xml.start()
        thread_write_xml_rem.start()
        thread_write_xml.join()
        thread_write_xml_rem.join()

    except Exception as e:
        print(f"Error: {e}")

        print("done")


if __name__ == "__main__":
    main()
