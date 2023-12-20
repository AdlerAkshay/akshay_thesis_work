import sys
import xml.etree.ElementTree as ET
import csv


def parse_xml(xml_file, vehicle_data=None):
    tree = ET.parse(xml_file)
    root = tree.getroot()

    result = []

    for tag_type in root.findall("vehicle"):
        vehicle_type = tag_type.get('type')
        vehicles = {}
        depart = None
        route_edges = None
        if vehicle_type is not None and "opti_driver" in vehicle_type:
            depart = float(tag_type.get('depart'))
            children_using_findall = tag_type.findall('*')
            for each_route in children_using_findall:
                route_edges = each_route.get('edges')
                vehicles['depart'] = depart
                vehicles['route_edges'] = route_edges
                result.append(vehicles)
                break

    return result


def write_to_csv(vehicles, csv_file):
    fieldnames = ['Depart', 'Route Edges']

    with open(csv_file, 'w', newline='') as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()

        for vehicle in vehicles:
            writer.writerow({
                'Depart': vehicle['depart'],
                'Route Edges': vehicle['route_edges'],
            })


def main():
    if len(sys.argv) != 3:
        print("Usage: python script.py input.xml output.csv")
        sys.exit(1)

    xml_file_path = sys.argv[1]
    csv_file_path = sys.argv[2]

    try:
        vehicles = parse_xml(xml_file_path)
        write_to_csv(vehicles, csv_file_path)
        print(f"Data written to {csv_file_path}")
        print('count')
    except Exception as e:
        print(f"Error: {e}")


if __name__ == "__main__":
    main()
