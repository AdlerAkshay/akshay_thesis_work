import csv
import xml.etree.ElementTree as ET
import operator
import pandas as pd
import numpy as np
import os
import sys
import geopandas as gpd
from shapely.geometry import Point, LineString


def write_an_extra_file(nodes, complete_nodes_dumped_as_file_as_csv):
    # Writing keys and values to a CSV file
    with open(complete_nodes_dumped_as_file_as_csv, 'w', newline='') as csv_file:
        try:
            writer = csv.writer(csv_file)

            # Write header
            header = ['edge_id', 'node_index', 'source_node_id', 'pos_x', 'pos_y', 'is_stop_only']
            writer.writerow(header)

            # Write data
            for key, inner_dict in nodes.items():
                row = [key] + [inner_dict.get(col, '') for col in header[1:]]
                writer.writerow(row)
        except NameError:
            print(" mor goya")


def create_nodes_and_edges_from_xml(xmlfile):
    # Approach is to parse the xml file of the sumo network for the relevant information
    tree = ET.parse(xmlfile)
    root_node = tree.getroot()

    nodes = {}  # node_index -> node info
    node_source_id_to_node_indices = {}  # node_source_id -> node_indices
    edges = {}  # edge_source_id -> edge info

    # create start and end node for each sumo link
    node_index = 0
    for tag in root_node.findall('edge'):

        function = tag.get('function')
        if function != 'internal':
            new_edgeID = tag.get('id')

            new_from_node = tag.get('from')
            if new_from_node is None:
                continue

            at_least_one_valid = False

            children_using_findall = tag.findall('*')
            for lane in children_using_findall:
                allowed = lane.get('allow')
                disallowed = lane.get('disallow')
                if allowed is not None and "taxi" not in allowed or disallowed is not None and "taxi" in disallowed:
                    continue
                else:
                    at_least_one_valid = True
                    break

            if at_least_one_valid is False:
                continue

            nodes[f"S_{new_edgeID}"] = {"node_index": node_index, "source_node_id": new_from_node}
            try:
                node_source_id_to_node_indices[new_from_node].append(f"S_{new_edgeID}")
            except KeyError:
                node_source_id_to_node_indices[new_from_node] = [f"S_{new_edgeID}"]
            node_index += 1

            new_to_node = tag.get('to')
            nodes[f"E_{new_edgeID}"] = {"node_index": node_index, "source_node_id": new_to_node}
            try:
                node_source_id_to_node_indices[new_to_node].append(f"E_{new_edgeID}")
            except KeyError:
                node_source_id_to_node_indices[new_to_node] = [f"E_{new_edgeID}"]
            node_index += 1

            edges[new_edgeID] = {"from_node": node_index - 2, "to_node": node_index - 1, "source_edge_id": new_edgeID}

    controlEdges = []

    for tag in root_node.findall('edge/lane'):
        lane_id = tag.get('id')
        lane_id = lane_id.split("_")
        # print(f'lane_id {lane_id} length {len(lane_id)}')
        if len(lane_id) == 2:
            edge_id = lane_id[0]  # Stops working if more than 9 lanes
            if edge_id not in controlEdges:
                # print(f'edge_id {edge_id}')
                new_distance = float(tag.get('length'))
                speed = float(tag.get('speed'))
                new_traveltime = new_distance / speed
                if edge_id in edges:
                    edges[edge_id]["travel_time"] = new_traveltime
                    edges[edge_id]["distance"] = new_distance
                    controlEdges.append(edge_id)

    # Add Connecting edges (within intersections)
    for tag in root_node.findall('connection'):
        fromEdge = tag.get('from')
        # print(f'fromEdge {fromEdge}')
        fromEdge = fromEdge.split("_")
        toEdge = tag.get('to')
        # print(f"toEdge {toEdge}")
        toEdge = toEdge.split("_")
        if len(fromEdge) == 1 and len(toEdge) == 1:
            fromEdge = fromEdge[0]
            toEdge = toEdge[0]
            # print(f'fromEdge {fromEdge}')
            # print(f"toEdge {toEdge}")
            newInternalEdgeID = 'i_' + str(fromEdge) + '_' + str(toEdge)

            if f"E_{fromEdge}" not in nodes or f"S_{toEdge}" not in nodes:
                continue

            from_node_index = nodes[f"E_{fromEdge}"]["node_index"]
            to_node_index = nodes[f"S_{toEdge}"]["node_index"]
            # print(from_node_index, to_node_index)
            edges[newInternalEdgeID] = {"from_node": from_node_index,
                                        "to_node": to_node_index,
                                        "distance": 0,
                                        "travel_time": 0,
                                        "source_edge_id": newInternalEdgeID}

    # add node information
    for tag in root_node.findall('junction'):

        type = tag.get('type')
        if type != 'internal':
            source_node_id = tag.get('id')
            pos_x = tag.get('x')
            pos_y = tag.get('y')

            if source_node_id not in node_source_id_to_node_indices:
                continue
            for node in node_source_id_to_node_indices[source_node_id]:
                nodes[node]["pos_x"] = float(pos_x)
                nodes[node]["pos_y"] = float(pos_y)
                nodes[node]["is_stop_only"] = False

    return nodes, edges


def store_network(nodes, edges, network_name):
    node_list = sorted(list(nodes.values()), key=lambda x: x["node_index"])
    edges_list = list(edges.values())
    node_df = pd.DataFrame(node_list)
    edges_df = pd.DataFrame(edges_list)

    p = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))),
                     "FleetPy", "data", "networks")
    p = os.path.join(p, network_name)
    if not os.path.isdir(p):
        os.mkdir(p)
    p = os.path.join(p, "base")
    if not os.path.isdir(p):
        os.mkdir(p)
    node_df.to_csv(os.path.join(p, "nodes.csv"), index=False)
    edges_df.to_csv(os.path.join(p, "edges.csv"), index=False)

    node_gdf_dict = {}
    for _, node in nodes.items():
        node["geometry"] = Point(node["pos_x"], node["pos_y"])
        node_gdf_dict[node["node_index"]] = node
    edge_gdf_dict = {}
    for edge in edges.values():
        edge["geometry"] = LineString([(node_gdf_dict[edge["from_node"]]["pos_x"],
                                        node_gdf_dict[edge["from_node"]]["pos_y"]),
                                       (node_gdf_dict[edge["to_node"]]["pos_x"],
                                        node_gdf_dict[edge["to_node"]]["pos_y"])])
        edge_gdf_dict[edge["source_edge_id"]] = edge
    node_gdf = gpd.GeoDataFrame(list(node_gdf_dict.values()))
    edge_gdf = gpd.GeoDataFrame(list(edge_gdf_dict.values()))
    node_gdf.to_file(os.path.join(p, "nodes_all_infos.geojson"), index=False, driver="GeoJSON")
    edge_gdf.to_file(os.path.join(p, "edges_all_infos.geojson"), index=False, driver="GeoJSON")

    with open(os.path.join(p, "crs.info"), "w") as f:
        f.write("unknowncrs")


def create_network(xmlfile, nw_name, ):
    nodes, edges = create_nodes_and_edges_from_xml(xmlfile)
    store_network(nodes, edges, nw_name)
    return nodes


if __name__ == "__main__":

    if len(sys.argv) == 4:
        xmlfile = sys.argv[1]
        nw_name = sys.argv[2]
        nodes = create_network(xmlfile, nw_name)

        if sys.argv[3] is not None:
            write_an_extra_file(nodes, sys.argv[3])
    else:
        print("wrong call!")
        print("arguments of this script should be the path to the SUMO network XML-file and "
              "the name of the created network (network files will be created in FLeetPy/data/networks/{network_name}")
