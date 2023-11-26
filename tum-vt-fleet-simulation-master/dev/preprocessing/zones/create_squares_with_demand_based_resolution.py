import os
import sys
import shutil
import glob
import numpy as np
import pandas as pd
import geopandas as gpd
import matplotlib.pyplot as plt
from shapely.geometry import Point, Polygon
MAIN_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.append(MAIN_DIR)
from src.misc.globals import *
from src.misc.safe_pathname import slugify


class SquaresDemandBasedResolution:
    def __init__(self, max_edge_length, max_zoom_lvls, ratio_first_zoom_lvl, network_base_dir, demand_network_dir,
                 new_zone_system_name, glob_str=None):
        """Initialization of class to create squares in operating area based on demand.
        The given ratio of zones in the lowest resolution with the most demand are represented in higher resolution
        squares. The threshold of demand for further zoom levels is determined by the zone with the most demand that was
        not represented in higher resolution.

        :param max_edge_length: edge length of lowest resolution squares in meters
        :param max_zoom_lvls: maximum number of zoom levels
        :param ratio_first_zoom_lvl: ratio in ]0,1[ of zones that should remain in lowest resolution
        :param network_base_dir: path to network base directory
        :param demand_network_dir: path to matched network dir of demand (with the csv files to be considered)
        :param new_zone_system_name: name of the newly generated zone system
        :param glob_str: can be used to filter demand glob file search
        """
        self.max_edge_length = float(max_edge_length)
        self.max_zoom_lvls = int(max_zoom_lvls)
        if self.max_zoom_lvls < 2:
            raise IOError(f"max_zoom_lvls has to be larger or equal to 2. Input: {max_zoom_lvls}")
        self.ratio_first_zoom_lvl = float(ratio_first_zoom_lvl)
        if 0 >= self.ratio_first_zoom_lvl or 1 <= self.ratio_first_zoom_lvl:
            raise IOError(f"ratio_first_zoom_lvl has to be in ]0,1[. Input: {ratio_first_zoom_lvl}")
        self.network_base_dir = network_base_dir
        self.demand_network_dir = demand_network_dir
        self.nw_name = os.path.basename(demand_network_dir)
        self.new_zone_system_name = slugify(new_zone_system_name)
        self.glob_str = glob_str
        #
        # script flow
        self.node_demand = self.get_node_demand()
        self.total_demand = sum(self.node_demand.values())
        self.node_df = self.load_nodes()
        self.x_min, self.x_max, self.y_min, self.y_max = self.get_boundaries()
        self.zone_gdf = self.create_zones_all_lvl()
        self.selected_zones = self.select_zones_per_lvl_first_zoom_ratio()
        self.selected_zone_gdf = self.zone_gdf[self.zone_gdf.index.isin(self.selected_zones)]
        self.create_centroids_and_write_output()

    def get_node_demand(self):
        """This method creates a dictionary that counts all demands for a node (departure & arrival).

        :return: node_demand: node_index -> demand_count
        :rtype: dict
        """
        print("\n\n... collecting demand per node")
        node_demand_df = None
        if self.glob_str is None:
            list_demand_f = glob.glob(f"{self.demand_network_dir}/*.csv")
        else:
            print(f"using glob-str {self.glob_str}")
            list_demand_f = glob.glob(f"{self.demand_network_dir}/{self.glob_str}")
        number_f = len(list_demand_f)
        counter = 0
        prt_str = ""
        for f in list_demand_f:
            counter += 1
            if counter % 10 == 0:
                print(f"\t ... processing file {counter}/{number_f}")
            tmp_df = pd.read_csv(f)
            # print(tmp_df.head())
            o_series = tmp_df[G_RQ_ORIGIN].value_counts()
            d_series = tmp_df[G_RQ_DESTINATION].value_counts()
            sum_df = pd.concat([o_series, d_series], axis=1)
            sum_df.fillna(0, inplace=True)
            sum_df["add_node_demand"] = sum_df[G_RQ_ORIGIN] + sum_df[G_RQ_DESTINATION]
            if node_demand_df is None:
                node_demand_df = sum_df[["add_node_demand"]]
                node_demand_df.rename({"add_node_demand": "node_demand"}, axis=1, inplace=True)
            else:
                node_demand_df["add_node_demand"] = sum_df["add_node_demand"]
                node_demand_df.fillna(0, inplace=True)
                node_demand_df["node_demand"] = node_demand_df["node_demand"] + node_demand_df["add_node_demand"]
                node_demand_df.drop("add_node_demand", axis=1, inplace=True)
            # print(node_demand_df)
            prt_str = f"File number:{counter}/{number_f} | Number of nodes:{len(node_demand_df)} |" \
                      f" Total demand: {node_demand_df['node_demand'].sum()}\nSummary:\n" \
                      f"{node_demand_df['node_demand'].describe()}"
            # print(prt_str)
        print(prt_str)
        return node_demand_df["node_demand"].to_dict()

    def load_nodes(self):
        """This method loads the network nodes into a dataframe.

        :return: node_df
        :rtype: DataFrame
        """
        print("\n\n... loading node information")
        node_f = os.path.join(self.network_base_dir, "nodes.csv")
        node_df = pd.read_csv(node_f, index_col=0)
        print(f"Number of nodes: {len(node_df)}")
        return node_df

    def get_boundaries(self):
        """This method defines boundaries for lowest resolution such that the min/max nodes have symmetric distance to
        the outer boundary.

        :return: (x_min, x_max, y_min, y_max)
        :rtype: tuple
        """
        print("\n\n... computing boundaries")
        x_min_p = self.node_df[G_NODE_X].min()
        x_max_p = self.node_df[G_NODE_X].max()
        x_delta_p = x_max_p - x_min_p
        number_x = np.ceil(x_delta_p / self.max_edge_length)
        x_total = number_x * self.max_edge_length
        x_add_bound = (x_total - x_delta_p)
        x_min = x_min_p - x_add_bound
        x_max = x_max_p + x_add_bound
        #
        y_min_p= self.node_df[G_NODE_Y].min()
        y_max_p = self.node_df[G_NODE_Y].max()
        y_delta_p = y_max_p - y_min_p
        number_y = np.ceil(y_delta_p / self.max_edge_length)
        y_total = number_y * self.max_edge_length
        y_add_bound = (y_total - y_delta_p)
        y_min = y_min_p - y_add_bound
        y_max = y_max_p + y_add_bound
        print(x_min, x_max, y_min, y_max)
        return x_min, x_max, y_min, y_max

    def create_zones_all_lvl(self):
        """This method creates all zones per lvl and creates a dataframe for it

        :return: zone_gdf [zoom_lvl, i_x, j_y, list_node_ids, share_of_demand, geometry]
        :rtype: GeoDataFrame
        """
        list_zones = []
        all_lvl_edge_length = {}
        print("\n\n... creating zones geometries on all levels")
        for zoom_lvl in range(self.max_zoom_lvls):
            lvl_edge_length = self.max_edge_length / (2**zoom_lvl)
            all_lvl_edge_length[zoom_lvl] = lvl_edge_length
            # number of zones
            n_x = int(np.round((self.x_max - self.x_min) / lvl_edge_length, 0))
            n_y = int(np.round((self.y_max - self.y_min) / lvl_edge_length, 0))
            # geometry of zones
            for i_x in range(n_x):
                for j_y in range(n_y):
                    p0 = (self.x_min + i_x * lvl_edge_length, self.y_min + j_y * lvl_edge_length)
                    p1 = (self.x_min + (i_x + 1) * lvl_edge_length, self.y_min + j_y * lvl_edge_length)
                    p2 = (self.x_min + (i_x + 1) * lvl_edge_length, self.y_min + (j_y + 1) * lvl_edge_length)
                    p3 = (self.x_min + i_x * lvl_edge_length, self.y_min + (j_y + 1) * lvl_edge_length)
                    list_zones.append([zoom_lvl, i_x, j_y, Polygon([p0, p1, p2, p3, p0])])
        zone_gdf = gpd.GeoDataFrame(list_zones, columns=["zoom_lvl", "i_x", "j_y", "geometry"])
        zone_gdf.set_index(["zoom_lvl", "i_x", "j_y"], inplace=True)
        nr_zoom_lvl_zones = zone_gdf.reset_index().groupby('zoom_lvl')["geometry"].count()
        print(f"Number of zones per lvl:\n{nr_zoom_lvl_zones}")
        #
        print("\n\n... creating zones geometries on all levels")
        zone_nodes = {}  # ("zoom_lvl", "i_x", "j_y") -> list nodes
        zone_demand_count = {}  # ("zoom_lvl", "i_x", "j_y") -> demand count
        for node_id, row in self.node_df.iterrows():
            node_demand = self.node_demand.get(node_id, 0)
            diff_x = row[G_NODE_X] - self.x_min
            diff_y = row[G_NODE_Y] - self.y_min
            for zoom_lvl in range(self.max_zoom_lvls):
                lvl_edge_length = all_lvl_edge_length[zoom_lvl]
                i_x = int(diff_x / lvl_edge_length)
                j_y = int(diff_y / lvl_edge_length)
                id_tuple = (zoom_lvl, i_x, j_y)
                try:
                    zone_nodes[id_tuple].append(node_id)
                    zone_demand_count[id_tuple] += node_demand
                except KeyError:
                    zone_nodes[id_tuple] = [node_id]
                    zone_demand_count[id_tuple] = node_demand
        for id_tuple, demand_count in zone_demand_count.items():
            zone_gdf.loc[id_tuple, "share_of_demand"] = 100 * demand_count / self.total_demand
        for id_tuple, list_zone_nodes in zone_nodes.items():
            zone_gdf.loc[id_tuple, "list_nodes"] = ";".join([str(x) for x in list_zone_nodes])
        print("Demand Distribution per lvl:")
        for zoom_lvl, zoom_df in zone_gdf.reset_index().groupby("zoom_lvl"):
            print(f"Zoom Level:{zoom_lvl}")
            print(zoom_df["share_of_demand"].describe())
        return zone_gdf

    def select_zones_per_lvl_first_zoom_ratio(self):
        """This method controls which zones are selected for the zone-system. This method eliminates zones with 0 demand
        and chooses zones according to following procedure:
        The given ratio of zones in the lowest resolution with the most demand are represented in higher resolution
        squares. The threshold of demand for further zoom levels is determined by the zone with the most demand that was
        not represented in higher resolution.

        :return: [(zoom_lvl, i_x, j_y)] of selected zones
        :rtype: list
        """
        print("\n\n... selecting zones for each resolution")
        return_zone_list = []
        # minimal resolution
        tmp_df = self.zone_gdf[["share_of_demand"]]
        min_res_zone_df = tmp_df[tmp_df.index.get_level_values("zoom_lvl") == 0]
        min_res_zone_df.fillna(0, inplace=True)
        non_zero_zone_df = min_res_zone_df[min_res_zone_df["share_of_demand"] > 0].sort_values("share_of_demand")
        number_min_res_zones = int(len(non_zero_zone_df) * self.ratio_first_zoom_lvl)
        demand_share_threshold = non_zero_zone_df["share_of_demand"].to_list()[number_min_res_zones]
        keep_low_res_df = non_zero_zone_df[non_zero_zone_df["share_of_demand"] <= demand_share_threshold]
        return_zone_list.extend(list(keep_low_res_df.index.values))
        # higher resolution
        use_higher_res_df = non_zero_zone_df[non_zero_zone_df["share_of_demand"] > demand_share_threshold]
        for id_tuple in use_higher_res_df.index:
            print(f"\n\n Level 0 zone: {id_tuple}")
            return_zone_list.extend(self._iteratively_create_higher_res_zones(id_tuple, demand_share_threshold))
        print(f"Total number of zones:{len(return_zone_list)} | "
              f"Number of lowest resolution zones:{number_min_res_zones}")
        return return_zone_list

    def create_centroids_and_write_output(self):
        """This method computes the centroids for each selected zone and creates all other outputs required to define
        the zone-system in the correct data-structure.

        :return: None
        """
        print("\n\n... determining node-zone relations and zone centroids")
        # print(self.selected_zone_gdf)
        # zones should be enumerated from 0
        polygon_gdf = self.selected_zone_gdf.reset_index()
        polygon_gdf.index.name = "zone_id"
        polygon_gdf["geo_centroid"] = polygon_gdf["geometry"].centroid
        # zone-node connection
        zone_to_nodes = {}
        node_df_infos = []
        node_is_centroid = {}
        for zone_id, row in polygon_gdf.iterrows():
            polygon_gdf.loc[zone_id, "zone_name"] = f"{row['zoom_lvl']}_{row['i_x']}_{row['j_y']}"
            # beware of cases with 0 or 1 nodes
            try:
                list_nodes = [int(x) for x in str(row["list_nodes"]).split(";")]
            except ValueError:
                list_nodes = []
            zone_to_nodes[zone_id] = list_nodes
            best_node_id = None
            best_distance = None
            for node_id in list_nodes:
                node_pos_x = self.node_df.loc[node_id, G_NODE_X]
                node_pos_y = self.node_df.loc[node_id, G_NODE_Y]
                distance = Point(node_pos_x, node_pos_y).distance(polygon_gdf.loc[zone_id, "geo_centroid"])
                if best_node_id is None or distance < best_distance:
                    best_node_id = node_id
                    best_distance = distance
            polygon_gdf.loc[zone_id, "centroid"] = best_node_id
            node_is_centroid[best_node_id] = 1
            for node_id in list_nodes:
                node_df_infos.append([node_id, zone_id, node_is_centroid.get(node_id, 0)])
        node_zone_df = pd.DataFrame(node_df_infos, columns=["node_index", "zone_id", "is_centroid"])
        node_zone_df.set_index("node_index", inplace=True)
        # add remaining nodes with zone_id -1
        for node_id in self.node_df.index:
            if node_id not in node_zone_df.index:
                node_zone_df.loc[node_id] = [-1, 0]
        node_zone_df.sort_index(inplace=True)
        polygon_gdf.drop(["list_nodes", "geo_centroid"], axis=1, inplace=True)
        #
        print("Summary of 1) number of zones and 2) share of demand per zoom level")
        print(polygon_gdf.groupby("zoom_lvl")["share_of_demand"].count())
        print(polygon_gdf.groupby("zoom_lvl")["share_of_demand"].sum())

        # prepare output
        print("\n\n... creating outputs")
        zone_main_dir = os.path.join(MAIN_DIR, "data", "zones", self.new_zone_system_name)
        zone_nw_dir = os.path.join(zone_main_dir, self.nw_name)
        if not os.path.isdir(zone_nw_dir):
            os.makedirs(zone_nw_dir)
        nw_crs_info = os.path.join(self.network_base_dir, "crs.info")
        if os.path.isfile(nw_crs_info):
            shutil.copy(nw_crs_info, zone_main_dir)
        #
        polygon_f = os.path.join(zone_main_dir, "polygon_definition.geojson")
        polygon_gdf.to_file(polygon_f, driver="GeoJSON")
        print(f"wrote {polygon_f}")
        #
        node_zone_f = os.path.join(zone_nw_dir, "node_zone_info.csv")
        node_zone_df.to_csv(node_zone_f)
        print(f"wrote {node_zone_f}")

    def _iteratively_create_higher_res_zones(self, current_id_tuple, threshold_demand_share):
        """This method creates the four sub-zones for a give current_id_tuple and creates the next level for each
        sub-zone that has a higher demand_share than the threshold_demand_share.

        :param current_id_tuple: current id tuple (zoom_lvl, i_x, j_y) to be considered
        :param threshold_demand_share: threshold demand share determining whether to take zoone or divide it further
        :return: list of id_tuples that should be created
        :rtype: list
        """
        (zoom_lvl, i_x, j_y) = current_id_tuple
        higher_res_zoom_lvl = zoom_lvl + 1
        return_list = []
        zone_1 = (zoom_lvl + 1, 2 * i_x, 2 * j_y)
        zone_2 = (zoom_lvl + 1, 2 * i_x + 1, 2 * j_y)
        zone_3 = (zoom_lvl + 1, 2 * i_x + 1, 2 * j_y + 1)
        zone_4 = (zoom_lvl + 1, 2 * i_x, 2 * j_y + 1)
        list_zones = [zone_1, zone_2, zone_3, zone_4]
        for high_res_id_tuple in list_zones:
            print(high_res_id_tuple)
            if self.zone_gdf.loc[high_res_id_tuple, "share_of_demand"] < threshold_demand_share or\
                    higher_res_zoom_lvl == self.max_zoom_lvls - 1:
                return_list.append(high_res_id_tuple)
            else:
                return_list.extend(self._iteratively_create_higher_res_zones(high_res_id_tuple, threshold_demand_share))
        return return_list


if __name__ == "__main__":
    if len(sys.argv) not in [7, 8]:
        doc_str = SquaresDemandBasedResolution.__init__.__doc__
        raise IOError(f"Incorrect number of arguments!\n\n{doc_str}")
    SquaresDemandBasedResolution(*sys.argv[1:])
