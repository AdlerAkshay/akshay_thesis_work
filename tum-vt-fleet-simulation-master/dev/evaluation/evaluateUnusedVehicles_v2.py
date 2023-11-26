import sys
import os
import geopandas as gpd
from shapely.geometry import Point
from fiona.crs import from_epsg
import pandas as pd

#
MAIN_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(MAIN_DIR)
from src.routing.NetworkBasic import NetworkBasic
from src.misc.globals import *

WGS_CODE = 4326


def get_unused_vehicle_gdf(output_dir):
    scenario_parameters, list_operator_attributes, _ = load_scenario_inputs(output_dir)
    dir_names = get_directory_dict(scenario_parameters)
    nw = NetworkBasic(dir_names[G_DIR_NETWORK])

    op_stat_f = os.path.join(output_dir, "2-0_op-stats.csv")
    op_stats = pd.read_csv(op_stat_f)

    final_state_f = os.path.join(output_dir, "final_state.csv")
    final_state = pd.read_csv(final_state_f)

    used_vids = {vid:1 for vid in op_stats["vehicle_id"].unique()}
    #print(used_vids)
    def isUsed(row):
        if used_vids.get(row["vehicle_id"]) is None:
            return False
        else:
            return True
    unused_f_state = final_state[final_state.apply(isUsed, axis=1) == False]
    print(f"scenario {output_dir}")
    print(f"{unused_f_state.shape[0]}/{final_state.shape[0]} vehicles unused")

    gdf_list = []
    for key, entries in unused_f_state.iterrows():
        end_node = entries["final_node_index"]
        coords = nw.return_node_coordinates(end_node)
        gdf_list.append({"vid" : entries["vehicle_id"], "node": end_node, "geometry":Point(coords)})
    gdf = gpd.GeoDataFrame(gdf_list)

    if "node" in gdf.columns:
        list_per_node = []
        for node, per_node_gdf in gdf.groupby("node"):
            number_unused = per_node_gdf.shape[0]
            geo = per_node_gdf["geometry"].to_list()[0]
            list_per_node.append({"node": node, "number_unused": number_unused, "geometry": geo})
        pn_gdf = gpd.GeoDataFrame(list_per_node)

        crs_f = os.path.join(dir_names[G_DIR_NETWORK], "base", "crs.info")
        if os.path.isfile(crs_f):
            with open(crs_f) as fh_csr:
                epsg_init_str = fh_csr.read().strip()
                epsg_code = int(epsg_init_str.split(":")[1])
            pn_gdf.crs = from_epsg(epsg_code)

        wgs_df = pn_gdf.to_crs(from_epsg(WGS_CODE))
        out_f = os.path.join(output_dir, "unused_vehicles.geojson")
        wgs_df.to_file(out_f, driver="GeoJSON")

        gdf.drop("node", axis=1, inplace=True)
    return gdf


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise IOError("Incorrect number of arguments!")
    scenario_output_dir = sys.argv[1]
    if not os.path.isdir(scenario_output_dir):
        raise IOError(f"Could not find result directory {scenario_output_dir}")
    get_unused_vehicle_gdf(scenario_output_dir)
