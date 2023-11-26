"""
Computes the correlations between zones based on a linearly decreasing Kernel with bandwidth h:
K(x,x_i) = |x-x_i|/h

V1: Assume that demand and supply are concentrated in the center of the zone. Script uses first centroid per zone. If no
    centroid is found, no correlation with other zones is assumed.
V2: Assume that demand and supply are uniformly distributed in zones. The uniform distribution is approximated by
    creating a grid and putting the same weight on all grid points of the same zone.

In both cases, the normalization process considers boundary effects (integral on points outside of the operating area)
by creating a grid that goes beyond the operating area boundary by 3*h.

The integrals over zones are approximated by the mean value of all points within a zone.

The outputs are three matrices:
1) a (possibly) asymmetric matrix K^{z',z}, which describes the impact of zone z' on zone z
2) a symmetric matrix K2^{z',z} = sum_{z''} K2^{z',z''} K2^{z'',z}, which can be used for squared objective values
3) a GeoDataFrame with squares around each grid point x (with zone association) for contour plots
4) a matrix K^I(x;z'), which describes the imbalance density function on grid point x created by an imbalance in zone z'
"""
import os
import numpy as np
import geopandas as gpd
from shapely.geometry import Point, Polygon
import multiprocessing as mp
import scipy.sparse

# number of nearest zones that should be considered as centroid
CHECK_NUMBER_ZONES = 8
# methodology -> constant number steps or step size
grid_methods = ["constant_step_size", "constant_number_steps"]
GRID_METHOD = grid_methods[0]
# step size of grid
GRID_STEP_SIZE = 100
# number of steps in x or y direction (the longer extent determines the step size)
NUMBER_STEPS = 400
# for performance testing
# NUMBER_STEPS = 100


def compute_single_zones_correlation_matrices(list_iz_bandwidth, grid_gdf, grid_step_size, zone_center_series, mode):
    """This method first approximates the density function K^I(x;z') for all grid points x by computing unit
        imbalances for each single zone z'.
        Based on this the correlation matrix entries K_iz,z are computed.

    :param list_iz_bandwidth: list of source (zone, bandwidth) tuples that should be treated
    :param grid_gdf: GeoDataFrame with column "Point" with grid points geometry and "zone_id" with associated zone
    :param grid_step_size: step size of grid
    :param zone_center_series: GeoSeries with zone centroids as geometry
    :param mode: 'V1' or 'V2'
    :return: {}: iz -> (K^I(x;iz), K_iz,z) for single zone with imbalance iz, where
                K^I(x;iz) is a dict {}: x -> value
                K_iz,z is a dict {}: z -> value
    :rtype: dict
    """
    #   4a) create unit imbalance density function and compute Zone2Grid correlation matrix K_Z_GP
    #       -> normalize density on points in operating area
    #       for each zone z (same or other)
    #           4b) approximate integral by taking sum of density function values of grid points of z
    return_dicts = {}
    grid_p_list = np.array([[p.x, p.y] for p in grid_gdf["point"]])
    nr_grid_p = len(grid_p_list)
    nr_zones = len(zone_center_series)
    zone_grid_p_dict = {}
    zone2gridid_dict = {}
    for zone_id, tmp_df in grid_gdf.groupby("zone_id"):
        zone_grid_p_dict[zone_id] = np.array([[p.x, p.y] for p in tmp_df["point"]])
        zone2gridid_dict[zone_id] = [int(x) for x in tmp_df.index]
    counter = 0
    total = len(list_iz_bandwidth)
    for iz_bw_tuple in list_iz_bandwidth:
        counter += 1
        iz, bandwidth_iz, cpu = iz_bw_tuple
        if mode == 'V1':
            imbalance_source_list = [zone_center_series[iz]]
            imbalance_source_x = np.array([p.x for p in imbalance_source_list])
            imbalance_source_y = np.array([p.y for p in imbalance_source_list])
        else:
            imbalance_source_points = zone_grid_p_dict[iz]
            imbalance_source_x = np.array([p[0] for p in imbalance_source_points])
            imbalance_source_y = np.array([p[1] for p in imbalance_source_points])
        exact_norm_factor = grid_step_size**2 * (3 / (len(imbalance_source_x) * np.pi * bandwidth_iz ** 2))

        def zone_kernel(x,y):
            """Creation of zone-imbalance kernel

            :param x: x-position
            :param y: y-position
            :return: density function value
            """
            grid = np.append((x - imbalance_source_x).reshape(-1, 1), (y - imbalance_source_y).reshape(-1, 1), axis=1)
            norm2 = np.linalg.norm(grid, axis=1).flatten() / bandwidth_iz
            z = np.zeros(norm2.shape)
            z[norm2 < 1] = 1 - norm2[norm2 < 1]
            return sum(z)

        # creation of iz-2-grid_point correlation matrix
        K_I_iz_x = {}
        unscaled_K_I_iz_x = np.array([zone_kernel(*gp) for gp in grid_p_list])
        grid_norm_factor = 1 / sum(unscaled_K_I_iz_x)
        for gp_index in range(nr_grid_p):
            if unscaled_K_I_iz_x[gp_index] != 0:
                K_I_iz_x[gp_index] = unscaled_K_I_iz_x[gp_index] * grid_norm_factor
        # creation of iz-2-zone correlation matrix
        K_iz_z = {}
        for zone_id in range(nr_zones):
            grid_point_indices = zone2gridid_dict.get(zone_id, [])
            sum_values = sum([K_I_iz_x.get(gp_index, 0) for gp_index in grid_point_indices])
            if sum_values > 0:
                K_iz_z[zone_id] = sum_values
        # record and add to output
        prt_str = f"\t\t proc {cpu} progress: zone {counter} / {total} | " \
                  f"used/exact norm-factor {grid_norm_factor/exact_norm_factor} (grid) | " \
                  f"entries in zone2zone correlation matrix: {len(K_iz_z)}"
        print(prt_str)
        return_dicts[iz] = (K_I_iz_x, K_iz_z)
    return return_dicts


def create_relation_and_grid_p_square(list_tuples, grid, grid_step_size, zone_center_series, zone_gdf):
    """This function creates a part of the relations between zones and grid points.

    :param list_tuples: cpu, x_i, y_j tuples
    :param grid: created mesh
    :param grid_step_size: required to build square around integration point (for visualization purposes)
    :param zone_center_series: GeoSeries with zone center coordinates
    :param zone_gdf: GeoDataFrame with zone polygons
    :return: list of (x_i, y_j, zone_id, center_point, square)
    """
    cpu_total = len(list_tuples)
    part_grid_geo_list = []
    counter = 0
    for cij_tuple in list_tuples:
        counter += 1
        cpu, x_i, y_j = cij_tuple
        if counter % 1000 == 0:
            print(f"\t ... cpu {cpu}: grid point {counter} / {cpu_total}")
        px, py = grid[0][x_i, y_j], grid[1][x_i, y_j]
        p = Point(px, py)
        offset = grid_step_size / 2
        grid_p0 = (px - offset, py - offset)
        grid_p1 = (px + offset, py - offset)
        grid_p2 = (px + offset, py + offset)
        grid_p3 = (px - offset, py + offset)
        polygon = Polygon([grid_p0, grid_p1, grid_p2, grid_p3])
        distance_series = zone_center_series.distance(p)
        sorted_distance_series = distance_series.sort_values()
        # also assign to a zone if a corner is in within the nearest zone
        # (there could be problem if point is exactly on boundary of two zones)
        found_solution = False
        for test_p in [p, Point(grid_p0), Point(grid_p1), Point(grid_p2), Point(grid_p3)]:
            inner_counter = 0
            for zone_id, distance in sorted_distance_series.iteritems():
                # points can also be without operating area and not part of any zone
                if test_p.within(zone_gdf.loc[zone_id, "geometry"]):
                    # # create relations
                    # grid2zone[(x_i, y_j)] = nearest_zone
                    # try:
                    #     zone2grid[nearest_zone].append((x_i, y_j))
                    # except KeyError:
                    #     zone2grid[nearest_zone] = [(x_i, y_j)]

                    # create square around point
                    part_grid_geo_list.append([x_i, y_j, zone_id, p, polygon])
                    found_solution = True
                    break
                inner_counter += 1
                if inner_counter == CHECK_NUMBER_ZONES:
                    break
            if found_solution:
                break
    return part_grid_geo_list


class DensityComputation:
    def __init__(self, zone_main_directory, constant_bandwidth, list_differing_bandwidths=[], mode="V1", ncpu=1):
        """Initialization of all attributes based on input.
        Mode 'V1' relates to imbalance concentrated in center point.
        Mode 'V2' relates to imbalance uniformly distributed in zone.

        :param zone_main_directory: path to zone main directory.
        :type zone_main_directory: str
        :param constant_bandwidth: constant radius of kernel for all points in meters
        :type constant_bandwidth: float
        :param list_differing_bandwidths: different bandwidths for each defined zone
        :type list_differing_bandwidths: list
        :param mode: flag to trigger different creation modes
        :type mode: str
        """
        self.zone_main_directory = zone_main_directory
        self.n_cpu = int(ncpu)
        if self.n_cpu < 1:
            raise IOError("Cannot compute with less than 1 CPU!")
        if self.n_cpu > mp.cpu_count():
            raise IOError(f"Cannot compute with more than {mp.cpu_count()} CPUs!")
        if mode == "V1":
            print("Assume that demand and supply are concentrated in the center of the zone.")
            self.mode = "V1"
        elif mode == "V2":
            print("Assume that demand and supply are uniformly distributed in zones. "
                  "The uniform distribution is approximated by creating a grid and putting"
                  "the same weight on all grid points of the same zone.")
            self.mode = "V2"
        else:
            raise IOError(f"mode={mode} - Only modes 'V1' and 'V2' implemented!")
        # 0a) check file existence
        gf = os.path.join(zone_main_directory, "polygon_definition.geojson")
        if os.path.isfile(gf):
            self.zone_gdf = gpd.read_file(gf)
        else:
            raise IOError(f"Could not find file {gf}!")
        self.zone_gdf["zone_centers"] = self.zone_gdf["geometry"].centroid
        self.zone_center_series = gpd.GeoSeries(self.zone_gdf["geometry"].centroid)
        self.number_zones = len(self.zone_gdf)
        self.bound_min_x, self.bound_min_y, self.bound_max_x, self.bound_max_y = self.zone_gdf.geometry.total_bounds
        # 0b) sanity check for metric system (check both coordinates with 180 degrees as order is not clear)
        if -180 <= self.bound_min_x <= self.bound_max_x <= 180 and -180 <= self.bound_min_y <= self.bound_max_y <= 180:
            raise AssertionError(f"Please make sure that {gf} is given in a metric system!")
        # 1) set bandwidth of each zone
        if list_differing_bandwidths:
            if type(list_differing_bandwidths) != list:
                raise AssertionError(f"list_differing_bandwidths argument has to be of type list!")
            self.bandwidths = [float(x) for x in list_differing_bandwidths]
            nr_bandwidths = len(self.bandwidths)
            if nr_bandwidths != self.number_zones:
                raise AssertionError(f"Number of given bandwidths {nr_bandwidths} != "
                                     f"number of defined zones {self.number_zones}!")
            min_bw = min(self.bandwidths)
            max_bw = max(self.bandwidths)
            self.bw_str = f"vbw_{min_bw}-{max_bw}"
        else:
            self.bandwidths = [float(constant_bandwidth)] * self.number_zones
            self.bw_str = f"cbw_{constant_bandwidth}"
        # 2) creating grid and define grid-zone relations
        if GRID_METHOD == "constant_number_steps":
            f_name =  f"grid_point_squares_{NUMBER_STEPS}steps.geojson"
        else:
            f_name = f"grid_point_squares_{GRID_STEP_SIZE}m.geojson"
        self.gp_f = os.path.join(self.zone_main_directory, f_name)
        if os.path.isfile(self.gp_f):
            # set unnecessary attributes to None
            self.grid = None
            self.nr_x = None
            self.nr_y = None
            self.grid_gdf, self.grid_step_size = self.load_grid()
        else:
            self.grid, self.nr_x, self.nr_y, self.grid_step_size = self.create_grid()
            self.grid_gdf = self.find_grid_zone_relations_and_create_grid_gdf()
        self.nr_grid_points = len(self.grid_gdf)
        # 3) compute zone2grid and zone2zone correlation matrices
        self.K_I_iz_x, self.K_iz_z = self.compute_correlation_matrices()
        # 4) compute squared correlation matrix
        self.K2 = np.matmul(self.K_iz_z, self.K_iz_z.transpose())
        # 5) save everything
        self.write_output()

    def create_grid(self):
        """Generates the grid for the density map
         The grid is generated by taking the minimum and maximum x and y within 3*bandwidth limit

        :return: a tuple of x and y grid points
        """
        print("Creating grid ...")
        # get limits with some extra
        max_bandwidth = max(self.bandwidths)
        min_x = self.bound_min_x - 3 * max_bandwidth
        max_x = self.bound_max_x + 3 * max_bandwidth
        min_y = self.bound_min_y - 3 * max_bandwidth
        max_y = self.bound_max_y + 3 * max_bandwidth
        # define step size and modify max/min points slightly to have them on grid
        diff_x = max_x - min_x
        diff_y = max_y - min_y
        if diff_x >= diff_y:
            if GRID_METHOD == "constant_number_steps":
                step_size = diff_x / NUMBER_STEPS
            else:
                step_size = GRID_STEP_SIZE
            mod_diff_y = np.ceil(diff_y / step_size) * step_size - diff_y
            min_y -= mod_diff_y/2
            max_y += mod_diff_y/2
        else:
            if GRID_METHOD == "constant_number_steps":
                step_size = diff_y / NUMBER_STEPS
            else:
                step_size = GRID_STEP_SIZE
            mod_diff_x = np.ceil(diff_x / step_size) * step_size - diff_x
            min_x -= mod_diff_x/2
            max_x += mod_diff_x/2
        # build grid
        lin_space_x = np.arange(min_x, max_x+step_size, step_size)
        nr_x = len(lin_space_x)
        lin_space_y = np.arange(min_y, max_y+step_size, step_size)
        nr_y = len(lin_space_y)
        print(f"\t ... created a mesh of {nr_x} x {nr_y} points with edge lengths of {step_size} meters")
        return np.meshgrid(lin_space_x, lin_space_y, indexing="ij"), nr_x, nr_y, step_size

    def find_grid_zone_relations_and_create_grid_gdf(self):
        """This method checks which grid points belong to which zones. Moreover, it builds regular squares around the
        grid points, which can be used to visualize the imbalance density.

        :return: two dictionaries with grid2zone and zone2grid relations
        """
        print("Creating relation of grid and zones on {self.n_cpu} CPU(s) ...")
        # grid2zone = {}
        # zone2grid = {}
        argument_list = []
        list_tuples_per_cpu = {}
        for cpu in range(self.n_cpu):
            list_tuples_per_cpu[cpu] = []
        counter = 0
        for x_i in range(self.nr_x):
            for y_j in range(self.nr_y):
                cpu = counter % self.n_cpu
                list_tuples_per_cpu[cpu].append((cpu, x_i, y_j))
                counter += 1
        for cpu in range(self.n_cpu):
            argument_list = [(list_tuples_per_cpu[cpu], self.grid, self.grid_step_size, self.zone_center_series,
                              self.zone_gdf) for cpu in range(self.n_cpu)]
        #
        # SP or MP
        if self.n_cpu == 1:
            list_tuples = list_tuples_per_cpu[0]
            grid_geo_list = create_relation_and_grid_p_square(list_tuples, self.grid, self.grid_step_size,
                                                              self.zone_center_series, self.zone_gdf)
        else:
            mp_pool = mp.Pool(self.n_cpu)
            nested_return = mp_pool.starmap(create_relation_and_grid_p_square, argument_list)
            grid_geo_list = []
            for tmp in nested_return:
                grid_geo_list.extend(tmp)
        #
        grid_gdf = gpd.GeoDataFrame(grid_geo_list, columns=["x_i", "y_j", "zone_id", "point", "geometry"])
        sorted_grid_gdf = grid_gdf.sort_values(["x_i", "y_j"])
        return sorted_grid_gdf

    def load_grid(self):
        """This method reads an existing grid file and returns the step size.

        :return: grid_gdf, grid_step_size
        """
        print(f"Loading grid from {self.gp_f}")
        grid_gdf = gpd.read_file(self.gp_f)
        # pick point in mid and check step in x-direction
        tmp_x_i = grid_gdf["x_i"].quantile(0.5)
        tmp_x_ip1 = tmp_x_i + 1
        tmp_x = grid_gdf[(grid_gdf["x_i"] == tmp_x_i)]["Point_x"].mean()
        tmp_xp1 = grid_gdf[(grid_gdf["x_i"] == tmp_x_ip1)]["Point_x"].mean()
        grid_step_size = tmp_xp1 - tmp_x
        grid_gdf["point"] = grid_gdf.apply(lambda p: Point(p["Point_x"], p["Point_y"]), axis=1)
        grid_gdf.drop(["Point_x", "Point_y"], axis=1, inplace=True)
        print(f"\t ... loaded a mesh with edge lengths of {grid_step_size} meters")
        return grid_gdf, grid_step_size

    def compute_correlation_matrices(self):
        """This method first approximates the density function K^I(x;z') for all grid points x by computing unit
        imbalances for each single zone z'.
        Based on this the correlation matrix K_zz' is computed.

        Parallel implementation to speed up processing.

        :return: K^I(x;z'), K_zz'
        """
        # prepare for possible multiprocessing
        # -> distribute zones originating imbalance among and respective zone bandwidths CPUs
        print(f"Computing correlation matrices on {self.n_cpu} CPU(s) ...")
        iz2cpu = {}
        argument_list = []
        for cpu in range(self.n_cpu):
            iz2cpu[cpu] = []
        for iz in range(self.number_zones):
            cpu = np.mod(iz, self.n_cpu)
            iz2cpu[cpu].append((iz, self.bandwidths[iz], cpu))
        for cpu in range(self.n_cpu):
            argument_list = [(iz2cpu[cpu], self.grid_gdf, self.grid_step_size, self.zone_center_series, self.mode)
                             for cpu in range(self.n_cpu)]
        # SP or MP
        if self.n_cpu == 1:
            iz2cpu = iz2cpu[0]
            iz2matrices = compute_single_zones_correlation_matrices(iz2cpu, self.grid_gdf, self.grid_step_size,
                                                                    self.zone_center_series, self.mode)
        else:
            mp_pool = mp.Pool(self.n_cpu)
            nested_return = mp_pool.starmap(compute_single_zones_correlation_matrices, argument_list)
            iz2matrices = {}
            for tmp in nested_return:
                iz2matrices.update(tmp)
        # build numpy matrices from return
        K_I_iz_x = np.zeros((self.number_zones, self.nr_grid_points))
        K_iz_z = np.zeros((self.number_zones, self.number_zones))
        for iz in range(self.number_zones):
            iz_x_dict = iz2matrices[iz][0]
            iz_z_dict = iz2matrices[iz][1]
            for x in range(self.nr_grid_points):
                K_I_iz_x[iz, x] = iz_x_dict.get(x, 0)
            for z in range(self.number_zones):
                K_iz_z[iz, z] = iz_z_dict.get(z, 0)
        return K_I_iz_x, K_iz_z

    def write_output(self):
        """This method creates output in the main zone directory.

        :return: None
        """
        print("Saving output ...")
        bw_m_str = self.bw_str + "_" + self.mode
        # grid point gdf
        if not os.path.isfile(self.gp_f):
            self.grid_gdf["Point_x"] = self.grid_gdf["point"].apply(lambda p:p.x)
            self.grid_gdf["Point_y"] = self.grid_gdf["point"].apply(lambda p:p.y)
            self.grid_gdf.drop("point", axis=1, inplace=True)
            self.grid_gdf.to_file(self.gp_f, driver="GeoJSON")
        # source zone -> grid correlation matrix
        # --------------------------------------
        # sz_g_c_f = os.path.join(self.zone_main_directory, f"zone_to_grid_correlations_{bw_m_str}.npy")
        # np.save(sz_g_c_f, self.K_I_iz_x)
        sz_g_c_sparse_f = os.path.join(self.zone_main_directory, f"zone_to_grid_correlations_{bw_m_str}.npz")
        K_I_iz_x_sparse = scipy.sparse.csr_matrix(self.K_I_iz_x)
        scipy.sparse.save_npz(sz_g_c_sparse_f, K_I_iz_x_sparse)
        # source zone -> zone correlation matrix
        # --------------------------------------
        # sz_z_c_f = os.path.join(self.zone_main_directory, f"zone_to_zone_correlations_{bw_m_str}.npy")
        # np.save(sz_z_c_f, self.K_iz_z)
        sz_z_c_sparse_f = os.path.join(self.zone_main_directory, f"zone_to_zone_correlations_{bw_m_str}.npz")
        K_iz_z_sparse = scipy.sparse.csr_matrix(self.K_iz_z)
        scipy.sparse.save_npz(sz_z_c_sparse_f, K_iz_z_sparse)
        # squared zone -> zone correlation matrix
        # ---------------------------------------
        # sz_z_sc_f = os.path.join(self.zone_main_directory, f"zone_to_zone_squared_correlations_{bw_m_str}.npy")
        # np.save(sz_z_sc_f, self.K2)
        sz_z_sc_sparse_f = os.path.join(self.zone_main_directory, f"zone_to_zone_squared_correlations_{bw_m_str}.npz")
        K2_sparse = scipy.sparse.csr_matrix(self.K2)
        scipy.sparse.save_npz(sz_z_sc_sparse_f, K2_sparse)


if __name__ == "__main__":
    # reset parameters after use in order to not overwrite GIT with every use.
    zmd = r"C:\Users\ne53qez\Data_and_Simulation\tum-vt-fleet-simulation\data\zones\MUC_A99_max2km_4lvl_mivoev"
    cb = 2000
    ldb = []
    method = "V2"
    n_proc = 4
    DensityComputation(zone_main_directory=zmd, constant_bandwidth=cb, list_differing_bandwidths=ldb, mode=method,
                       ncpu=n_proc)
