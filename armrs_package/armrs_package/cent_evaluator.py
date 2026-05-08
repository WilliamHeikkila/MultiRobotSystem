import numpy as np

class CentralizedEvaluator():
    def __init__(self, scenario_dict, param=None):

        self.list_ID = scenario_dict.list_robot_ID

        # Save list of robots within each formation
        self.form_ids = {}
        self.fleet_size = {}
        for f_id in scenario_dict.form_param:
            self.form_ids[f_id] = [int(i) for i in scenario_dict.form_param[f_id]['ids']]
            self.fleet_size[f_id] = len(self.form_ids[f_id])

        # Variable to save formation information
        self.form_cent = {}
        for f_id in self.form_ids:
            self.form_cent[f_id] = None

        # Store density grid and Shapely Voronoi outputs for visualization/control
        self.grid_roi = None
        self.density_map = None
        self.weighted_com = {}  # per-robot weighted center of mass
        self.voronoi_polygons = {}  # robot_id -> Nx2 array of polygon vertices
        self.grid_step = 0.05
        self.density_means = [[0.1, 0.1], [5, 5]]
        self.density_covariance = np.array([[2, 0], [0, 2]])

        # Field bounds for density grid: prefer ParamLoader values when provided
        if param is not None:
            try:
                self.field_bounds = [param.field_x[0], param.field_x[1], param.field_y[0], param.field_y[1]]
            except Exception:
                self.field_bounds = [-1, 8, -1, 8]
        else:
            self.field_bounds = [-1, 8, -1, 8]


    def assess(self, robot_est_dict, field_bounds=None):
        """
        Assess fleet and compute weighted centroids based on non-uniform density Voronoi coverage.

        Args:
            robot_est_dict: dict of Estimation objects
            field_bounds: [xmin, xmax, ymin, ymax] defaults to [-1, 8, -1, 8]
        """
        if field_bounds is None:
            field_bounds = self.field_bounds

        self.field_bounds = field_bounds
        self._build_density_map(field_bounds)
        self.update_formation_centroids(robot_est_dict)

    def _build_density_map(self, field_bounds):
        """Generate the density grid used for non-uniform coverage."""
        grid_step = self.grid_step
        minx, maxx, miny, maxy = field_bounds[0], field_bounds[1], field_bounds[2], field_bounds[3]
        x_grid = np.arange(minx, maxx + 0.5 * grid_step, grid_step)
        y_grid = np.arange(miny, maxy + 0.5 * grid_step, grid_step)
        x_grid = x_grid[x_grid <= maxx]
        y_grid = y_grid[y_grid <= maxy]
        gx, gy = np.meshgrid(x_grid, y_grid)
        self.grid_roi = np.column_stack((gx.ravel(), gy.ravel()))

        self.density_map = self.evaluate_density(self.grid_roi)
        max_density = np.max(self.density_map)
        if max_density > 0:
            self.density_map = self.density_map / max_density  # normalize to max=1

    def evaluate_density(self, points):
        """Evaluate the configured non-uniform density function at grid points."""
        points = np.asarray(points)
        density = np.zeros(points.shape[0])
        for mean in self.density_means:
            density += self._gaussian_density(points, mean, self.density_covariance)
        return density

    def update_formation_centroids(self, robot_est_dict):
        """Update formation centroids from weighted COMs when they are available."""
        for f_id in self.form_ids:
            ids = self.form_ids[f_id]
            valid_coms = [self.weighted_com[i] for i in ids if i in self.weighted_com and self.weighted_com[i] is not None]
            if valid_coms:
                self.form_cent[f_id] = np.mean(valid_coms, axis=0)
            else:
                # Fallback to arithmetic centroid
                sum_of_pos, robot_num = np.zeros(3), 0
                for i in ids:
                    i_pos = robot_est_dict[i].lahead_pos
                    if i_pos is not None:
                        sum_of_pos += i_pos
                        robot_num += 1
                if robot_num > 0:
                    self.form_cent[f_id] = sum_of_pos / robot_num

    def _gaussian_density(self, points, mean, covariance):
        """Evaluate an unnormalised Gaussian density at each mesh point."""
        mean = np.asarray(mean)
        covariance = np.asarray(covariance)
        inv_cov = np.linalg.inv(covariance)
        diff = points - mean
        exponent = -0.5 * np.einsum('ij,jk,ik->i', diff, inv_cov, diff)
        return np.exp(exponent)

    def reset_voronoi_results(self):
        """Clear Voronoi outputs before recomputing Shapely cells."""
        self.voronoi_polygons = {}
        self.weighted_com = {}

    def weighted_centroid_for_cell(self, cell_shape):
        """Compute numerical density-weighted COM inside one Shapely Voronoi cell."""
        if cell_shape is None or cell_shape.is_empty:
            return None
        if self.grid_roi is None or self.density_map is None:
            return None

        in_cell = self._grid_points_in_convex_cell(cell_shape)
        if not np.any(in_cell):
            return None

        points_in_cell = self.grid_roi[in_cell]
        density_in_cell = self.density_map[in_cell]
        density_sum = np.sum(density_in_cell)
        if density_sum <= 0:
            return None

        com = (density_in_cell @ points_in_cell) / density_sum
        return np.array([com[0], com[1], 0])

    def _grid_points_in_convex_cell(self, cell_shape):
        """Return a mask for grid points inside a bounded convex Shapely cell."""
        vertices = np.asarray(cell_shape.exterior.coords[:-1], dtype=float)
        if vertices.shape[0] < 3:
            return np.zeros(self.grid_roi.shape[0], dtype=bool)

        edges = np.roll(vertices, -1, axis=0) - vertices
        rel_points = self.grid_roi[:, None, :] - vertices[None, :, :]
        crosses = edges[None, :, 0] * rel_points[:, :, 1] - edges[None, :, 1] * rel_points[:, :, 0]

        signed_area = 0.5 * np.sum(
            vertices[:, 0] * np.roll(vertices[:, 1], -1)
            - np.roll(vertices[:, 0], -1) * vertices[:, 1]
        )
        eps = 1e-9
        if signed_area >= 0:
            return np.all(crosses >= -eps, axis=1)
        return np.all(crosses <= eps, axis=1)
