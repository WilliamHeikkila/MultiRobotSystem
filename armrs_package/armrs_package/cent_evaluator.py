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

        # Store grid, density, and Voronoi data for visualization
        self.grid_roi = None
        self.density_map = None
        self.voronoi_cells = {}  # robot_id -> indices of mesh points in that cell
        self.weighted_com = {}  # per-robot weighted center of mass
        self.vor = None
        self.cell_assignment = None  # robot id assigned to each mesh point
        self.voronoi_polygons = {}  # robot_id -> Nx2 array of polygon vertices

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
        
        # Step 1: Generate mesh grid within field bounds
        grid_step = 0.05
        minx, maxx, miny, maxy = field_bounds[0], field_bounds[1], field_bounds[2], field_bounds[3]
        x_grid = np.arange(minx, maxx + 0.5 * grid_step, grid_step)
        y_grid = np.arange(miny, maxy + 0.5 * grid_step, grid_step)
        x_grid = x_grid[x_grid <= maxx]
        y_grid = y_grid[y_grid <= maxy]
        gx, gy = np.meshgrid(x_grid, y_grid)
        self.grid_roi = np.column_stack((gx.ravel(), gy.ravel()))
        grid_num = self.grid_roi.shape[0]
        
        # Step 2: Assign custom density based on Gaussian function
        list_means = [[0.1, 0.1], [5, 5]]  # Two density peaks
        covariance = np.array([[2, 0], [0, 2]])
        self.density_map = np.zeros(grid_num)
        for mean in list_means:
            self.density_map += self._gaussian_density(self.grid_roi, mean, covariance)
        max_density = np.max(self.density_map)
        if max_density > 0:
            self.density_map = self.density_map / max_density  # normalize to max=1
        
        # Step 3: Assign mesh points to bounded Voronoi cells
        self._compute_voronoi_cells(robot_est_dict, field_bounds)
        
        # Step 4: Compute weighted centroids per robot in their Voronoi cell
        self._compute_weighted_centroids_per_robot(grid_step)
        
        # Step 5: Update formation centroids as average of member robot weighted COMs
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

    def _compute_voronoi_cells(self, robot_est_dict, field_bounds):
        """Assign mesh points to each robot's bounded Voronoi cell."""
        self.voronoi_cells = {}
        self.voronoi_polygons = {}
        self.weighted_com = {}
        self.cell_assignment = None
        self.vor = None

        # Collect all robot positions (use lahead if available)
        robot_positions = []
        robot_ids_valid = []
        for robot_id in self.list_ID:
            robot_est = robot_est_dict.get(robot_id)
            if robot_est is not None and robot_est.lahead_pos is not None:
                robot_positions.append(robot_est.lahead_pos[:2])  # x, y only
                robot_ids_valid.append(robot_id)
        
        if len(robot_positions) == 0:
            return
        
        robot_positions = np.array(robot_positions)

        # Discrete Voronoi: each mesh point belongs to the nearest robot.
        distances = np.linalg.norm(
            self.grid_roi[:, None, :] - robot_positions[None, :, :],
            axis=2,
        )
        nearest_robot_idx = np.argmin(distances, axis=1)
        self.cell_assignment = np.array(
            [robot_ids_valid[i] for i in nearest_robot_idx],
            dtype=int,
        )

        for i, robot_id in enumerate(robot_ids_valid):
            self.voronoi_cells[robot_id] = np.where(nearest_robot_idx == i)[0]
            self.voronoi_polygons[robot_id] = self._bounded_voronoi_polygon(
                robot_positions[i],
                np.delete(robot_positions, i, axis=0),
                field_bounds,
            )

    def _bounded_voronoi_polygon(self, robot_position, other_positions, field_bounds):
        """Clip the rectangular field by pairwise nearest-neighbour half-planes."""
        minx, maxx, miny, maxy = field_bounds
        polygon = np.array([
            [minx, miny],
            [maxx, miny],
            [maxx, maxy],
            [minx, maxy],
        ], dtype=float)

        for other_position in other_positions:
            if np.allclose(robot_position, other_position):
                continue

            normal = 2 * (other_position - robot_position)
            limit = np.dot(other_position, other_position) - np.dot(robot_position, robot_position)
            polygon = self._clip_polygon_halfplane(polygon, normal, limit)
            if polygon.shape[0] == 0:
                return None

        return polygon if polygon.shape[0] >= 3 else None

    def _clip_polygon_halfplane(self, polygon, normal, limit):
        """Sutherland-Hodgman clipping for normal dot point <= limit."""
        clipped = []
        eps = 1e-9

        for i, current in enumerate(polygon):
            previous = polygon[i - 1]
            current_value = np.dot(normal, current) - limit
            previous_value = np.dot(normal, previous) - limit
            current_inside = current_value <= eps
            previous_inside = previous_value <= eps

            if current_inside != previous_inside:
                direction = current - previous
                denominator = np.dot(normal, direction)
                if abs(denominator) > eps:
                    t = (limit - np.dot(normal, previous)) / denominator
                    clipped.append(previous + t * direction)

            if current_inside:
                clipped.append(current)

        if len(clipped) == 0:
            return np.empty((0, 2))

        return np.array(clipped)

    def _compute_weighted_centroids_per_robot(self, grid_step):
        """Compute weighted center of mass for each robot in its Voronoi cell."""
        for robot_id, cell_indices in self.voronoi_cells.items():
            if cell_indices is None or len(cell_indices) == 0:
                self.weighted_com[robot_id] = None
                continue
            
            try:
                density_in_cell = self.density_map[cell_indices]
                points_in_cell = self.grid_roi[cell_indices]
                density_sum = np.sum(density_in_cell)

                if density_sum > 0:
                    com = (density_in_cell @ points_in_cell) / density_sum
                    self.weighted_com[robot_id] = np.array([com[0], com[1], 0])
                else:
                    self.weighted_com[robot_id] = None
            except Exception as e:
                print(f"Error computing weighted COM for robot {robot_id}: {e}")
                self.weighted_com[robot_id] = None
