from .nebosim_core.range_sensing import calc_detected_pos
from qpsolvers import Problem, solve_problem

import numpy as np
import math

def calc_lahead_pos(pos, theta, ell):
    return np.array([pos[0] + ell*np.cos(theta), 
                     pos[1] + ell*np.sin(theta), 
                     pos[2] ])    


class Estimation():
    def __init__(self, robot_ID, param_dict):
        self.robot_ID = robot_ID

        # SENSOR-BASED Variables
        self.pos = None
        self.theta = None
        self.range_data = None
        self.range_pos = None
        self.obs_pos = None
        self.target_point_x: float = 0.0
        self.target_point_y: float = 0.0

        self.Ts = param_dict.Ts
        self.look_ahead_dist = param_dict.ell
        self.lahead_pos = None

        sensing_resolution = 360 # normal LiDAR on turtlebot
        self.beam_angles = np.linspace(0., 2*np.pi, num=sensing_resolution, endpoint=False)

        # CONSENSUS-BASED & COMMUNICATION EXCHANGE Variables
        self.neigh_ids = [] # THIS NEED TO BE INIIATED FROM Controller VARIABLE
        self.neighbours_data = {} # store neighbours information

        self.goal = np.zeros(3)
        self.vel_command = np.zeros(3)


    # SENSOR-BASED ESTIMATION
    # ----------------------------------------------------------------------------------
    def update_state_reading(self, pos, theta):
        # receiving accurate position and theta directly
        # e.g., from simulator or motion capture
        self.pos = pos
        self.theta = theta
        self.lahead_pos = calc_lahead_pos(pos, theta, self.look_ahead_dist)


    def update_range_sensors(self, range_data, beam_angles = None):
        self.range_data = range_data
        if beam_angles is None: beam_angles = self.beam_angles
        # compute the position of the detected obstacles
        offset = np.pi/2
        self.range_pos = calc_detected_pos(range_data, self.pos, self.theta + offset, beam_angles)
        # filter valid obs
        self.obs_pos = self.range_pos[range_data > 0.05]
        # self.obs_pos = self.range_pos[range_data < 0.99 * max_value]


    # CONSENSUS-BASED ESTIMATION & COMMUNICATION EXCHANGE
    # ----------------------------------------------------------------------------------
    def update_neigh_pose(self, robot_id, pos, theta):
        neigh_lahead = calc_lahead_pos(pos, theta, self.look_ahead_dist)
        try: # update existing data
            self.neighbours_data[robot_id]['pos'] = pos
            self.neighbours_data[robot_id]['theta'] = theta
            self.neighbours_data[robot_id]['lahead'] = neigh_lahead
        except: # initiate the first time
            self.neighbours_data[robot_id] = {'pos': pos, 'theta':theta, 'lahead': neigh_lahead}

    # TODO - an example below





class Controller():
    def __init__(self, robot_ID, scenario_dict):
        self.robot_ID = robot_ID

        ## ------------------------------------
        # INITIATE ALL VARIABLES FOR CONTROLLER
        ## ------------------------------------
        # Initiate variable to list of in-neighbour
        self.neigh_ids = scenario_dict.get_neigh_ids(robot_ID)

        # Default rectangular field bounds when no parameter object is available.
        self.field_bounds = [-1.0, 8.0, -1.0, 8.0]

        self.current_goal = np.zeros(3)

    def compute_control_input(self, estimation_dict: Estimation):
        current_pos = estimation_dict.pos
        if current_pos is None:
            estimation_dict.goal = self.current_goal
            estimation_dict.vel_command = np.zeros(3)
            return np.zeros(3)

        neighbor_positions = []
        for neigh_id, neigh_data in estimation_dict.neighbours_data.items():
            neigh_pos = neigh_data.get('pos')
            if neigh_pos is not None:
                neighbor_positions.append(neigh_pos[:2])

        neighbor_positions = np.array(neighbor_positions, dtype=float) if len(neighbor_positions) > 0 else np.empty((0, 2), dtype=float)

        target_point = self._compute_density_voronoi_centroid(current_pos[:2], neighbor_positions, self.field_bounds)
        if target_point is None:
            target_point = current_pos[:2]

        estimation_dict.target_point_x = target_point[0]
        estimation_dict.target_point_y = target_point[1]

        dir_vec = target_point - current_pos[:2]
        magnitude = np.linalg.norm(dir_vec)
        if magnitude > 1e-6:
            vel_command_xy = 0.5 * dir_vec
        else:
            vel_command_xy = np.zeros(2)

        vel_command = np.array([vel_command_xy[0], vel_command_xy[1], 0.0])
        estimation_dict.goal = np.array([target_point[0], target_point[1], 0.0])
        estimation_dict.vel_command = vel_command
        return vel_command

    def _compute_density_voronoi_centroid(self, robot_position, neighbor_positions, field_bounds):
        polygon = self._bounded_voronoi_polygon(robot_position, neighbor_positions, field_bounds)
        if polygon is None or polygon.shape[0] < 3:
            return None
        return self._weighted_polygon_centroid(robot_position, polygon)

    def _bounded_voronoi_polygon(self, robot_position, other_positions, field_bounds):
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
            return np.empty((0, 2), dtype=float)

        return np.array(clipped, dtype=float)

    def _weighted_polygon_centroid(self, robot_position, polygon):
        minx, miny = polygon[:, 0].min(), polygon[:, 1].min()
        maxx, maxy = polygon[:, 0].max(), polygon[:, 1].max()
        sample_step = 0.2
        x_vals = np.arange(minx, maxx + sample_step * 0.5, sample_step)
        y_vals = np.arange(miny, maxy + sample_step * 0.5, sample_step)
        if x_vals.size == 0 or y_vals.size == 0:
            return robot_position

        xv, yv = np.meshgrid(x_vals, y_vals)
        candidate_points = np.column_stack((xv.ravel(), yv.ravel()))
        inside_mask = self._points_inside_polygon(candidate_points, polygon)
        if not np.any(inside_mask):
            return robot_position

        points_in_cell = candidate_points[inside_mask]
        density = self._density_function(points_in_cell)
        total_density = np.sum(density)
        if total_density <= 0:
            return np.mean(points_in_cell, axis=0)

        centroid = (density[:, None] * points_in_cell).sum(axis=0) / total_density
        return centroid

    def _density_function(self, points):
        covariance = np.array([[2.0, 0.0], [0.0, 2.0]])
        inv_cov = np.linalg.inv(covariance)
        density = np.zeros(points.shape[0], dtype=float)
        means = [np.array([1.0, 3.5]), np.array([5.0, 5.0])]
        for mean in means:
            diff = points - mean
            exponent = -0.5 * np.einsum('ij,jk,ik->i', diff, inv_cov, diff)
            density += np.exp(exponent)
        max_density = np.max(density) if density.size > 0 else 1.0
        if max_density > 0:
            density /= max_density
        return density

    def _points_inside_polygon(self, points, polygon):
        x = points[:, 0]
        y = points[:, 1]
        inside = np.zeros(points.shape[0], dtype=bool)
        n = polygon.shape[0]

        for i in range(n):
            j = (i - 1) % n
            xi, yi = polygon[i]
            xj, yj = polygon[j]
            intersect = ((yi > y) != (yj > y)) & (
                x < (xj - xi) * (y - yi) / ((yj - yi) + 1e-12) + xi
            )
            inside ^= intersect

        return inside

    @staticmethod
    def si_to_unicycle(u, theta, ell):
        vel_lin = u[0]*np.cos(theta) + u[1]*np.sin(theta)
        vel_ang = (- u[0]*np.sin(theta) + u[1]*np.cos(theta))/ell
        return vel_lin, vel_ang
