#!/usr/bin/python3
import rclpy, signal
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Polygon, Point32
from armrs_msgs.msg import VoronoiData, StateExchange, FleetInformation 

from functools import partial

import os
import numpy as np
from shapely.geometry import MultiPoint, Point
from shapely.geometry import Polygon as ShapelyPolygon
from shapely.ops import voronoi_diagram

from .yaml_loader import ParamLoader, ScenarioLoader
from .visualizer import PlotVisualizer
from .main_controller import Estimation
from .cent_evaluator import CentralizedEvaluator
from .nebosim_core.range_sensing import calc_detected_pos

from . import ROS2_py_common as ros2py


class Computation(Node):
    def __init__(self, ROS_NODE_NAME):
        super().__init__(ROS_NODE_NAME)
        
        # Read yaml parameter from launcher
        self.declare_parameter('param_yaml', '')
        self.declare_parameter('scenario_yaml', '')
        param_file = self.get_parameter('param_yaml').get_parameter_value().string_value
        scenario_file = self.get_parameter('scenario_yaml').get_parameter_value().string_value
        self.get_logger().info('Reading param yaml %s' % param_file)
        self.get_logger().info('Reading scenario yaml %s' % scenario_file)

        # Load parameters from yaml file
        param = ParamLoader(param_file)
        scenario = ScenarioLoader(scenario_file)

        # Initiate the estimation to STORE DATA for each robot
        self.robot_est = {}
        for id in scenario.list_robot_ID:
            self.robot_est[id] = Estimation(id, param)
        # Initiate the fleet evaluator
        self.evaluator = CentralizedEvaluator(scenario, param)
        self.field_bounds = self.evaluator.field_bounds

        # DEFINE SUBSCRIBER
        for robot_index in scenario.list_robot_ID:
            tb_name = f'tb4_0{robot_index}'

            # Create pose subscribers
            vrpn_name = f'tb_0{robot_index}'
            self.get_logger().info(f'Creating pose subscriber /{vrpn_name}/pose')
            self.pose_sub = self.create_subscription(PoseStamped,
                                    f'/vrpn_mocap/{vrpn_name}/pose',
                                    partial(self.pose_callback, index=robot_index),
                                    qos_profile=qos_profile_sensor_data)

            # Create LiDAR subscribers
            self.get_logger().info(f'Creating LiDAR data subscriber: /{tb_name}/scan')
            self.create_subscription(LaserScan,
                                     f'/{tb_name}/scan',
                                     partial(self.scan_LIDAR_callback, index=robot_index),
                                     qos_profile=qos_profile_sensor_data)


            # Create subscribers for state exchanges information
            self.get_logger().info(f'Creating StateExchange subscriber /{tb_name}/state')
            self.state_sub = self.create_subscription(StateExchange,
                                                    f'/{tb_name}/state',
                                                    partial(self.state_callback, index=robot_index),
                                                    qos_profile=qos_profile_sensor_data)

        self.fleet_msg, self.fleet_pubs = {}, {}
        for f_id in self.evaluator.form_ids:
            fleet_name = f'fleet_{f_id}'
            self.fleet_msg[f_id] = FleetInformation()
            # create StateExchange publisher
            self.get_logger().info(f'Creating FleetInformation publisher: /{fleet_name}/diagnosis')
            self.fleet_pubs[f_id] = self.create_publisher(FleetInformation, '/{}/diagnosis'.format(fleet_name), 1)
        
        # Set timer for controller loop in each iteration
        self.ROS_RATE = round(1/param.Ts)
        self.Ts = param.Ts
        self.sim_timer = self.create_timer(self.Ts, self.vis_loop)
        self.it = 0
        self.start_t = self.time()
        self.check_t = self.time()

        #VOVORON
        self.voronoi_pub = self.create_publisher(VoronoiData, "voronoi_data", 1)

        self.list_of_robot_id_voronoi: list = scenario.list_robot_ID

    def time(self):
        """Returns the current time in seconds."""
        return self.get_clock().now().nanoseconds / 1e9

    def pose_callback(self, msg, index):
        pos, yaw = ros2py.get_pos_yaw(msg)
        # update to estimation
        self.robot_est[index].update_state_reading( np.array([pos.x, pos.y, 0]), yaw )

    def state_callback(self, msg, index):
        pass
        # TODO: pass for now
        # IF needed here you can store any data from StateExchange.msg into robot_est[index] 
        # in calculating centralized assessment


    def scan_LIDAR_callback(self, msg, index): 
        scan_data, beam_angles = ros2py.get_scan_data(msg)
        if self.robot_est[index].pos is not None:
            self.robot_est[index].update_range_sensors(scan_data, beam_angles)
        # else: no position data yet


    # MAIN LOOP VISUALIZER
    def vis_loop(self):

        now = self.time()
        diff = (now - self.check_t)
        if diff > (1.1 * self.Ts):  # Add 10% extra margin
            self.get_logger().info(
                'WARNING loop rate is slower than expected. Period (ms): {:0.2f}'.format(diff * 1000))
        self.check_t = now

        self.calculate_voronoi()

        # Publish each fleet data
        for f_id in self.evaluator.form_ids:
            ros2py.cent_evaluator_to_msg(f_id, self.fleet_msg[f_id], self.evaluator)
            self.fleet_pubs[f_id].publish( self.fleet_msg[f_id] )
    
    
    def calculate_voronoi(self):
        self.evaluator.assess(self.robot_est, self.field_bounds)

        active_robot_ids = []
        pos_list = []
        for robot_id, data in self.robot_est.items():
            if data.pos is not None:
                pos_list.append([float(data.pos[0]), float(data.pos[1])])
                active_robot_ids.append(robot_id)

        if len(pos_list) < 2:
            return

        minx, maxx, miny, maxy = self.field_bounds
        boundary_coords = [[minx, miny], [minx, maxy], [maxx, maxy], [maxx, miny]]
        bound_poly = ShapelyPolygon(boundary_coords)
        extension_box = bound_poly.buffer(2.0).envelope
        points = np.asarray(pos_list, dtype=np.float32)
        msg = VoronoiData()

        try:
            self.evaluator.reset_voronoi_results()
            cells = self._bounded_voronoi_cells(points, bound_poly, extension_box)

            for i, cell_shape in enumerate(cells):
                if cell_shape is None:
                    continue

                ros_poly = Polygon()
                vertices = np.asarray(cell_shape.exterior.coords, dtype=float)
                for x, y in vertices:
                    ros_poly.points.append(Point32(x=float(x), y=float(y), z=0.0))

                robot_id = active_robot_ids[i]
                msg.ids.append(robot_id)
                msg.cells.append(ros_poly)
                self.evaluator.voronoi_polygons[robot_id] = vertices

                target = self.evaluator.weighted_centroid_for_cell(cell_shape)
                if target is None:
                    centroid = cell_shape.centroid
                    target = np.array([centroid.x, centroid.y, 0])
                self.evaluator.weighted_com[robot_id] = target

                msg.target_x.append(float(target[0]))
                msg.target_y.append(float(target[1]))
                self.robot_est[robot_id].target_point_x = float(target[0])
                self.robot_est[robot_id].target_point_y = float(target[1])

            self.evaluator.update_formation_centroids(self.robot_est)

            self.voronoi_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Voronoi calculation failed: {e}")

    def _bounded_voronoi_cells(self, points, bound_poly, extension_box):
        """Build Shapely Voronoi cells and map them back to input robot order."""
        diagram = voronoi_diagram(MultiPoint(points), envelope=extension_box)
        diagram_cells = []
        for vor_poly in diagram.geoms:
            cell = self._clip_voronoi_cell(vor_poly, bound_poly)
            if cell is not None:
                diagram_cells.append(cell)

        ordered_cells = []
        used_indices = set()
        for point_xy in points:
            point = Point(float(point_xy[0]), float(point_xy[1]))
            match_idx = None

            for idx, cell in enumerate(diagram_cells):
                if idx not in used_indices and cell.covers(point):
                    match_idx = idx
                    break

            if match_idx is None and diagram_cells:
                unused = [idx for idx in range(len(diagram_cells)) if idx not in used_indices]
                if unused:
                    match_idx = min(unused, key=lambda idx: diagram_cells[idx].distance(point))

            if match_idx is None:
                ordered_cells.append(None)
            else:
                used_indices.add(match_idx)
                ordered_cells.append(diagram_cells[match_idx])

        return ordered_cells

    @staticmethod
    def _clip_voronoi_cell(vor_poly, bound_poly):
        cell_shape = vor_poly.intersection(bound_poly)
        if cell_shape.is_empty:
            return None
        if cell_shape.geom_type == "MultiPolygon":
            cell_shape = max(cell_shape.geoms, key=lambda geom: geom.area)
        if not hasattr(cell_shape, 'exterior'):
            return None
        return cell_shape





def main(args=None):
    ROS_NODE_NAME = 'mrs_fleet_evaluator'

    rclpy.init(args=args)
    node = Computation(ROS_NODE_NAME)
    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
