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
import shapely
from shapely.geometry import MultiPoint, Point
from shapely.geometry import Polygon as ShapelyPolygon
from shapely.plotting import plot_polygon

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
        self.evaluator = CentralizedEvaluator(scenario)

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

        self.calculate_voronoi()

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

        self.evaluator.assess(self.robot_est)

        # Publish each fleet data
        for f_id in self.evaluator.form_ids:
            ros2py.cent_evaluator_to_msg(f_id, self.fleet_msg[f_id], self.evaluator)
            self.fleet_pubs[f_id].publish( self.fleet_msg[f_id] )
        
        self.calculate_voronoi()
    
    
    def calculate_voronoi(self):
        boundary = np.array([
            [-0.2, -1.0], 
            [-0.2, 7.2], 
            [7.0, 7.2], 
            [7.0, -1.0]
        ])

        bound_poly: ShapelyPolygon = ShapelyPolygon(boundary)
        
        points: np.array = np.array([])

        pos_list = []
        for data in self.robot_est.values():
            if data.pos is None:
                continue
            pos_list.append(data.pos)

        points = np.asarray(pos_list, dtype=np.float32)
        
        vor = {}
        msg: VoronoiData = VoronoiData()

        vor_ori = shapely.voronoi_polygons(MultiPoint(points), extend_to=bound_poly, ordered=True)
        #for i, vor_poly in enumerate(vor_ori.geoms):
        #    vor[i] = vor_poly.intersection(bound_poly)
        #    msg.cells.append(vor_poly.intersection(bound_poly))
        #    msg.ids.append(i)
        
        for i, vor_poly in enumerate(vor_ori.geoms):
            # 3. Intersect and keep valid
            cell_shape = vor_poly.intersection(bound_poly)
            
            # 4. CONVERT Shapely Polygon -> ROS geometry_msgs/Polygon
            ros_poly = Polygon()
            
            # Check if intersection produced a valid polygon with an exterior
            if hasattr(cell_shape, 'exterior'):
                for x, y in cell_shape.exterior.coords:
                    p = Point32()
                    p.x = float(x)
                    p.y = float(y)
                    p.z = 0.0
                    ros_poly.points.append(p)
            
            msg.cells.append(ros_poly)
            msg.ids.append(i)
        
        if len(pos_list) < 4:
            return
        self.voronoi_pub.publish(msg)
        





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
