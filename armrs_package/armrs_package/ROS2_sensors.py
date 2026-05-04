#!/usr/bin/python3
import rclpy, signal
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan

from functools import partial

import os
import numpy as np
from .yaml_loader import ParamLoader, ScenarioLoader
from .simulator import Sensor, spawn_robots
from . import ROS2_py_common as ros2py


class Computation(Node):
    def __init__(self, ROS_NODE_NAME):
        super().__init__(ROS_NODE_NAME)
        
        # Read yaml parameter from launcher
        self.declare_parameter('param_yaml', '')
        self.declare_parameter('scenario_yaml', '')
        param_file = self.get_parameter('param_yaml').get_parameter_value().string_value
        scenario_file = self.get_parameter('scenario_yaml').get_parameter_value().string_value
        # self.get_logger().info('Reading param yaml %s' % param_file)
        # self.get_logger().info('Reading scenario yaml %s' % scenario_file)

        # Load parameters from yaml file
        param = ParamLoader(param_file)
        scenario = ScenarioLoader(scenario_file)


        # Initialize Range Finder
        self.robot_list = scenario.list_robot_ID
        self.sensors = Sensor(scenario)

        # register obstacles from scenario settings
        for key in param.obstacles:
            obj_vertices = param.obstacles[key]
            self.sensors.eval_env.register_obstacle_bounded(key, obj_vertices)

        # DEFINE SUBSCRIBER & PUBLISHER
        self.all_scan = {}
        self.scan_pubs = {}        

        for robot_index in scenario.list_robot_ID:
            tb_name = f'tb4_0{robot_index}'

            # Create pose subscribers - Optitrack Motion Capture (VRPN)
            vrpn_name = f'tb_0{robot_index}'
            self.get_logger().info(f'Creating pose subscriber /{vrpn_name}/pose')
            self.pose_sub = self.create_subscription(PoseStamped,
                                    f'/vrpn_mocap/{vrpn_name}/pose',
                                    partial(self.pose_callback, index=robot_index),
                                    qos_profile=qos_profile_sensor_data)

            self.all_scan[robot_index] = LaserScan()

            # create Pose2D publisher
            self.get_logger().info(f'Creating LaserScan publisher: /{tb_name}/scan')
            self.scan_pubs[tb_name] = self.create_publisher(LaserScan, '/{}/scan'.format(tb_name), 1)


        # Set timer for controller loop in each iteration
        self.ROS_RATE = round(1/param.Ts)
        self.Ts = param.LIDAR_Ts
        self.sim_timer = self.create_timer(self.Ts, self.sim_loop)
        self.it = 0
        self.check_t = self.time()

    def time(self):
        """Returns the current time in seconds."""
        return self.get_clock().now().nanoseconds / 1e9

    # BELOW IS PROCESSING poseStamped from VRPN data
    # -----------------------------------------------------------------
    def pose_callback(self, msg, index):
        pos, yaw = ros2py.get_pos_yaw(msg)
        # update to estimation
        self.sensors.update_robot_i(index, np.array([pos.x, pos.y, 0]), yaw)


    # MAIN LOOP SIMULATOR
    def sim_loop(self):

        now = self.time()
        diff = (now - self.check_t)
        if diff > (1.1 * self.Ts):  # Add 10% extra margin
            self.get_logger().info(
                'WARNING loop rate is slower than expected. Period (ms): {:0.2f}'.format(diff * 1000))
        self.check_t = now

        # Sensor data
        for id in self.robot_list:
            sensing_data = self.sensors.get_range_measurement(id)
            # self.get_logger().info(sensing_data)
            self.all_scan[id].angle_min = self.sensors.beam_angles[0]
            self.all_scan[id].angle_max = self.sensors.beam_angles[-1]
            self.all_scan[id].ranges = sensing_data.tolist()

        # PUBLISH each robot TF and sensor reading
        for id in self.robot_list:
            self.scan_pubs[f"tb4_0{id}"].publish( self.all_scan[id] )



def main(args=None):
    ROS_NODE_NAME = 'mrs_sensors'

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
