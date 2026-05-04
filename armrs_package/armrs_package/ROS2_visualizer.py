#!/usr/bin/python3
import rclpy, signal
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from armrs_msgs.msg import StateExchange, FleetInformation

from functools import partial

import os
import numpy as np
from .yaml_loader import ParamLoader, ScenarioLoader
from .visualizer import PlotVisualizer
from .main_controller import Estimation
from .cent_evaluator import CentralizedEvaluator
from .nebosim_core.range_sensing import calc_detected_pos

from . import ROS2_py_common as ros2py

VIDEO_OUT = False
# WARNING: The video output time is slightly faster than the real simulation time
if VIDEO_OUT: from .nebosim_core.video import video

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

        # Initiate visualization for animating the robot movement
        self.plot_vis = PlotVisualizer(param, scenario)
        self.evaluator = CentralizedEvaluator(scenario)

        # OVERWRITE PARAMETERS ON VISUALIZATION
        # By default they all turned on (True)
        # self.plot_vis.SHOW_LIDAR_DETECTION = False
        # self.plot_vis.SHOW_COMMUNICATION = False
        # self.plot_vis.SHOW_ROBOT_GOAL = False
        # self.plot_vis.SHOW_ROBOT_ID = False
        # self.plot_vis.SHOW_ROBOT_SI_VELOCITY = False

        # register and draw static obstacles from scenario settings
        for key in param.obstacles:
            obj_vertices = param.obstacles[key]
            self.plot_vis.ax_2D.plot(obj_vertices[:, 0], obj_vertices[:, 1], 'k')

        # Initiate the estimation to STORE DATA for each robot
        self.robot_est = {}
        for id in scenario.list_robot_ID:
            self.robot_est[id] = Estimation(id, param)


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


        for f_id in self.evaluator.form_ids:
            fleet_name = f'fleet_{f_id}'
            # Create subscribers for fleet information
            self.get_logger().info(f'Creating FleetInformation subscriber /{fleet_name}/diagnosis')
            self.fleet_sub = self.create_subscription(FleetInformation,
                                                    f'/{fleet_name}/diagnosis',
                                                    partial(self.fleet_callback, index=f_id),
                                                    qos_profile=qos_profile_sensor_data)

        # Set timer for controller loop in each iteration
        self.Ts = 0.1 # 10Hz
        self.ROS_RATE = round(1/self.Ts)
        self.sim_timer = self.create_timer(self.Ts, self.vis_loop)
        self.it = 0
        self.start_t = self.time()
        self.check_t = self.time()

        # OVERWRITE plot_vis PARAMETER to adjust with the changed Ts
        self.plot_vis.Ts = self.Ts
        self.plot_vis.tseries_data_num = round(self.plot_vis.time_series_window/self.plot_vis.Ts)
        self.plot_vis.array_time = [None] * self.plot_vis.tseries_data_num

        if VIDEO_OUT:
            # Initiate video output
            self.vid_fname = "sim_video.avi"
            self.video_out = video(self.plot_vis.fig, self.vid_fname, self.ROS_RATE)
            self.get_logger().info(f'Initiate saving video into: {self.vid_fname}')


    def time(self):
        """Returns the current time in seconds."""
        return self.get_clock().now().nanoseconds / 1e9

    def pose_callback(self, msg, index):
        pos, yaw = ros2py.get_pos_yaw(msg)
        # update to estimation
        self.robot_est[index].update_state_reading( np.array([pos.x, pos.y, 0]), yaw )

    def state_callback(self, msg, index):
        # parse navigation data for visualization
        self.robot_est[index].goal, self.robot_est[index].vel_command = \
            ros2py.get_navigation_data(msg)


    def scan_LIDAR_callback(self, msg, index): 
        scan_data, beam_angles = ros2py.get_scan_data(msg)
        if self.robot_est[index].pos is not None:
            self.robot_est[index].update_range_sensors(scan_data, beam_angles)
        # else: no position data yet


    def fleet_callback(self, msg, index):
        ros2py.msg_to_cent_evaluator_data(index, msg, self.evaluator)


    # MAIN LOOP VISUALIZER
    def vis_loop(self):

        now = self.time()
        diff = (now - self.check_t)
        if diff > (1.1 * self.Ts):  # Add 10% extra margin
            self.get_logger().info(
                'WARNING loop rate is slower than expected. Period (ms): {:0.2f}'.format(diff * 1000))
        self.check_t = now

        # Update the plot
        elapsed_time = (now - self.start_t)
        self.plot_vis.update(elapsed_time, self.robot_est, self.evaluator)

        # store frame to video
        if VIDEO_OUT: self.video_out.save_image()


def main(args=None):
    ROS_NODE_NAME = 'mrs_visualizer'

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
