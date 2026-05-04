#!/usr/bin/python3
import rclpy, signal
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from armrs_msgs.msg import StateExchange

from functools import partial

import os
import numpy as np
from .yaml_loader import ParamLoader, ScenarioLoader
from .main_controller import Estimation, Controller

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

        self.declare_parameter('robot_ID', 0)
        robot_index = self.get_parameter('robot_ID').get_parameter_value().integer_value
        # self.get_logger().info('Hello robot %d!' % robot_index)


        # Initialize Controller
        # Initiate the estimation and controller for each robot
        self.id = robot_index
        # for id in scenario.list_robot_ID:
        self.robot_est_i = Estimation(robot_index, param)
        self.robot_cont_i = Controller(robot_index, scenario)
        # pass data between Controller & Estimation if needed
        self.robot_est_i.neigh_ids = self.robot_cont_i.neigh_ids


        # for robot_index in scenario.list_robot_ID:
        tb_name = f'tb4_0{robot_index}'

        # Create pose subscribers - Optitrack Motion Capture (VRPN)
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


        self.cmd_vel_i = Twist()
        # Create cmd_vel publisher
        self.get_logger().info(f'Creating Twist cmd_vel publisher: /{tb_name}/cmd_vel')
        self.cmd_vel_pubs_i = self.create_publisher(Twist, '/{}/cmd_vel'.format(tb_name), 1)


        self.state_i = StateExchange()
        # create StateExchange publisher
        self.get_logger().info(f'Creating StateExchange publisher: /{tb_name}/state')
        self.state_pubs_i = self.create_publisher(StateExchange, '/{}/state'.format(tb_name), 1)


        # Subscribe to Neighbour's data 
        # ENFORCE SYNC CALCULATION - by only focusing on past data before flag (not latest)
        self.sync_latest_est = {}

        # DEFINE SUBSCRIBER TO NEIGHBORS
        for robot_index in self.robot_est_i.neigh_ids:
            tb_name = f'tb4_0{robot_index}'

            # Create subscribers for state exchanges information
            self.get_logger().info(f'Creating StateExchange subscriber /{tb_name}/state')
            self.state_sub = self.create_subscription(StateExchange,
                                    f'/{tb_name}/state',
                                    partial(self.neigh_state_callback, index=robot_index),
                                    qos_profile=qos_profile_sensor_data)


        # Set timer for controller loop in each iteration
        self.ROS_RATE = round(1/param.Ts)
        self.Ts = param.Ts
        self.sim_timer = self.create_timer(self.Ts, self.control_loop)
        self.it = 0
        self.check_t = self.time()

    def time(self):
        """Returns the current time in seconds."""
        return self.get_clock().now().nanoseconds / 1e9

    # UPDATE SELF DATA
    def pose_callback(self, msg, index):
        pos, yaw = ros2py.get_pos_yaw(msg)
        # update to estimation
        self.robot_est_i.update_state_reading(np.array([pos.x, pos.y, 0]), yaw)

    def scan_LIDAR_callback(self, msg, index): 
        scan_data, beam_angles = ros2py.get_scan_data(msg)
        if self.robot_est_i.pos is not None:
            self.robot_est_i.update_range_sensors(scan_data, beam_angles)
        # else: no position data yet

    @staticmethod
    # CONSTRUCTING MESSAGES FOR COMMUNICATION EXCHANGE
    def construct_msgs_state(pubs_msg, self_est_dict):
        # publish robot state & command velocity
        ros2py.est2msg_robot_state(pubs_msg, self_est_dict)
        # publish goals & command velocity
        ros2py.est2msg_navigation(pubs_msg, self_est_dict)

        return pubs_msg

    @staticmethod
    # PARSING INFORMATION FROM COMMUNICATION EXCHANGE
    def update_est_neigh_state(self_est_dict, neigh_id, neigh_msg):
        # update neighbour robot's state
        pos, theta = ros2py.get_robot_state(neigh_msg)
        self_est_dict.update_neigh_pose(neigh_id, pos, theta)


    def evaluate_est_sync_validity(self, index):
        recv_time = self.sync_latest_est[index][0]
        msg = self.sync_latest_est[index][1]

        # DETERMINE LATEST SYNC FLAG from current time
        # WARNING: require sync clock throughout the whole distributed systems
        # which is fine on simulation setup
        sync_T = int(self.time() / self.Ts) * self.Ts

        if recv_time < sync_T:
            # data is received before the latest SYNC FLAG (OK to update) 
            self.update_est_neigh_state(self.robot_est_i, index, msg)# Use DATA to UPDATE
        # else: 
            # data is received after the latest SYNC FLAG 
            # leave it stored, to be checked later in main loop (Evaluate on COMPUTATION)       

    def neigh_state_callback(self, msg, index): 
        if (index in self.sync_latest_est):
            self.evaluate_est_sync_validity(index) # Evaluate on RECEIVE
        
        self.sync_latest_est[index] = (self.time(), msg)


    # MAIN LOOP SIMULATOR
    def control_loop(self):

        now = self.time()
        diff = (now - self.check_t)
        if diff > (1.1 * self.Ts):  # Add 10% extra margin
            self.get_logger().info(
                'WARNING loop rate is slower than expected. Period (ms): {:0.2f}'.format(diff * 1000))
        self.check_t = now

        # Showing Time Stamp
        # if (self.it > 0) and (self.it % self.ROS_RATE == 0):
        #     t = self.it * self.Ts
        #     self.get_logger().info('Simulation t = {}s.'.format(t))
        # self.it += 1


        # COMPUTE CONTROL INPUT and PUBLISH CMD_VEL
        # evaluate neighbour data over sync time
        for j in self.robot_est_i.neigh_ids:
            if (j in self.sync_latest_est):
                self.evaluate_est_sync_validity(j) # Evaluate on COMPUTATION

        if self.robot_est_i.pos is not None:
            # TODO: if needed update any estimation calculation here 
            # BEFORE calculating control input

            # Controller part
            vel_command = self.robot_cont_i.compute_control_input(self.robot_est_i)
            lin_vel, ang_vel = self.robot_cont_i.si_to_unicycle(vel_command, 
                                                                    self.robot_est_i.theta, 
                                                                    self.robot_est_i.look_ahead_dist)

            # Sending command data
            self.cmd_vel_i.linear.x = lin_vel
            self.cmd_vel_i.angular.z = ang_vel
            self.cmd_vel_pubs_i.publish( self.cmd_vel_i )

            # Exchange the robot's state - Publish the data
            state_msg = self.construct_msgs_state(self.state_i, self.robot_est_i) 
            self.state_pubs_i.publish( state_msg )


def main(args=None):
    ROS_NODE_NAME = 'mrs_controller'

    print(args)

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
