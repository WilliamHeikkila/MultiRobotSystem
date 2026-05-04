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


        # Initialize Controller
        # Initiate the estimation and controller for each robot
        self.robot_est, self.robot_cont = {}, {}
        for id in scenario.list_robot_ID:
            self.robot_est[id] = Estimation(id, param)
            self.robot_cont[id] = Controller(id, scenario)
            # TODO: pass data between Controller & Estimation if needed
            self.robot_est[id].neigh_ids = self.robot_cont[id].neigh_ids


        # DEFINE SUBSCRIBER & PUBLISHER        
        self.cmd_vel_pubs = {}
        self.all_cmd_vel = {}

        self.state_pubs = {}
        self.all_state = {}

        for robot_index in scenario.list_robot_ID:
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


            self.all_cmd_vel[robot_index] = Twist()
            # Create cmd_vel publisher
            self.get_logger().info(f'Creating Twist cmd_vel publisher: /{tb_name}/cmd_vel')
            self.cmd_vel_pubs[tb_name] = self.create_publisher(Twist, '/{}/cmd_vel'.format(tb_name), 10)


            self.all_state[robot_index] = StateExchange()
            # create StateExchange publisher
            self.get_logger().info(f'Creating StateExchange publisher: /{tb_name}/state')
            self.state_pubs[tb_name] = self.create_publisher(StateExchange, '/{}/state'.format(tb_name), 1)



        # Set timer for controller loop in each iteration
        self.ROS_RATE = round(1/param.Ts)
        self.Ts = param.Ts
        self.sim_timer = self.create_timer(self.Ts, self.control_loop)
        self.it = 0
        self.check_t = self.time()

    def time(self):
        """Returns the current time in seconds."""
        return self.get_clock().now().nanoseconds / 1e9

    def pose_callback(self, msg, index):
        pos, yaw = ros2py.get_pos_yaw(msg)
        # update to estimation
        self.robot_est[index].update_state_reading( np.array([pos.x, pos.y, 0]), yaw )

    def scan_LIDAR_callback(self, msg, index): 
        scan_data, beam_angles = ros2py.get_scan_data(msg)
        if self.robot_est[index].pos is not None:
            self.robot_est[index].update_range_sensors(scan_data, beam_angles)
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


    # MAIN LOOP SIMULATOR
    def control_loop(self):

        now = self.time()
        diff = (now - self.check_t)
        if diff > (1.1 * self.Ts):  # Add 10% extra margin
            self.get_logger().info(
                'WARNING loop rate is slower than expected. Period (ms): {:0.2f}'.format(diff * 1000))
        self.check_t = now

        # # Showing Time Stamp
        # if (self.it > 0) and (self.it % self.ROS_RATE == 0):
        #     t = self.it * self.Ts
        #     self.get_logger().info('Simulation t = {}s.'.format(t))
        # self.it += 1

        # SIMPLIFIED COMMUNICATION EXCHANGE FOR NOW
        # Update communication from other robots, later merge into callback
        for id in self.all_cmd_vel:
            for neigh_id in self.robot_est[id].neigh_ids:
                # update neighbour's pose 
                if self.robot_est[neigh_id].pos is not None:
                    # In distributed settings this is retrieved from VRPN data
                    self.robot_est[id].update_neigh_pose(neigh_id, self.robot_est[neigh_id].pos, self.robot_est[neigh_id].theta)
                # Update from neighbour StateExchange messages
                self.update_est_neigh_state(self.robot_est[id], neigh_id, self.all_state[neigh_id]) # Use DATA to UPDATE

        # COMPUTE CONTROL INPUT and PUBLISH CMD_VEL
        for id in self.all_cmd_vel:
            # Update the consensus estimation for centroid
            if self.robot_est[id].pos is not None:
                # TODO: if needed update any estimation calculation here 
                # BEFORE calculating control input

                # Controller part
                vel_command = self.robot_cont[id].compute_control_input(self.robot_est[id])
                lin_vel, ang_vel = self.robot_cont[id].si_to_unicycle(vel_command, 
                                                                    self.robot_est[id].theta, 
                                                                    self.robot_est[id].look_ahead_dist)

                # Sending command data
                self.all_cmd_vel[id].linear.x = lin_vel
                self.all_cmd_vel[id].angular.z = ang_vel
                self.cmd_vel_pubs[f"tb4_0{id}"].publish( self.all_cmd_vel[id] )

                # Exchange the robot's state - Publish the data
                self.state_pubs[f"tb4_0{id}"].publish( 
                    self.construct_msgs_state(self.all_state[id], self.robot_est[id]) )


def main(args=None):
    ROS_NODE_NAME = 'mrs_controller'

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
