#!/usr/bin/python3
import rclpy, signal
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, PoseStamped

from functools import partial
import numpy as np


import os
from .yaml_loader import ParamLoader, ScenarioLoader
from .simulator import Sensor, spawn_robots


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


        # Initialize Robot and Range Finder
        self.robot_list = spawn_robots(scenario, param)

        # DEFINE SUBSCRIBER & PUBLISHER
        self.pose_pubs = {}
        self.scan_pubs = {}        
        
        self.all_pose = {}
        self.all_cmd_vel = {}

        for robot_index in scenario.list_robot_ID:
            tb_name = f'tb4_0{robot_index}'

            # Create cmd_vel subscribers
            self.get_logger().info(f'Creating Twist cmd_vel subscriber: /{tb_name}/cmd_vel')
            self.create_subscription(Twist,
                                     f'/{tb_name}/cmd_vel',
                                     partial(self.cmd_vel_callback, index=robot_index),
                                     qos_profile=qos_profile_sensor_data)

            # Create PoseSTamped publisher - Optitrack Motion Capture (VRPN)
            vrpn_name = f'tb_0{robot_index}'
            self.all_pose[robot_index] = PoseStamped()
            self.get_logger().info(f'Creating pose publisher /vrpn_mocap/{vrpn_name}/pose')
            self.pose_pubs[tb_name] = self.create_publisher(PoseStamped, f'/vrpn_mocap/{vrpn_name}/pose', 1)


        # Set timer for controller loop in each iteration
        self.ROS_RATE = round(1/param.Ts)
        self.Ts = param.Ts
        self.sim_timer = self.create_timer(self.Ts, self.sim_loop)
        self.it = 0
        self.check_t = self.time()

    def time(self):
        """Returns the current time in seconds."""
        return self.get_clock().now().nanoseconds / 1e9

    def cmd_vel_callback(self, msg, index): 
        self.all_cmd_vel[index] = msg
        self.robot_list[index].set_input_unicycle(msg.linear.x, msg.angular.z)


    # MAIN LOOP SIMULATOR
    def sim_loop(self):

        now = self.time()
        diff = (now - self.check_t)
        if diff > (1.1 * self.Ts):  # Add 10% extra margin
            self.get_logger().info(
                'WARNING loop rate is slower than expected. Period (ms): {:0.2f}'.format(diff * 1000))
        self.check_t = now

        # Showing Time Stamp
        if (self.it > 0) and (self.it % self.ROS_RATE == 0):
            t = self.it * self.Ts
            self.get_logger().info('Simulation t = {}s.'.format(t))
        self.it += 1

        # Update Simulation and PUBLISH each robot TF and sensor reading
        for id, robot in self.robot_list.items():
            robot.update()
            # Sensor data - representation in quaternion
            # Source: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
            self.all_pose[id].pose.position.x = robot.state["pos"][0]
            self.all_pose[id].pose.position.y = robot.state["pos"][1]
            self.all_pose[id].pose.position.z = 0.
            r, p, y = 0., 0., robot.state["theta"]
            cr, sr = np.cos(0.5 * r), np.sin(0.5 * r)
            cp, sp = np.cos(0.5 * p), np.sin(0.5 * p)
            cy, sy = np.cos(0.5 * y), np.sin(0.5 * y)
            self.all_pose[id].pose.orientation.w = cr * cp * cy + sr * sp * sy 
            self.all_pose[id].pose.orientation.x = sr * cp * cy - cr * sp * sy 
            self.all_pose[id].pose.orientation.y = cr * sp * cy + sr * cp * sy 
            self.all_pose[id].pose.orientation.z = cr * cp * sy - sr * sp * cy 

            self.pose_pubs[f"tb4_0{id}"].publish( self.all_pose[id] )



def main(args=None):
    ROS_NODE_NAME = 'mrs_simulator'

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
