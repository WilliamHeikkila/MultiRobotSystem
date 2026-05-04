from .nebosim_core.model import Unicycle
from .nebosim_core.range_sensing import DetectObstacle2D, calc_robot_circ_bounds

import numpy as np

def spawn_robots(scenario_dict, param_dict):
    robot_list = {}
    for id in scenario_dict.list_robot_ID:
        robot_list[id] = Unicycle(param_dict.Ts, 
                                  init_pos=scenario_dict.init_pos[id], init_theta=scenario_dict.init_theta[id], 
                                  look_ahead_dist=param_dict.ell, robot_ID=id)
    return robot_list


class Sensor():
    def __init__(self, scenario_dict):
        self.robot_pos = {}
        self.robot_theta = {}
        # self.offset_lidar = 0.
        self.offset_lidar = np.pi/2 # Setup on Turtlebot4 calculated from robot body X-axis

        # Initiate the environment evaluator for range sensor
        self.eval_env = DetectObstacle2D()
        # Specification of the range sensor model
        sensing_resolution, self.max_value = 360, 3. # assuming 360 resolution with maximum 3m reading
        self.beam_angles = np.linspace(0., 2*np.pi, num=sensing_resolution, endpoint=False)

        # Register all robots and obstacles
        self.robot_rad = 0.1 
        for id in scenario_dict.list_robot_ID:
            self.robot_pos[id] = scenario_dict.init_pos[id]
            self.robot_theta[id] = 0.

            self.eval_env.register_obstacle_bounded(id, calc_robot_circ_bounds(self.robot_pos[id], self.robot_theta[id], self.robot_rad))


    def get_range_measurement(self, robot_ID):
        return self.eval_env.get_sensing_data(
                    self.robot_pos[robot_ID][0], self.robot_pos[robot_ID][1], self.robot_theta[robot_ID] + self.offset_lidar,
                    exclude=[robot_ID], beam_angles=self.beam_angles, max_distance=self.max_value, default_empty_val=0.)


    def update_robot_i(self, robot_ID, pos, theta):
        self.robot_pos[robot_ID] = pos
        self.robot_theta[robot_ID] = theta
        self.eval_env.register_obstacle_bounded(robot_ID, calc_robot_circ_bounds(pos, theta, self.robot_rad))    

