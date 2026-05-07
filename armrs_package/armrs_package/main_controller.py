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

        # TODO
        # - Store the needed setup from scenario_dict
        self.current_goal = np.zeros(3) # TODO: e.g., parse from yaml



    def compute_control_input(self, estimation_dict: Estimation):
        current_pos = estimation_dict.lahead_pos

        ## ------------------------------------
        # CONTROLLER CALCULATION
        ## ------------------------------------
        # Compute nominal control # TODO
        dir_x = estimation_dict.target_point_x - estimation_dict.pos[0]
        dir_y = estimation_dict.target_point_y - estimation_dict.pos[1]

        if dir_x == 0.0 and dir_y == 0.0:
            return

        magnitude = math.sqrt(dir_x**2 + dir_y**2)
        unit_x = 0.0
        unit_y = 0.0
        if magnitude != 0.0:
            unit_x = dir_x / (magnitude * 4.0)
            unit_y = dir_y / (magnitude * 4.0)
        
        vx = unit_x
        vy = unit_y

        u_nom = np.array([vx, vy, 0.0])

        # Implement safety controller # TODO
        vel_command = u_nom

        # return the resulting control input
        estimation_dict.goal = self.current_goal
        estimation_dict.vel_command = vel_command
        return vel_command



    @staticmethod
    def si_to_unicycle(u, theta, ell):
        vel_lin = u[0]*np.cos(theta) + u[1]*np.sin(theta)
        vel_ang = (- u[0]*np.sin(theta) + u[1]*np.cos(theta))/ell
        return vel_lin, vel_ang
