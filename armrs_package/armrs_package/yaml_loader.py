import yaml
import numpy as np

class ParamLoader():
    def __init__(self, param_file):
        # Load parameters from yaml file
        with open(param_file, 'r') as stream:
            self.param_dict = yaml.safe_load(stream)

        self.Ts = self.param_dict['Ts']
        self.Tmax = self.param_dict['t_max']
        self.ell = self.param_dict['look_ahead_ell']

        self.LIDAR_Ts = self.param_dict['LIDAR_Ts']

        field = self.param_dict['field']
        self.field_x = [field[0], field[1]]
        self.field_y = [field[2], field[3]]

        self.parse_obstacles()

    def parse_obstacles(self):
        self.obstacles={}
        if 'obstacles' in self.param_dict:
            for key in self.param_dict['obstacles']:
                self.obstacles[key] = np.array(self.param_dict['obstacles'][key], dtype=float)


class ScenarioLoader():

    def __init__(self, scenario_file):
        with open(scenario_file, 'r') as stream:
            self.scenario_dict = yaml.safe_load(stream)

        self.list_robot_ID = self.scenario_dict['robot_ID']
        self.robot_num = len(self.scenario_dict['robot_ID'])

        self.parse_formation()


    def parse_formation(self):
        self.form_param = {}
        self.init_pos, self.init_theta = {}, {}
        self.grouping = {} # reverse lookup table on which formation a robot belong

        # Store param for each formation group
        for form_id in self.scenario_dict['formation']:
            param = self.scenario_dict['formation'][form_id]
            # Store formation distance (A matrix) and tolerance (epsilon value)
            self.form_param[form_id] = {
                'ids': np.array(param['member'], dtype=int),
                'adj_mat': np.array(param['adjacency_mat'], dtype=int),
            }

            # Parse initial position
            init_pose = np.array(param['init_pos'], dtype=float)
            for i in range(len(param['member'])):
                robot_id = param['member'][i]

                self.init_pos[robot_id] = np.array([init_pose[i][0], init_pose[i][1], 0.])
                self.init_theta[robot_id] = init_pose[i][2]
                self.grouping[robot_id] = form_id

    def get_neigh_ids(self, robot_id):
        # grab robot's formation id and robot's index in list of ids
        form_id = self.grouping[robot_id]
        i = np.where(self.form_param[form_id]['ids'] == robot_id)[0][0]

        # Extract neighbours from Adjacency matrix
        i_neigh = []
        ids = self.form_param[form_id]['ids']
        A = self.form_param[form_id]['adj_mat']
        for j in range(len(ids)):
            if (i != j) and (A[i,j] > 0.):
                i_neigh += [ids[j]] # store into list
        
        return i_neigh

