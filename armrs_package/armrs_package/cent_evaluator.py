import numpy as np

class CentralizedEvaluator():
    def __init__(self, scenario_dict):

        self.list_ID = scenario_dict.list_robot_ID

        # Save list of robots within each formation
        self.form_ids = {}
        self.fleet_size = {}
        for f_id in scenario_dict.form_param:
            self.form_ids[f_id] = [int(i) for i in scenario_dict.form_param[f_id]['ids']]
            self.fleet_size[f_id] = len(self.form_ids[f_id])

        # Variable to save formation information
        self.form_cent = {}
        for f_id in self.form_ids:
            self.form_cent[f_id] = None


    def assess(self, robot_est_dict):
        # Calculate the true centroid position
        for f_id in self.form_ids:
            ids = self.form_ids[f_id]
            sum_of_pos, robot_num = np.zeros(3), 0            
            for i in ids:
                i_pos = robot_est_dict[i].lahead_pos
                if i_pos is not None:
                    sum_of_pos += i_pos
                    robot_num += 1

            if robot_num > 0:
                self.form_cent[f_id] = sum_of_pos / robot_num

