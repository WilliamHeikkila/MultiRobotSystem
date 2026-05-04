import numpy as np

# Message encapsulation and deconstruct

# --------------------------------------------------------------
# StateExchange Message <---> Estimation Object
# --------------------------------------------------------------
# Robot's State
def est2msg_robot_state(msg, est):    
    msg.pose.x = est.pos[0]
    msg.pose.y = est.pos[1]
    msg.pose.theta = est.theta
    
def get_robot_state(msg):
    return np.array([msg.pose.x, msg.pose.y, 0]), msg.pose.theta


# Robot's navigation
def est2msg_navigation(msg, est):
    msg.goal.x = est.goal[0]
    msg.goal.y = est.goal[1]

    msg.si_vel = est.vel_command.tolist()

def get_navigation_data(msg):
    return np.array([msg.goal.x, msg.goal.y, 0]), np.array(msg.si_vel)


# --------------------------------------------------------------
# PoseStamped & LaserScan <---> Estimation Object
# --------------------------------------------------------------
def get_pos_yaw(msg):
    # processing poseStamped from VRPN data
    pos = msg.pose.position
    quat = msg.pose.orientation
    # yaw calculation (quat to rpy)
    # Source: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
    cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return pos, yaw

def get_scan_data(msg):
    # Parse LaserScan data
    a_min, a_max = msg.angle_min, msg.angle_max
    scan_data = np.array(msg.ranges)
    data_size = scan_data.shape[0]
    beam_angles, step = np.linspace(a_min, a_max, num=data_size, endpoint=True, retstep = True)
    # # SANITY CHECK
    # self.get_logger().info(f'robot {index}: sensor data step {msg.angle_increment} vs beam step {step}')

    return scan_data, beam_angles


# --------------------------------------------------------------
# FleetInformation Message <---> CentralizedEvaluator Object
# --------------------------------------------------------------
def cent_evaluator_to_msg(f_id, msg, cent_evaluator):
    form_ids = cent_evaluator.form_ids[f_id]
    msg.robot_ids = form_ids
    msg.fleet_size = cent_evaluator.fleet_size[f_id]

    # The true centroid position
    centroid_form = cent_evaluator.form_cent[f_id]
    if centroid_form is not None:
        msg.centroid_formation.x = centroid_form[0]
        msg.centroid_formation.y = centroid_form[1]


def msg_to_cent_evaluator_data(f_id, msg, cent_evaluator):
    robot_ids = msg.robot_ids
    fleet_size = msg.fleet_size

    # The true centroid position
    cent_evaluator.form_cent[f_id] = np.array([msg.centroid_formation.x, msg.centroid_formation.y, 0])
