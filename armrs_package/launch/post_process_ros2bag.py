from launch import LaunchDescription, actions
from launch_ros.actions import Node
import yaml

# PACKAGE AND NODE INFO
MRS_PKG = 'armrs_package'
MRS_NAMESPACE = 'mrs'

# YAML TO PARSE
src_path = "/armrs_ros2_humble/ros2_ws/src/" # TODO: Fix this to absolute src path in your setup
yaml_path = src_path + MRS_PKG + "/" + MRS_PKG + "/" 

param_file = yaml_path + "sim_setup.yaml" # TODO: Simulation parameter & environment setting
scenario_file = yaml_path + "scenario_demo_1formation.yaml" # TODO: robot and controller parameter
# scenario_file = yaml_path + "scenario_demo_2formation.yaml" # TODO: robot and controller parameter

# EXTRACT LIST OF ROBOT
with open(scenario_file, 'r') as stream:
    scenario_dict = yaml.safe_load(stream)

all_robots_ID = scenario_dict['robot_ID']

# LAUNCH SETTINGS
IS_DISTRIBUTED_COMPUTATION = True



def generate_launch_description():
    ld = LaunchDescription()

    yaml_param = { 'param_yaml': param_file, 
                   'scenario_yaml': scenario_file } 

    # ROS2 BAG PLAY
    tag = '20260402'
    fname = 'simulation_' + tag
    # fname = 'experiment_' + tag
    rosbag_path = fname + '/' + fname + '_0.db3' 

    ld.add_action(
        actions.ExecuteProcess(cmd=['ros2', 'bag', 'play', rosbag_path], 
        cwd='rosbag2', output='screen', log_cmd=True)
    )

    # Visualizer
    ld.add_action(
        Node(package=MRS_PKG, namespace=MRS_NAMESPACE,
             executable='ROS2_visualizer', name='viz',
             parameters=[ yaml_param ]
        )
    )

    return ld