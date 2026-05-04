from launch import LaunchDescription, actions
from launch_ros.actions import Node
from datetime import datetime
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
IS_DISTRIBUTED_COMPUTATION = False



def generate_launch_description():
    ld = LaunchDescription()

    yaml_param = { 'param_yaml': param_file, 
                   'scenario_yaml': scenario_file } 

    # # ROS2 BAG recording
    # now = datetime.now() # current date and time
    # filename = 'experiment_' + now.strftime("%Y%m%d_%H%M%S")
    # ld.add_action(
    #     actions.ExecuteProcess(cmd=['ros2', 'bag', 'record', '-a',
    #         '-o', filename], 
    #     cwd='rosbag2', output='screen', log_cmd=True)
    # )

    # # Simulator Node
    # ld.add_action(
    #     Node(package=MRS_PKG, namespace=MRS_NAMESPACE,
    #          executable='ROS2_sim', name='sim',
    #          parameters=[ yaml_param ]
    #     )
    # )

    # ld.add_action(
    #     Node(package=MRS_PKG, namespace=MRS_NAMESPACE,
    #          executable='ROS2_sensors', name='sens',
    #          parameters=[ yaml_param ]
    #     )
    # )
    
    # Fleet Evaluator
    ld.add_action(
        Node(package=MRS_PKG, namespace=MRS_NAMESPACE,
             executable='ROS2_fleet_evaluator', name='fleet',
             parameters=[ yaml_param ]
        )
    )

    # Visualizer
    ld.add_action(
        Node(package=MRS_PKG, namespace=MRS_NAMESPACE,
             executable='ROS2_visualizer', name='viz',
             parameters=[ yaml_param ]
        )
    )

    if not IS_DISTRIBUTED_COMPUTATION:
        # Centralized Controller
        ld.add_action(
            Node(package=MRS_PKG, namespace=MRS_NAMESPACE,
                executable='ROS2_controller', name='controller',
                parameters=[ yaml_param ]
            )
        )

    else:
        # Distributed Controller
        for i in all_robots_ID:
            i_param = {'robot_ID': i}
            i_param.update(yaml_param)

            ld.add_action(
                Node(package=MRS_PKG,namespace=MRS_NAMESPACE,
                    executable='ROS2_dist_controller',
                    name='controller_' + str(i),
                    parameters=[ i_param ]
                )
            )

    return ld