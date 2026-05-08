## Setup
USE "working" BRANCH!

Clone into workspace folder, rename repo folder to src

## To run:

cd TO YOUR WORKSPACE

colcon build --symlink-install

source install/setup.bash

ros2 launch armrs_package ROS2_sim_launch.py
