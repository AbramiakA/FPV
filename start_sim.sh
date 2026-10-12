#!/bin/bash

pkill -f gz 2>/dev/null
pkill -f mavproxy 2>/dev/null
pkill -f arducopter 2>/dev/null

source /opt/ros/humble/setup.bash
source "$HOME/ardu_ws/install/setup.bash"

export GZ_VERSION=harmonic

export GZ_SIM_RESOURCE_PATH="$HOME/ardu_ws/install/ardupilot_gazebo/share:$HOME/ardu_ws/install/ardupilot_sitl_models/share:${GZ_SIM_RESOURCE_PATH:-}"
export SDF_PATH="$HOME/ardu_ws/install/ardupilot_gazebo/share:$HOME/ardu_ws/install/ardupilot_sitl_models/share:${SDF_PATH:-}"
export ROS_PACKAGE_PATH="$HOME/ardu_ws/install/ardupilot_gazebo/share:$HOME/ardu_ws/install/ardupilot_sitl_models/share:${ROS_PACKAGE_PATH:-}"

ros2 launch ardupilot_gz_bringup iris_maze.launch.py
