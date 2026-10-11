#!/bin/bash

cleanup() {
    killall -9 gzserver gzclient 2>/dev/null
}

trap cleanup SIGINT

source /usr/share/gazebo/setup.sh
source /opt/ros/humble/setup.bash
source "$HOME/ardu_ws/install/setup.bash"

export GAZEBO_MODEL_PATH=$HOME/ardupilot_gazebo/models:$GAZEBO_MODEL_PATH
export GAZEBO_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:$GAZEBO_PLUGIN_PATH

gazebo --verbose ~/ardupilot_gazebo/worlds/iris_arducopter_runway.world &

sleep 5
cd "$HOME/fpv_labs"

ros2 launch ardupilot_sitl sitl_dds_udp.launch.py \
    transport:=udp4 \
    synthetic_clock:=True \
    wipe:=True \
    model:=gazebo-iris \
    speedup:=1 \
    slave:=0 \
    instance:=0 \
    defaults:=$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/copter.parm,$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/gazebo-iris.parm,$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/dds_udp.parm \
    sim_address:=127.0.0.1 \
    master:=tcp:127.0.0.1:5760 \
    sitl:=127.0.0.1:5501

cleanup
