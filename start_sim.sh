#!/bin/bash

# Функція автоматичного очищення
cleanup() {
    echo -e "\nЗупинка симулятора. Очищення фонових процесів Gazebo..."
    killall -9 gzserver gzclient 2>/dev/null
    exit 0
}

# Перехоплення натискання Ctrl+C (сигнал SIGINT)
trap cleanup SIGINT

# Експорт шляхів
source /usr/share/gazebo/setup.sh
export GAZEBO_MODEL_PATH=$HOME/ardupilot_gazebo/models:$GAZEBO_MODEL_PATH
export GAZEBO_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:$GAZEBO_PLUGIN_PATH

# Запуск Gazebo у фоні
gazebo --verbose ~/ardupilot_gazebo/worlds/iris_arducopter_runway.world &

sleep 5

# Запуск ArduPilot
cd ~/ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --console
