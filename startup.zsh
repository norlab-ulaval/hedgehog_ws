#!/bin/zsh

source /opt/ros/humble/setup.zsh
source /home/hedgehog/hedgehog_ws/install/setup.zsh

screen -r -S "startup" -X quit 2>/dev/null

screen -S startup -dm bash -c "ros2 launch hedgehog_system system.launch.py"
