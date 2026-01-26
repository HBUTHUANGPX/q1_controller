#!/bin/bash


# 导出PYTHONPATH环境变量
export PYTHONPATH=/home/niic/tmp/ros2_install/lib/python3.10/site-packages:$PYTHONPATH

# 加载ROS2安装环境
source /home/niic/tmp/ros2_install/setup.bash

# 加载自定义安装环境
source /home/niic/RL_control/Q1_control/install/setup.bash

ros2 run dexterous_hand hand_node