#!/bin/bash


# 导出PYTHONPATH环境变量
export PYTHONPATH=/home/niic/tmp/ros2_install/lib/python3.10/site-packages:$PYTHONPATH

# 加载ROS2安装环境
source /home/niic/tmp/ros2_install/setup.bash

# 加载自定义安装环境
source /home/niic/RL_control/Q1_control/install/setup.bash

# 执行ROS2命令，使用脚本参数替换
# ros2 topic echo /imu
# ros2 topic echo /gamepad_data
# ros2 topic echo /joint_states/position
# ros2 bag record /joint_states /observations /scaled_action -o 1208_1017_01_hpx_mimic
# rqt
# ros2 topic info  /multi_motor_state
# ros2 topic echo  /multi_motor_state
# ros2 topic echo  /gamepad_data
# ros2 bag record /multi_motor_state -o 0115_1717_hpx_mimic
# ros2 run dexterous_hand hand_node --ros-args -p hand_mode:=dual_hand
# ros2 topic hz /feedback/hand/left
# ros2 topic pub --once /control/hand/custom std_msgs/msg/Int32 "{data: 0}"
ros2 topic echo /control/hand/custom
