#!/bin/bash

# 检查是否提供了足够的参数
if [ $# -lt 3 ]; then
    echo "用法: sudo -E ./ros2_runner.sh <param1> <param2> <param3>"
    echo "例如: sudo -E ./ros2_runner.sh aaa bbb ccc"
    exit 1
fi

# 导出PYTHONPATH环境变量
export PYTHONPATH=/home/niic/tmp/ros2_install/lib/python3.10/site-packages:$PYTHONPATH

# 加载ROS2安装环境
source /home/niic/tmp/ros2_install/setup.bash

# 加载自定义安装环境
source /home/niic/RL_control/Q1_control/install/setup.bash

# 执行ROS2命令，使用脚本参数替换
ros2 $1 $2 $3