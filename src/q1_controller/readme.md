# 使用trt库进行onnx转engine的指令
/usr/src/tensorrt/bin/trtexec --onnx=src/q1_controller/policy/2025-10-31_21-45-24_Q1_251021_03_walk_120Hz_67500.onnx --saveEngine=src/q1_controller/policy/2025-10-31_21-45-24_Q1_251021_03_walk_120Hz_67500.engine --fp16

# 额外库安装的指令
sudo apt-get install ros-humble-gps-msgs
pip install pygame -i https://pypi.tuna.tsinghua.edu.cn/simple

# 连接wifi的指令
sudo nmcli device wifi connect iPhone password hpx09201538


# 国讯上进行编译的指令
note： 先编译communication然后再编译所有
colcon build --packages-select communication
colcon build --cmake-args -DUSE_BACKEND=TensorRT
# 在主机上进行编译的指令
colcon build --cmake-args -DUSE_BACKEND=ONNX --packages-select q1_controller

# 查看ros2环境的指令
echo "当前 ros2 命令路径 : $(which ros2 2>/dev/null || echo '未找到')"



sudo ./install/ecmaster/lib/ecmaster/multi-master-leg --f1 NIIC_ENI_1113/NIIC_ENI_ECO_4.xml 


# ethercat相关的指令
sudo ethercatctl stop 
sudo ethercatctl start 
sudo ethercatctl status
sudo ethercat slaves 


# 钛虎的标之前运行multi-master，换边标也要运行一次。不需要断电再开
# shoulder pitch、roll使用u型槽，shoulder yaw下槽口旋转90度至朝身体外侧，elbow使用u型槽，
# forearm_yaw下槽口旋转90度至朝身体前侧。
# 上肢校零
cd /home/niic/1127/setZero/ti5/examples/build
# 左臂校零
sudo ./bin/ethercat_ti5_pp -o 6 -m 0 -f ../../../../hq_code/NIIC_ENI_1113/NIIC_ENI_Ti5_7.xml
# 右臂校零
sudo ./bin/ethercat_ti5_pp -o 6 -m 2 -f ../../../../hq_code/NIIC_ENI_1113/NIIC_ENI_Ti5_7.xml


# 实机相关指令
bash rl_motor.sh
sudo ethercatctl stop 
sudo ethercatctl start 

cd /home/niic/1127/hq_code && sudo ./install/ecmaster/lib/ecmaster/multi-master --f1 NIIC_ENI_1113/NIIC_ENI_ECO_4.xml --f0 NIIC_ENI_1113/NIIC_ENI_Ti5_7.xml --f2 NIIC_ENI_1113/NIIC_ENI_Ti5_7.xml 

sudo chmod -R 777 /dev/tty* && sudo chmod -R 777 /dev/input/js*
sudo -E bash sudoros2.sh run q1_controller gamepad_publisher.py
sudo -E bash sudoros2.sh launch hipnuc_imu imu_spec_msg.launch.py
sudo -E bash sudoros2.sh launch q1_controller robot_state_publisher.launch.py



sudo -E bash sudoros2.sh bag record /joint_states

sudo -E export PYTHONPATH=/home/niic/tmp/ros2_install/lib/python3.10/site-packages:$PYTHONPATH && source /home/niic/tmp/ros2_install/setup.bash && source /home/niic/RL_control/Q1_control/install/setup.bash && [bag record /joint_states /observations /scaled_action -o 1202_1543](../../sudoros2.sh)