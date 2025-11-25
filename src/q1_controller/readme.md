/usr/src/tensorrt/bin/trtexec --onnx=src/q1_controller/policy/2025-10-31_21-45-24_Q1_251021_03_walk_120Hz_67500.onnx --saveEngine=src/q1_controller/policy/2025-10-31_21-45-24_Q1_251021_03_walk_120Hz_67500.engine --fp16

ros2 launch hipnuc_imu imu_spec_msg.launch.py

sudo nmcli device wifi connect iPhone password hpx09201538

sudo chmod -R 777 /dev/ttyUSB*

colcon build --cmake-args -DUSE_BACKEND=TensorRT
colcon build --packages-select communication
colcon build --packages-select communication --cmake-args -DUSE_BACKEND=TensorRT

colcon build --cmake-args -DUSE_BACKEND=ONNX --packages-select q1_controller

echo "当前 ros2 命令路径 : $(which ros2 2>/dev/null || echo '未找到')"
pip install fastapi uvicorn jinja2



sudo ./install/ecmaster/lib/ecmaster/multi-master-leg --f1 NIIC_ENI_1113/NIIC_ENI_ECO_4.xml 
sudo ethercatctl stop 
sudo ethercatctl start 
sudo ethercatctl status
sudo ethercat slaves 




# 上肢校零
cd /home/niic/1119/setZero/ti5/examples/build
# 左臂校零
sudo ./bin/ethercat_ti5_pp -o 6 -m 0 -f ../../../../hq_code/NIIC_ENI_1113/NIIC_ENI_Ti5_7.xml
# 右臂校零
sudo ./bin/ethercat_ti5_pp -o 6 -m 2 -f ../../../../hq_code/NIIC_ENI_1113/NIIC_ENI_Ti5_8.xml







bash rl_motor.sh
sudo ethercatctl stop 
sudo ethercatctl start 

cd /home/niic/1119/hq_code && sudo ./install/ecmaster/lib/ecmaster/multi-master --f1 NIIC_ENI_1113/NIIC_ENI_ECO_4.xml --f0 NIIC_ENI_1113/NIIC_ENI_Ti5_7.xml --f2 NIIC_ENI_1113/NIIC_ENI_Ti5_8.xml 

sudo chmod -R 777 /dev/tty* && sudo chmod -R 777 /dev/input/js*
sudo -E bash sudoros2.sh run q1_controller gamepad_publisher.py
sudo -E bash sudoros2.sh launch hipnuc_imu imu_spec_msg.launch.py
sudo -E bash sudoros2.sh launch q1_controller robot_state_publisher.launch.py