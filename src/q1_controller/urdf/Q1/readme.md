# Q1 URDF
----
## release v2.0 2025.11.6
- ##### 华勤Q1人形机器人全身不含灵巧手，共有29个关节自由度，单腿6个转动关节自由度，单臂7个转动关节自由度，头部2个转动关节自由度，腰部1个转动关节自由度。b若包含灵巧手，单手有11个关节自由度。本次更新主要增加了傲意灵巧手。
- ##### 目录中包含三个文件夹,` meshes/`用于存放`.STL`格式的CAD文件，`urdf/`用于存放`.urdf`格式的URDF文件，`mjcf/`用于存放`.xml`格式的mujoco通用机器人描述文件
```shell
.
├── meshes
├── mjcf
└── urdf
```
- ##### `.xml`格式的mujoco通用机器人描述文件是使用mujoco.viewer转换`.urdf`得到的。两者不存在任何实质上的区别。
- ##### 关节名称索引按照深度优先搜索原则的列表为：
```python
 [
    'L_hip_roll_joint', 
    'L_hip_yaw_joint', 
    'L_hip_pitch_joint', 
    'L_knee_joint', 
    'L_ankle_pitch_joint', 
    'L_ankle_roll_joint', 
    'R_hip_roll_joint', 
    'R_hip_yaw_joint', 
    'R_hip_pitch_joint', 
    'R_knee_joint', 
    'R_ankle_pitch_joint', 
    'R_ankle_roll_joint', 
    'pelvis_joint', 
    'L_shoulder_pitch_joint', 
    'L_shoulder_roll_joint', 
    'L_shoulder_yaw_joint', 
    'L_elbow_joint', 
    'L_forearm_yaw_joint', 
    'L_wrist_roll_joint', 
    'L_wrist_pitch_joint', 
    'l_if_proximal_joint', 
    'l_if_distal_joint', 
    'l_mf_proximal_joint', 
    'l_mf_distal_joint', 
    'l_rf_proximal_joint', 
    'l_rf_distal_joint', 
    'l_lf_proximal_joint', 
    'l_lf_distal_joint', 
    'l_th_root_joint', 
    'l_th_proximal_joint', 
    'l_th_distal_joint', 
    'R_shoulder_pitch_joint', 
    'R_shoulder_roll_joint', 
    'R_shoulder_yaw_joint', 
    'R_elbow_joint', 
    'R_forearm_yaw_joint', 
    'R_wrist_roll_joint', 
    'R_wrist_pitch_joint', 
    'r_if_proximal_joint', 
    'r_if_distal_joint', 
    'r_mf_proximal_joint', 
    'r_mf_distal_joint', 
    'r_rf_proximal_joint', 
    'r_rf_distal_joint', 
    'r_lf_proximal_joint', 
    'r_lf_distal_joint', 
    'r_th_root_joint', 
    'r_th_proximal_joint', 
    'r_th_distal_joint', 
    'head_yaw_joint', 
    'head_pitch_joint'
]
```
- ##### 刚体名称索引按照深度优先搜索原则的列表为：
```python
[
    'pelvis_link', 
    'L_hip_roll_link', 
    'L_hip_yaw_link', 
    'L_hip_pitch_link', 
    'L_knee_link', 
    'L_ankle_pitch_link', 
    'L_ankle_roll_link', 
    'R_hip_roll_link', 
    'R_hip_yaw_link', 
    'R_hip_pitch_link', 
    'R_knee_link', 
    'R_ankle_pitch_link', 
    'R_ankle_roll_link', 
    'torso_link', 
    'L_shoulder_pitch_link', 
    'L_shoulder_roll_link', 
    'L_shoulder_yaw_link', 
    'L_elbow_link', 
    'L_forearm_yaw_link', 
    'L_wrist_roll_link', 
    'l_hand_fix_link', 
    'l_if_proximal_link', 
    'l_if_distal_link', 
    'l_mf_proximal_link', 
    'l_mf_distal_link', 
    'l_rf_proximal_link', 
    'l_rf_distal_link', 
    'l_lf_proximal_link', 
    'l_lf_distal_link', 
    'l_th_root_link', 
    'l_th_proximal_link', 
    'l_th_distal_link', 
    'R_shoulder_pitch_link', 
    'R_shoulder_roll_link', 
    'R_shoulder_yaw_link', 
    'R_elbow_link', 
    'R_forearm_yaw_link', 
    'R_wrist_roll_link', 
    'r_hand_fix_link', 
    'r_if_proximal_link', 
    'r_if_distal_link', 
    'r_mf_proximal_link', 
    'r_mf_distal_link', 
    'r_rf_proximal_link', 
    'r_rf_distal_link', 
    'r_lf_proximal_link', 
    'r_lf_distal_link', 
    'r_th_root_link', 
    'r_th_proximal_link', 
    'r_th_distal_link', 
    'head_yaw_link', 
    'head_pitch_link'
]
```


## release v1.0 2025.11.1
- ##### 华勤Q1人形机器人全身不含灵巧手，共有29个关节自由度，单腿6个转动关节自由度，单臂7个转动关节自由度，头部2个转动关节自由度，腰部1个转动关节自由度。
- ##### 目录中包含三个文件夹,` meshes/`用于存放`.STL`格式的CAD文件，`urdf/`用于存放`.urdf`格式的URDF文件，`mjcf/`用于存放`.xml`格式的mujoco通用机器人描述文件
```shell
.
├── meshes
├── mjcf
└── urdf
```
- ##### `.xml`格式的mujoco通用机器人描述文件是使用mujoco.viewer转换`.urdf`得到的。两者不存在任何实质上的区别。
- ##### 关节名称索引按照深度优先搜索原则的列表为：
```python
 [
    'L_hip_roll_joint', 
    'L_hip_yaw_joint', 
    'L_hip_pitch_joint', 
    'L_knee_joint', 
    'L_ankle_pitch_joint', 
    'L_ankle_roll_joint', 
    'R_hip_roll_joint', 
    'R_hip_yaw_joint', 
    'R_hip_pitch_joint', 
    'R_knee_joint', 
    'R_ankle_pitch_joint', 
    'R_ankle_roll_joint', 
    'pelvis_joint', 
    'L_shoulder_pitch_joint', 
    'L_shoulder_roll_joint', 
    'L_shoulder_yaw_joint', 
    'L_elbow_joint', 
    'L_forearm_yaw_joint', 
    'L_wrist_roll_joint', 
    'L_wrist_yaw_joint', 
    'R_shoulder_pitch_joint', 
    'R_shoulder_roll_joint', 
    'R_shoulder_yaw_joint', 
    'R_elbow_joint', 
    'R_forearm_yaw_joint', 
    'R_wrist_roll_joint', 
    'R_wrist_yaw_joint', 
    'head_yaw_joint', 
    'head_pitch_joint'
]
```
- ##### 刚体名称索引按照深度优先搜索原则的列表为：
```python
[
    'pelvis_link', 
    'L_hip_roll_link', 
    'L_hip_yaw_link', 
    'L_hip_pitch_link', 
    'L_knee_link', 
    'L_ankle_pitch_link', 
    'L_ankle_roll_link', 
    'R_hip_roll_link', 
    'R_hip_yaw_link', 
    'R_hip_pitch_link', 
    'R_knee_link', 
    'R_ankle_pitch_link', 
    'R_ankle_roll_link', 
    'torso_link', 
    'L_shoulder_pitch_link', 
    'L_shoulder_roll_link', 
    'L_shoulder_yaw_link', 
    'L_elbow_link', 
    'L_forearm_yaw_link', 
    'L_wrist_roll_link', 
    'L_wrist_yaw_link', 
    'R_shoulder_pitch_link', 
    'R_shoulder_roll_link', 
    'R_shoulder_yaw_link', 
    'R_elbow_link', 
    'R_forearm_yaw_link', 
    'R_wrist_roll_link', 
    'R_wrist_yaw_link', 
    'head_yaw_link', 
    'head_pitch_link'
]
```
