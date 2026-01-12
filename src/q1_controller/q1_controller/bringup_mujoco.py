#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import mujoco
import mujoco.viewer

from tf2_ros import TransformBroadcaster  # 新增：TF 广播器
from std_msgs.msg import Bool, String, Float64
from sensor_msgs.msg import JointState, Imu
from control_msgs.msg import MultiDOFCommand, MultiDOFStateStamped, SingleDOFState
from geometry_msgs.msg import Vector3, Quaternion, TransformStamped
from rosgraph_msgs.msg import Clock

from builtin_interfaces.msg import Time
import time
import os
from scipy.spatial.transform import Rotation
import numpy as np
from q1_controller.msg import (
    MultiMotorCommand,
    SingleMotorCommand,
    Dataset,
)  # 新增：自定义消息导入

import yaml  # 导入PyYAML库


class MujocoSimNode(Node):
    def __init__(self):
        super().__init__("mujoco_sim_node")
        # 加载MuJoCo模型（替换为您的XML文件路径）
        model_path = os.path.expanduser(
            "src/q1_controller/urdf/Q1/mjcf/Q1_wo_hand_ankle_close_loop.xml"
        )
        self.spec = mujoco.MjSpec.from_file(model_path)
        self.model = self.spec.compile()
        self.data = mujoco.MjData(self.model)
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)

        self.dt = 0.002
        self.model.opt.timestep = self.dt
        # 创建发布者，话题名为'/mujoco_joint_states'
        # 创建Clock发布者，用于模拟时间
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.all_joint_name = [joint.name for joint in self.spec.joints][1:]
        self.URDF_joint_name = []
        self.URDF_joint_pos_index = []
        self.URDF_joint_vel_index = []
        URDF_joint_pos_index = 7
        URDF_joint_vel_index = 6

        self.control_joint_name = []
        self.control_joint_pos_index = []
        self.control_joint_vel_index = []
        control_joint_pos_index = 7
        control_joint_vel_index = 6
        self.flange_joint_name = []
        self.virtual_joint_name = []
        for i, joint in enumerate(self.spec.joints[1:]):
            if joint.group == 0 or joint.group == 1:
                self.URDF_joint_name.append(joint.name)
                self.URDF_joint_pos_index.append(URDF_joint_pos_index)
                self.URDF_joint_vel_index.append(URDF_joint_vel_index)
                URDF_joint_pos_index += 1
                URDF_joint_vel_index += 1
            elif joint.group == 2:
                URDF_joint_pos_index += 1
                URDF_joint_vel_index += 1
            elif joint.group == 3:
                URDF_joint_pos_index += 4
                URDF_joint_vel_index += 3

            if joint.group == 0 or joint.group == 2:
                self.control_joint_name.append(joint.name)
                self.control_joint_pos_index.append(control_joint_pos_index)
                self.control_joint_vel_index.append(control_joint_vel_index)
                control_joint_pos_index += 1
                control_joint_vel_index += 1
            elif joint.group == 1:
                control_joint_pos_index += 1
                control_joint_vel_index += 1
            elif joint.group == 3:
                control_joint_pos_index += 4
                control_joint_vel_index += 3

            if joint.group == 1:
                self.virtual_joint_name.append(joint.name)
            if joint.group == 2:
                self.flange_joint_name.append(joint.name)
        print("URDF_joint_name :\r\n", self.URDF_joint_name)
        print("URDF_joint_pos_index :\r\n", self.URDF_joint_pos_index)
        print("URDF_joint_vel_index :\r\n", self.URDF_joint_vel_index)
        print("control_joint_name :\r\n", self.control_joint_name)
        print("virtual_joint_name :\r\n", self.virtual_joint_name)
        print("flange_joint_name :\r\n", self.flange_joint_name)
        self.target_cmd_sub = self.create_subscription(
            MultiMotorCommand, "/target_pos", self.target_cmd_callback, 10
        )
        self.mocap_dataset_sub = self.create_subscription(
            Dataset, "/Dataset", self.mocap_dataset_callback, 10
        )

        self.robot_reset_zero = self.create_subscription(
            Bool, "/reset_zero", self._reset_zero_callback, 10
        )

        self.motor_index_map = {
            joint.name: i for i, joint in enumerate(self.spec.joints[1:])
        }
        self.latest_commands = {}
        for name in self.control_joint_name:
            self.latest_commands[name] = {
                "target_pos": 0.0,
                "target_vel": 0.0,
                "p_gain": 0.0,
                "d_gain": 0.0,
                "ff_effort": 0.0,
            }
        ##################33
        # 定时器：每0.01秒执行一次仿真步进（仿真频率100Hz）
        self.control_decimation = 10
        timer_period = 10 * self.dt  # 秒
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self._init_joint_state_publish()
        self._init_imu_publish()
        self.get_logger().info("MuJoCo simulation node initialized.")
        self.last_wall_time = time.time()
        # 新增：TF 广播器，用于广播 world 到 pelvis_link 的变换
        self.tf_broadcaster = TransformBroadcaster(self)
        self.reset_zero_flag = False
        self.yaml_data = self.read_yaml_file(
            "src/q1_controller/config/h1.yaml"
        )
        INDEX = self.yaml_data["isaac_sim2mujoco_index"]
        self.isaac_sim2mujoco_index = self.yaml_data["isaac_sim2mujoco_index"]
        self.mujoco2isaac_sim_index = self.yaml_data["mujoco2isaac_sim_index"]
        print(self.isaac_sim2mujoco_index)

    def read_yaml_file(self, file_path):
        try:
            with open(file_path, "r", encoding="utf-8") as file:
                data = yaml.safe_load(file)
            return data
        except FileNotFoundError:
            print(f"文件 {file_path} 不存在。")
            return None
        except yaml.YAMLError as exc:
            print(f"YAML解析错误: {exc}")
            return None

    def _reset_zero_callback(self, msg: Bool):
        self.data.qpos[0:3] = np.array([0, 0, 1], dtype=np.float32)
        self.data.qpos[3:7] = np.array([1, 0, 0, 0], dtype=np.float32)
        self.data.qpos[7:] = 0.0
        self.data.qvel[:] = 0.0
        mujoco.mj_step(self.model, self.data)
        print("=======reset_zero======")

    def _init_imu_publish(self):
        self.imu_publisher = self.create_publisher(Imu, "/imu", 10)
        self.imu = Imu()
        self.imu.header.frame_id = "imu_link"

    def _init_joint_state_publish(self):
        self.mujoco_joint_state_publisher = self.create_publisher(
            JointState, "/mujoco_joint_states", 10
        )
        self.joint_state_publisher = self.create_publisher(
            JointState, "/joint_states", 10
        )
        self.joint_state = JointState()
        self.joint_state.name = self.URDF_joint_name

    def target_cmd_callback(self, msg: MultiMotorCommand):
        """回调函数：接收 MultiMotorCommand 并存储最新命令。"""
        # self.latest_commands.clear()
        for cmd in msg.commands:
            self.latest_commands[cmd.motor_name] = {
                "target_pos": cmd.target_pos,
                "target_vel": cmd.target_vel,
                "p_gain": cmd.p_gain,
                "d_gain": cmd.d_gain,
                "ff_effort": cmd.ff_effort,
            }
            ...
        # self.get_logger().info("Received new motor commands.")

    def mocap_dataset_callback(self, msg: Dataset):
        motor_state: JointState = msg.motor_state
        root_angular_velocity: Vector3 = msg.root_angular_velocity
        root_velocity: Vector3 = msg.root_velocity
        root_position: Vector3 = msg.root_position
        root_quat: Quaternion = msg.root_quat

        self.data.qpos[0:3] = np.array(
            [root_position.x, root_position.y, root_position.z], dtype=np.float32
        )
        self.data.qpos[3:7] = np.array(
            [root_quat.w, root_quat.x, root_quat.y, root_quat.z], dtype=np.float32
        )
        self.data.qpos[self.URDF_joint_pos_index][
            self.mujoco2isaac_sim_index
        ] = motor_state.position
        self.data.qvel[0:3] = np.array(
            [root_velocity.x, root_velocity.y, root_velocity.z], dtype=np.float32
        )
        self.data.qvel[3:6] = np.array(
            [root_angular_velocity.x, root_angular_velocity.y, root_angular_velocity.z],
            dtype=np.float32,
        )
        self.data.qvel[self.URDF_joint_vel_index][
            self.mujoco2isaac_sim_index
        ] = motor_state.velocity
        mujoco.mj_step(self.model, self.data)
        self.get_logger().info("Received mocap dataset.")

    def _imu_publish(self):
        self.joint_state.header.stamp = (
            self.time_stamp
        )  # 使用节点时钟（会同步到sim time如果设置）
        torso_link_xquat = self.data.xquat[
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "torso_link"), :
        ]
        ang_vel_data = self.data.sensor("imu-angular-velocity").data.tolist()
        angular_velocity = Vector3(
            x=ang_vel_data[0], y=ang_vel_data[1], z=ang_vel_data[2]
        )
        self.imu.angular_velocity = angular_velocity
        lin_acc_data = self.data.sensor("imu-linear-acceleration").data.tolist()
        linear_acceleration = Vector3(
            x=lin_acc_data[0], y=lin_acc_data[1], z=lin_acc_data[2]
        )
        self.imu.linear_acceleration = linear_acceleration
        orientation = Quaternion(
            w=torso_link_xquat[0],
            x=torso_link_xquat[1],
            y=torso_link_xquat[2],
            z=torso_link_xquat[3],
        )
        self.imu.orientation = orientation
        self.imu_publisher.publish(self.imu)

    def _joint_state_publish(self):
        self.joint_state.header.stamp = (
            self.time_stamp
        )  # 使用节点时钟（会同步到sim time如果设置）
        self.joint_state.position = self.data.qpos[self.URDF_joint_pos_index].tolist()
        self.joint_state.velocity = self.data.qvel[self.URDF_joint_vel_index].tolist()
        self.joint_state.effort = (
            self.data.actuator_force
            # self.data.qvel[self.URDF_joint_vel_index] * 0
        ).tolist()
        self.joint_state_publisher.publish(self.joint_state)
        self.mujoco_joint_state_publisher.publish(self.joint_state)
        # print(self.data.actuator_force)
        # print(self.data.qfrc_actuator)

    def _publish_world_to_pelvis_tf(self):
        """广播 world 到 pelvis_link 的 TF 变换。"""
        # 获取 pelvis_link 的世界位置和姿态（假设 pelvis_link 是 Mujoco 中的 "pelvis_link" body）
        pelvis_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "pelvis_link"
        )
        pelvis_pos = self.data.xpos[pelvis_id]  # [x, y, z]
        pelvis_quat = self.data.xquat[
            pelvis_id
        ]  # [w, x, y, z] (Mujoco 的 xquat 是 w-first)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.time_stamp
        tf_msg.header.frame_id = "world"  # 父 frame
        tf_msg.child_frame_id = "pelvis_link"  # 子 frame
        tf_msg.transform.translation.x = pelvis_pos[0]
        tf_msg.transform.translation.y = pelvis_pos[1]
        tf_msg.transform.translation.z = pelvis_pos[2]
        tf_msg.transform.rotation.w = pelvis_quat[0]  # w
        tf_msg.transform.rotation.x = pelvis_quat[1]  # x
        tf_msg.transform.rotation.y = pelvis_quat[2]  # y
        tf_msg.transform.rotation.z = pelvis_quat[3]  # z

        self.tf_broadcaster.sendTransform(tf_msg)

    def timer_callback(self):
        # 获取当前墙钟时间
        current_wall_time = time.time()
        delta_wall = current_wall_time - self.last_wall_time

        # 计算需要执行的步数（补偿延迟）
        accumulated_time = 0.0  # 用于调整last_wall_time
        for _ in range(self.control_decimation):
            idx = 0
            # print("=======================")
            for motor_name, cmd in self.latest_commands.items():
                idx = self.control_joint_name.index(motor_name)
                current_pos = self.data.qpos[self.control_joint_pos_index[idx]]
                current_vel = self.data.qvel[self.control_joint_vel_index[idx]]
                # print(motor_name)
                
                torque = (
                    cmd["p_gain"] * (cmd["target_pos"] - current_pos)
                    + cmd["d_gain"] * (cmd["target_vel"] - current_vel)
                    + cmd["ff_effort"]
                )
                # if motor_name == "L_Flange_A_joint":
                #     print("L_Flange_A_joint current_pos:{:7.4f}".format(current_pos))
                # if motor_name == "L_Flange_B_joint":
                #     print("L_Flange_B_joint current_pos:{:7.4f}".format(current_pos))
                # if motor_name == "R_Flange_A_joint":
                #     print("R_Flange_A_joint current_pos:{:7.4f}".format(current_pos))
                #     print(
                #         "p_gain :{:7.4f},target_pos :{:7.4f},d_gain :{:7.4f},target_vel :{:7.4f},torque :{:7.4f}".format(
                #             cmd["p_gain"],
                #             cmd["target_pos"],
                #             cmd["d_gain"],
                #             cmd["target_vel"],
                #             torque,
                #         )
                #     )
                # if motor_name == "R_Flange_B_joint":
                #     print("R_Flange_B_joint current_pos:{:7.4f}".format(current_pos))
                # print("target_pos:\r\n",cmd["target_pos"])
                self.data.ctrl[idx] = torque
                # idx+=1
            # 执行MuJoCo步进
            # print(self.data.ctrl)
            # self.data.qpos[0:3] = np.array([0, 0, 1.2])
            # self.data.qpos[3:7] = np.array([1, 0, 0, 0])
            # self.data.qvel[0:3] = np.array([0, 0, 0])
            # self.data.qvel[3:6] = np.array([0, 0, 0])
            mujoco.mj_step(self.model, self.data)
            # 准备并发布JointState（在多步后发布一次，或每步发布根据需求）
            self.time_stamp = self.get_clock().now().to_msg()
            self._joint_state_publish()
            self._imu_publish()
            self._publish_world_to_pelvis_tf()
            # 发布模拟时间（基于data.time）
            clock_msg = Clock()
            sim_time_sec = int(self.data.time)
            sim_time_nsec = int((self.data.time - sim_time_sec) * 1e9)
            clock_msg.clock = Time(sec=sim_time_sec, nanosec=sim_time_nsec)
            self.clock_pub.publish(clock_msg)

            accumulated_time += self.dt
        self.viewer.sync()
        # 更新last_wall_time（减去未完整步的剩余时间）
        self.last_wall_time = current_wall_time - (delta_wall - accumulated_time)


def main(args=None):
    rclpy.init(args=args)
    node = MujocoSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
