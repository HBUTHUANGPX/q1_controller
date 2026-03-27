#!/usr/bin/env python3
import sys

# 获取当前Python解释器的路径
python_path = sys.executable
print(f"当前Python环境的路径：{python_path}")

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Twist, Accel
from xsens_mvn_ros2_msgs.msg import LinkState, LinkStateArray  # 假设消息包已安装
import numpy as np
import general_motion_retargeting
from general_motion_retargeting import GeneralMotionRetargeting as GMR
from general_motion_retargeting import ROBOT_XML_DICT
import mujoco
import mujoco.viewer
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


class LinkStatesSubscriber(Node):
    """
    一个ROS 2节点类，用于订阅/link_states话题，并解析LinkStateArray消息。
    该类采用OOP设计，封装了订阅逻辑和消息解析功能。
    """

    def __init__(self, robot, actual_human_height):
        super().__init__("link_states_subscriber")
        self.robot = robot
        self.actual_human_height = actual_human_height
        # 创建订阅者，话题类型为LinkStateArray
        self.subscription = self.create_subscription(
            LinkStateArray, "/link_states", self.listener_callback, 10  # QoS深度
        )
        self.get_logger().info("Subscribed to /link_states topic.")
        self.joint_state_pub = self.create_publisher(JointState, "/xsens_gmr/joint_states", 10)
        self.retargeter = GMR(
            src_human="xsens_bvh_online",
            tgt_robot=robot,
            actual_human_height=actual_human_height,
        )
        self.init_mujoco()
        self.first_time = True
        self.first_pos = None
        self.first_quat = None
        self.first_frame = None

    def init_mujoco(self):
        self.xml_path = ROBOT_XML_DICT[self.robot]
        self.spec = mujoco.MjSpec.from_file(str(self.xml_path))
        self.model = mujoco.MjModel.from_xml_path(str(self.xml_path))
        self.data = mujoco.MjData(self.model)
        self.mujoco_joint_name = [joint.name for joint in self.spec.joints][1:]
        self.viewer = mujoco.viewer.launch_passive(
             self.model,
             self.data,
             # show_left_ui=False,
             # show_right_ui=False
        )
        return

    def _draw_geom(
        self,
        position,
        rotation_matrix=None,
        axis_length=0.1,
        shaft_width=0.008,
        position_offset=np.array([0, 0, 0]),
        joint_name=None,
        orientation_correction=Rotation.from_euler("xyz", [0, 0, 0]),
    ):
        # 添加X轴箭头（红色）\
        from_pos = np.array(position + position_offset, dtype=np.float64).reshape(-1, 1)
        if rotation_matrix is not None:
            rotation_matrix = (
                np.array(rotation_matrix, dtype=np.float64).reshape(9).reshape(-1, 1)
            )
        else:
            rotation_matrix = (
                np.eye(3).flatten().astype(np.float64).reshape(-1, 1)
            )  # 默认单位矩阵
        rgba_list = [
            np.array([1.0, 0.0, 0.0, 1.0], dtype=np.float32).reshape(-1, 1),  # 红色
            np.array([0.0, 1.0, 0.0, 1.0], dtype=np.float32).reshape(-1, 1),  # 绿色
            np.array([0.0, 0.0, 1.0, 1.0], dtype=np.float32).reshape(-1, 1),  # 蓝色
        ]
        for axis_idx in range(3):
            geom = self.viewer.user_scn.geoms[self.viewer.user_scn.ngeom]
            if joint_name is not None:
                geom.label = joint_name
            # print("from_pos:\r\n\t",from_pos.shape,"\r\n\t",from_pos)
            # print("rotation_matrix:\r\n\t",rotation_matrix.shape,"\r\n\t",rotation_matrix)
            # print("rgba_list[axis_idx]:\r\n\t",rgba_list[axis_idx].shape,"\r\n\t",rgba_list[axis_idx])
            mujoco.mjv_initGeom(
                geom,
                type=mujoco.mjtGeom.mjGEOM_ARROW,
                size=np.array([0.0, 0.0, 0.0], dtype=np.float64).reshape(-1, 1),
                pos=from_pos,
                mat=rotation_matrix,
                rgba=rgba_list[axis_idx],
            )
            fix = orientation_correction.as_matrix().astype(np.float64)
            to_pos = from_pos + axis_length * (rotation_matrix.reshape(3, 3) @ fix)[
                :, axis_idx
            ].reshape(-1, 1)
            # print("to_pos:\r\n\t",to_pos.shape,"\r\n\t",to_pos)
            mujoco.mjv_connector(
                geom,
                type=mujoco.mjtGeom.mjGEOM_ARROW,
                width=shaft_width,
                from_=from_pos,
                to=to_pos,
            )
            self.viewer.user_scn.ngeom += 1

    def listener_callback(self, msg: LinkStateArray):
        """
        回调函数：接收并解析LinkStateArray消息。
        模拟读取和解析过程，包括头信息和每个LinkState的细节。
        """
        # 解析头信息
        header: Header = msg.header
        self.get_logger().info(f"Received message")
        # print(self.mujoco_joint_name)
        self.frames = []
        # 遍历并解析每个LinkState
        self.viewer.user_scn.ngeom = 0
        result = {}
        for i, state in enumerate(msg.states):
            pose: Pose = state.pose
            twist: Twist = state.twist
            accel: Accel = state.accel
            header_link: Header = state.header
            # 模拟解析并打印（可替换为实际处理逻辑）

            pos = np.array(
                [
                    pose.position.x,
                    pose.position.y,
                    pose.position.z,
                ]
            )

            quat = np.array(
                [
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ]
            )
            quat = Rotation.from_quat(quat.copy(), scalar_first=False).as_quat(
                scalar_first=True
            )
            if not self.first_time:
                (first_pos, first_quat) = self.first_frame[0][header_link.frame_id]
                # result[header_link.frame_id] = (pos.copy()-first_pos,quat.copy())
                result[header_link.frame_id] = (pos.copy(), quat.copy())
            else:
                result[header_link.frame_id] = (pos.copy(), quat.copy())
            # self.get_logger().info(f'Parsed LinkState {header_link.frame_id}')
            # if header_link.frame_id == "pelvis":
            #     self.data.qpos[0:3] = pos.copy()
            #     self.data.qpos[3:7] = quat.copy()
            # self.get_logger().info(f'Parsed LinkState {header_link.frame_id}:')
            # self.get_logger().info(f'  Pose: position=({pose.position.x}, {pose.position.y}, {pose.position.z}), '
            #                     f'orientation=({pose.orientation.x}, {pose.orientation.y}, {pose.orientation.z}, {pose.orientation.w})')
            # self.get_logger().info(f'  Twist: linear=({twist.linear.x}, {twist.linear.y}, {twist.linear.z}), '
            #                        f'angular=({twist.angular.x}, {twist.angular.y}, {twist.angular.z})')
            # self.get_logger().info(f'  Accel: linear=({accel.linear.x}, {accel.linear.y}, {accel.linear.z}), '
            #                        f'angular=({accel.angular.x}, {accel.angular.y}, {accel.angular.z})')
            # if header_link.frame_id == "right_shoulder" or header_link.frame_id == "left_shoulder":
            
            self._draw_geom(
                pos.copy(),
                rotation_matrix=Rotation.from_quat(quat.copy(), scalar_first=True)
                .as_matrix()
                .flatten(),
                joint_name=header_link.frame_id,
            )

        result["LeftFootMod"] = (
            np.array(
                [
                    result["left_foot"][0][0],
                    result["left_foot"][0][1],
                    result["left_foot"][0][2],
                    # result["LeftToe"][0][2],
                ]
            ),
            result["left_foot"][1],
            # result["LeftToe_end_site"][1],
        )
        result["RightFootMod"] = (
            np.array(
                [
                    result["right_foot"][0][0],
                    result["right_foot"][0][1],
                    result["right_foot"][0][2],
                    # result["RightToe"][0][2],
                ]
            ),
            result["right_foot"][1],
            # result["RightToe_end_site"][1],
        )

        self.frames.append(result)
        # self.data.qpos[0:3] = np.array([0,0,1.3])
        # self.data.qpos[3:7] = np.array([0,0,0,1])
        self.data.qvel[:] = 0
        # print(self.frames)
        self.make_retargeting()
        self.viewer.sync()

        if self.first_time:
            self.first_time = False
            self.first_frame = self.frames.copy()

    def make_retargeting(self):
        # self.get_logger().info("start retargeting")
        qpos = self.retargeter.retarget(self.frames[0])
        root_pos = qpos[:3]
        root_quat = qpos[3:7]
        dof_pos = qpos[7:]
        self.data.qpos[0:3] = root_pos.copy()
        self.data.qpos[3:7] = root_quat.copy()
        self.data.qpos[7:] = dof_pos.copy()
        mujoco.mj_step(self.model, self.data)
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.mujoco_joint_name
        msg.position = dof_pos.tolist()
        self.joint_state_pub.publish(msg)


        # self.get_logger().info("end")


def main(args=None):
    rclpy.init(args=args)
    node = LinkStatesSubscriber(robot="Q1", actual_human_height=1.66)

    # 主循环：模拟持续读取和解析，使用spin_once手动处理
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)  # 每0.1秒检查一次消息
            # 这里可添加其他模拟逻辑，例如延时或条件退出
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
