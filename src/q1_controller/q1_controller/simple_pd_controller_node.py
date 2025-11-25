#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class SimplePDControllerNode(Node):
    def __init__(self):
        super().__init__('simple_pd_controller_node')
        # 定义默认关节目标位置（单位：弧度）
        self.default_dof_pos = [0.0] * 29
        # PD 控制参数
        self.Kp = 10.0  # 比例增益
        self.Kd = 1.0   # 微分增益
        # 当前关节位置和速度（初始化为零）
        self.dof_pos = [0.0] * 29
        self.dof_vel = [0.0] * 29
        # 关节力矩发布器
        self.joint_effort_pub = self.create_publisher(
            Float64MultiArray, '/joint_effort_controller/commands', 10
        )
        # 关节状态订阅器
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        # 定时器：每 0.005 秒执行一次控制循环
        self.timer = self.create_timer(0.005, self.control_loop)
        self.get_logger().info("Simple PD Controller Node initialized.")

    def joint_state_callback(self, msg: JointState):
        if len(msg.position) >= 29 and len(msg.velocity) >= 29:
            # 更新关节位置和速度（注意关节顺序映射）
            self.dof_pos = [p for p in msg.position]
            self.dof_vel = [p for p in msg.velocity]
            print(f"dof_pos: {self.dof_pos}")
            print(f"dof_vel: {self.dof_vel}")

    def control_loop(self):
        # 计算 PD 控制力矩：\(\tau = K_p (q_{des} - q) - K_d \dot{q}\)
        torques = []
        for i in range(29):
            error_pos = self.default_dof_pos[i] - self.dof_pos[i]
            torque = self.Kp * error_pos - self.Kd * self.dof_vel[i]
            torques.append(torque)
        # 发布力矩命令
        msg = Float64MultiArray(data=torques)
        print(f"torques: {torques}")
        self.joint_effort_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimplePDControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()