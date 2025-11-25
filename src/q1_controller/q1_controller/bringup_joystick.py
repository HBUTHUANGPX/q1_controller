#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from std_srvs.srv import Trigger  # 用于振动服务的简单触发
import pygame
import time
from q1_controller.msg import (
    MultiMotorCommand,
    SingleMotorCommand,
    Dataset,
    XboxJoy,
)  # 新增：自定义消息导入
class XboxJoyNode(Node):
    def __init__(self):
        super().__init__('xbox_joy_node')
        self.publisher_ = self.create_publisher(XboxJoy, '/xbox_joy', 10)
        self.timer = self.create_timer(0.01, self.timer_callback)  # 10Hz 发布频率

        # 初始化Pygame Joystick模块
        pygame.init()
        pygame.joystick.init()

        # 检查模块初始化状态
        if not pygame.joystick.get_init():
            self.get_logger().error('Pygame Joystick模块未正确初始化')
            rclpy.shutdown()
            return

        # 检查连接的手柄数量
        joystick_count = pygame.joystick.get_count()
        if joystick_count == 0:
            self.get_logger().error('未检测到手柄设备')
            rclpy.shutdown()
            return

        # 实例化第一个手柄
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

        # 检查手柄初始化状态
        if not self.joystick.get_init():
            self.get_logger().error('手柄初始化失败')
            rclpy.shutdown()
            return

        # 记录手柄详细信息
        self.get_logger().info(f'已连接手柄: {self.joystick.get_name()}')
        self.get_logger().info(f'手柄ID: {self.joystick.get_id()}')
        self.get_logger().info(f'实例ID: {self.joystick.get_instance_id()}')
        self.get_logger().info(f'GUID: {self.joystick.get_guid()}')
        self.get_logger().info(f'电源水平: {self.joystick.get_power_level()}')
        self.get_logger().info(f'轴数量: {self.joystick.get_numaxes()}')
        self.get_logger().info(f'按钮数量: {self.joystick.get_numbuttons()}')
        self.get_logger().info(f'帽子数量: {self.joystick.get_numhats()}')
        self.get_logger().info(f'轨迹球数量: {self.joystick.get_numballs()}')

        # 创建振动服务（/trigger_rumble），使用Trigger服务类型进行简单触发
        self.rumble_srv = self.create_service(Trigger, '/trigger_rumble', self.rumble_callback)
        self.get_logger().info('振动服务 /trigger_rumble 已启用')

    def timer_callback(self):
        pygame.event.pump()  # 更新Pygame事件队列

        # 读取轴值
        axes = [self.joystick.get_axis(i) for i in range(self.joystick.get_numaxes())]

        # 读取按钮状态
        buttons = [self.joystick.get_button(i) for i in range(self.joystick.get_numbuttons())]

        # 读取帽子状态
        hats = []
        for i in range(self.joystick.get_numhats()):
            hat = self.joystick.get_hat(i)
            hats.extend([hat[0], hat[1]])  # 展平为列表

        # 如果有轨迹球，读取其数据（对于Xbox手柄通常为空列表）
        balls = []
        for i in range(self.joystick.get_numballs()):
            ball = self.joystick.get_ball(i)
            balls.extend([ball[0], ball[1]])

        # 填充消息（注：当前消息未包括balls；若需扩展msg文件，可添加float32[] balls字段）
        msg = XboxJoy()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'xbox_joy'
        msg.axes = axes
        msg.buttons = buttons
        msg.hats = hats

        # 发布消息
        self.publisher_.publish(msg)
        # self.get_logger().debug('已发布Xbox手柄数据')  # 使用debug级别避免日志过载

    def rumble_callback(self, request, response):
        # 触发振动：低频和高频强度为0.5，持续500ms
        if self.joystick.rumble(0.5, 0.5, 500):
            self.get_logger().info('振动已触发')
            time.sleep(0.5)  # 等待振动完成（可选，非阻塞）
            self.joystick.stop_rumble()  # 显式停止振动
            response.success = True
            response.message = '振动执行成功'
        else:
            self.get_logger().warn('振动失败：手柄不支持或错误')
            response.success = False
            response.message = '振动执行失败'
        return response

    def destroy_node(self):
        self.joystick.quit()
        pygame.joystick.quit()
        pygame.quit()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = XboxJoyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()