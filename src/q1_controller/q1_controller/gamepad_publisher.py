#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import pygame
from q1_controller.msg import XboxJoy  # 导入自定义消息

class GamepadPublisher(Node):
    def __init__(self):
        super().__init__('gamepad_publisher')
        self.publisher_ = self.create_publisher(XboxJoy, '/gamepad_data', 10)
        timer_period = 1/100  # 发布频率为10Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # 初始化Pygame和手柄
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            self.get_logger().error('No gamepad detected!')
            return
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        
        self.get_logger().info('Gamepad initialized: ' + self.joystick.get_name())

    def timer_callback(self):
        pygame.event.pump()  # 更新Pygame事件
        
        # 读取轴数据
        axes = [self.joystick.get_axis(i) for i in range(self.joystick.get_numaxes())]
        
        # 读取按钮数据
        buttons = [self.joystick.get_button(i) for i in range(self.joystick.get_numbuttons())]
        
        # 读取帽子数据
        hats = []
        for i in range(self.joystick.get_numhats()):
            hat = self.joystick.get_hat(i)
            hats.extend(hat)  # 帽子数据为元组 (x, y)，扩展为列表
        
        # 创建并发布消息
        msg = XboxJoy()
        msg.axes = axes
        msg.buttons = buttons
        msg.hats = hats
        self.publisher_.publish(msg)
        
        # self.get_logger().info('Published gamepad data')

def main(args=None):
    rclpy.init(args=args)
    node = GamepadPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    pygame.quit()

if __name__ == '__main__':
    main()