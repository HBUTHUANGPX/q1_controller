#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from q1_controller.msg import XboxJoy
from std_msgs.msg import String

class VoiceTriggerNode(Node):
    def __init__(self):
        super().__init__('voice_trigger')
        self.b_was_pressed = False
        self.a_was_pressed = False
        self.tts_pub = self.create_publisher(String, '/tts_say', 10)
        self.joy_sub = self.create_subscription(
            XboxJoy, '/gamepad_data', self.joy_callback, 10
        )
        self.get_logger().info("VoiceTrigger 启动，监听 B 键和 A 键...")

    def joy_callback(self, msg):
        if len(msg.buttons) <= 1:
            return
        b_is_pressed = msg.buttons[1] > 0
        a_is_pressed = msg.buttons[0] > 0
        if not self.b_was_pressed and b_is_pressed: # B按键：拜年
            self.tts_pub_trigger("华勤未来机器人给大家拜年啦")
            # self.tts_pub_trigger("Q1给大家拜年啦")
        self.b_was_pressed = b_is_pressed

        if not self.a_was_pressed and a_is_pressed: # A按键：打招呼
            self.tts_pub_trigger("我是华勤未来机器人，Q1，很高兴见到你")
            # self.tts_pub_trigger("你好，我是Q1机器人，很高兴见到你")
        self.a_was_pressed = a_is_pressed

    def tts_pub_trigger(self,text = "你好，我是Q1机器人，很高兴见到你"):
        self.tts_pub.publish(String(data=text))
        self.get_logger().info(f"触发语音: {text}")
def main():
    rclpy.init()
    node = VoiceTriggerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()