#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import struct
import time
import json
from threading import Thread, Event
from queue import Queue, Empty
from socket import socket, AF_INET, SOCK_STREAM


TARGET_PORT = 19199
TARGET_IP = "192.168.50.6"


class AIUIClient:
    def __init__(self, server_ip, server_port=TARGET_PORT):
        self.client_socket = None
        self.server_ip_port = (server_ip, server_port)
        self.stop_event = Event()
        self.msg_id = 1
        self.connect()

    def connect(self):
        while not self.stop_event.is_set():
            try:
                if self.client_socket:
                    self.client_socket.close()
                self.client_socket = socket(AF_INET, SOCK_STREAM)
                self.client_socket.settimeout(5)
                self.client_socket.connect(self.server_ip_port)

                # 初始化配置
                self.send_control_message({"type": "voice", "content": {"enable_voice": True}})
                self.send_control_message({"type": "voice", "content": {"vol_value": 15}})
                self.send_control_message({
                    "type": "aiui_msg",
                    "content": {"msg_type": 9, "arg1": 30000, "arg2": 5000, "params": "timeout_config"}
                })
                self.send_control_message({
                    "type": "aiui_msg",
                    "content": {"msg_type": 8, "arg1": 0, "arg2": 0, "params": "", "data": ""}
                })
                return
            except Exception as e:
                print(f"[AIUI] 连接失败: {e}. 2秒后重试...")
                time.sleep(2)

    def send_control_message(self, control_json: dict):
        payload_bytes = json.dumps(control_json, separators=(',', ':')).encode('utf-8')
        length = len(payload_bytes)
        sync_head = 0xA5
        user_id = 0x01
        msg_type = 0x05
        msg_id = self.msg_id
        self.msg_id = (self.msg_id % 65535) + 1
        header = struct.pack('<BBBHH', sync_head, user_id, msg_type, length, msg_id)
        all_bytes = header + payload_bytes
        checksum = (~sum(all_bytes) + 1) & 0xFF
        packet = all_bytes + bytes([checksum])
        self.client_socket.send(packet)

    def say(self, text: str):
        self.send_control_message({
            "type": "tts",
            "content": {
                "action": "start",
                "text": text,
                "parameters": {
                    "vcn": "x4_lingxiaoqi_oral",
                    "data_type": "text",
                    "scene": "IFLYTEK.tts",
                    "emot": "neutral"
                }
            }
        })
        print(f"[TTS] 播报: {text}")

    def close(self):
        self.stop_event.set()
        if self.client_socket:
            try:
                self.client_socket.close()
            except Exception:
                pass


class AIUITTSNode(Node):
    def __init__(self):
        super().__init__('aiui_tts_node')
        self.aiui_client = None
        self.text_queue = Queue()
        self.shutdown_event = Event()

        # 启动连接线程
        self.conn_thread = Thread(target=self._connect_loop, daemon=True)
        self.conn_thread.start()

        # 订阅 TTS 话题
        self.subscription = self.create_subscription(
            String, '/tts_say', self.tts_callback, 10
        )

    def _connect_loop(self):
        """后台线程：连接 AIUI + 连接成功后补播队列里的消息。"""
        self.get_logger().info(f"尝试连接 Q1: {TARGET_IP}:{TARGET_PORT}")

        while rclpy.ok() and not self.shutdown_event.is_set():
            try:
                self.aiui_client = AIUIClient(TARGET_IP, TARGET_PORT)
                self.get_logger().info("已连接 Q1，开始处理待播报队列。")

                # 连上后：持续消费队列（包含连接前积压的文本）
                while rclpy.ok() and not self.shutdown_event.is_set():
                    try:
                        text = self.text_queue.get(timeout=0.5)
                    except Empty:
                        continue

                    if text == "__STOP__":
                        return

                    try:
                        self.aiui_client.say(text)
                    except Exception as e:
                        self.get_logger().warn(f"TTS 发送失败，将重连：{e}")
                        # 失败了：丢回队列头（简单起见放回尾部）
                        self.text_queue.put(text)
                        # 触发重连
                        try:
                            self.aiui_client.close()
                        except Exception:
                            pass
                        self.aiui_client = None
                        break  # 跳出内层，回到外层重连

            except Exception as e:
                self.get_logger().warn(f"连接失败：{e}，2秒后重试")
                time.sleep(2)

    def tts_callback(self, msg: String):
        # 永远入队，让后台线程统一发送（线程安全 & 自动补播）
        self.text_queue.put(msg.data)

    def destroy_node(self):
        self.shutdown_event.set()
        self.text_queue.put("__STOP__")
        if self.aiui_client:
            self.aiui_client.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AIUITTSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()