#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import struct
import time
import json
from threading import Thread, Event, Lock
from queue import Queue, Empty
from socket import socket, AF_INET, SOCK_STREAM


TARGET_PORT = 19199
TARGET_IP = "192.168.50.6"  # [FIX] 改成你当前可用的开发板IP；推荐固定租约后用 192.168.50.2


# ============================
# [FIX] 低延迟策略参数（可调）
# ============================
KEEP_ONLY_LATEST = True     # [FIX] 本地只保留最新一句，避免“按很多下后突然冒出旧句子”
PREEMPT_STOP = True         # [FIX] 每次start前先stop，减少AIUI内部排队
STOP_SETTLE_S = 0.08        # [FIX] stop后等待AIUI状态机切换（太小可能无效，太大增加延迟）
MIN_GAP_S = 0.05            # [FIX] 两次发送最小间隔，避免过密触发导致对端卡住
SOCKET_TIMEOUT_S = 5        # [FIX] socket超时
RECONNECT_DELAY_S = 1.0     # [FIX] 重连间隔
# ============================


class AIUIClient:
    def __init__(self, server_ip, server_port=TARGET_PORT):
        self.server_ip_port = (server_ip, server_port)
        self.client_socket = None
        self.stop_event = Event()
        self.msg_id = 1
        self._send_lock = Lock()  # [FIX] 串行化发送，避免并发写socket造成包错乱
        self.connect()

    def connect(self):
        while not self.stop_event.is_set():
            try:
                if self.client_socket:
                    try:
                        self.client_socket.close()
                    except Exception:
                        pass

                self.client_socket = socket(AF_INET, SOCK_STREAM)
                self.client_socket.settimeout(SOCKET_TIMEOUT_S)
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
                print(f"[AIUI] 连接失败: {e}. {RECONNECT_DELAY_S}秒后重试...")
                time.sleep(RECONNECT_DELAY_S)

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

        with self._send_lock:  # [FIX]
            self.client_socket.send(packet)

    # [FIX] 新增：抢占停止
    def tts_stop(self):
        self.send_control_message({"type": "tts", "content": {"action": "stop"}})

    # 保持你原来的 TTS start 格式
    def tts_start(self, text: str):
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
        self.shutdown_event = Event()

        self.text_queue: "Queue[str]" = Queue()

        # [FIX] 工作线程：负责连接 + 消费队列 + 发送（统一出口，避免乱序）
        self.worker = Thread(target=self._worker_loop, daemon=True)
        self.worker.start()

        self.subscription = self.create_subscription(
            String, '/tts_say', self.tts_callback, 10
        )

        self.get_logger().info(f"TTS Node started. Target={TARGET_IP}:{TARGET_PORT}")

        self._last_send_ts = 0.0  # [FIX] 最小发送间隔

    def tts_callback(self, msg: String):
        text = (msg.data or "").strip()
        if not text:
            return

        # [FIX] 只保留最新一句（清空队列）
        if KEEP_ONLY_LATEST:
            try:
                while True:
                    self.text_queue.get_nowait()
            except Empty:
                pass

        self.text_queue.put(text)
        # 这个日志用于确认“节点确实收到了按键触发的消息”
        self.get_logger().info(f"[RECV] {text}")  # [FIX]

    def _ensure_connected(self):
        if self.aiui_client is None:
            self.get_logger().info("Connecting AIUI...")
            self.aiui_client = AIUIClient(TARGET_IP, TARGET_PORT)
            self.get_logger().info("AIUI connected.")

    def _worker_loop(self):
        while rclpy.ok() and not self.shutdown_event.is_set():
            try:
                text = self.text_queue.get(timeout=0.2)
            except Empty:
                continue

            # [FIX] 如果队列里又来了更新的句子，只保留最后一句（避免旧句子延迟冒出）
            if KEEP_ONLY_LATEST:
                try:
                    while True:
                        newer = self.text_queue.get_nowait()
                        text = newer
                except Empty:
                    pass

            try:
                self._ensure_connected()

                # [FIX] 最小发送间隔（很小，只为避免过密触发卡住）
                now = time.time()
                gap = (self._last_send_ts + MIN_GAP_S) - now
                if gap > 0:
                    time.sleep(gap)

                # [FIX] 抢占：stop -> 等 -> start
                if PREEMPT_STOP:
                    self.aiui_client.tts_stop()
                    time.sleep(STOP_SETTLE_S)

                self.aiui_client.tts_start(text)
                self._last_send_ts = time.time()
                self.get_logger().info(f"[SEND] {text}")

            except Exception as e:
                # [FIX] 发送失败：重连 + 把最新一句放回去
                self.get_logger().warn(f"TTS send error: {e} -> reconnect")
                try:
                    if KEEP_ONLY_LATEST:
                        # 只放回最新一句
                        try:
                            while True:
                                self.text_queue.get_nowait()
                        except Empty:
                            pass
                    self.text_queue.put(text)
                except Exception:
                    pass

                try:
                    if self.aiui_client:
                        self.aiui_client.close()
                except Exception:
                    pass
                self.aiui_client = None
                time.sleep(RECONNECT_DELAY_S)

    def destroy_node(self):
        self.shutdown_event.set()
        try:
            if self.aiui_client:
                self.aiui_client.close()
        except Exception:
            pass
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