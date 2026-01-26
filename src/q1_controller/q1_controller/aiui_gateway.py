#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import struct
import time
from threading import Thread, Event
import socket
from socket import timeout as SocketTimeout

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from utils_log import log

from aiui_gateway_config import (
    AIUI_PORT,
    TOPIC_TTS,
    TOPIC_AIUI_EVENT, TOPIC_AIUI_IAT, TOPIC_AIUI_NLP, TOPIC_AIUI_INTENT
)
from aiui_gateway_tts_engine import StableTTSEngine

from processStrategy import ConfirmProcess, AiuiMessageProcess


class SocketDemoLike:

    def __init__(self, on_event=None):
        self.on_event = on_event
        self.status = "idle"

    # ======== 原版函数：从 main.py 原样复制（不要改） ========

    def get_aiui_type(self, data):
        content = data.get("content", {})
        info = content.get("info", {})
        #兜底
        if isinstance(info, str):
            try:
                info = json.loads(info)
            except Exception:
                return None

        data_array = info.get("data", [])
        #兜底
        if isinstance(data_array, str):
            try:
                data_array = json.loads(data_array)
            except Exception:
                return None

        if len(data_array) > 0:
            data_item = data_array[0]
            params = data_item.get("params", {})
            return params.get("sub")
        return None

    def get_iat_result(self, data):
        words = []
        try:
            ws_list = data.get('content', {}).get('result', {}).get('text', {}).get('ws', [])
            for item in ws_list:
                for cw in item.get('cw', []):
                    words.append(cw.get('w', ''))
            sn = data.get('content', {}).get('result', {}).get('text', {}).get('sn')
            ls = data.get('content', {}).get('result', {}).get('text', {}).get('ls')
            status = 0 if sn == 1 else (2 if ls is True else 1)
            result = ''.join(words)
            if result or status == 2:
                log(f"识别结果是: {result} {status}")
            return result, status
            
        except Exception:
            return '', 1

    def get_nlp_result(self, data):
        try:
            text = data.get('content', {}).get('result', {}).get('nlp', {}).get('text')
            status = data.get('content', {}).get('result', {}).get('nlp', {}).get('status')
            if text is not None and status is not None:
                log(f"大模型回答结果是: {text} {status}")
            return text, status
        except Exception:
            return None, None

    def get_intent_result(self, data):
        text_value = data.get('content', {}).get('result', {}).get('cbm_semantic', {}).get('text')
        if text_value is not None:
            try:
                intent = json.loads(text_value)
                if intent.get('rc') == 0:
                    log(f"技能结果: {intent.get('category', '')}")
                return intent
            except json.JSONDecodeError:
                return None
        return None

    # ======== IO 辅助：与 main.py 同样的帧格式 ========

    def receive_full_data(self, sock, expected_length):
        received = bytearray()
        while len(received) < expected_length:
            try:
                chunk = sock.recv(expected_length - len(received))
                if not chunk:
                    return None
                received.extend(chunk)
            except SocketTimeout:
                return None
        return bytes(received)

    def process_once(self, sock, tts_engine: StableTTSEngine):
        """
        单次处理一帧：等价 main.py 的 process() 内部循环体
        - ConfirmProcess / AiuiMessageProcess 仍然用原版
        - 唤醒时“我在”改成走稳定引擎 push
        """
        # 读头
        recv_data = self.receive_full_data(sock, 7)
        if not recv_data or len(recv_data) < 7:
            return

        sync_head, user_id, msg_type, msg_length, msg_id = struct.unpack('<BBBHH', recv_data)

        # 读 body+checksum
        msg_data = self.receive_full_data(sock, msg_length + 1)
        if not msg_data or len(msg_data) < msg_length + 1:
            return

        msg = msg_data[:msg_length]

        if not (sync_head == 0xA5 and user_id == 0x01):
            return

        # confirm
        if msg_type in (0x01, 0x04):
            ConfirmProcess().process(sock, msg_id)

        success, result = AiuiMessageProcess().process(sock, msg)
        if not success:
            return

        data = json.loads(result)
        # 兜底：如果解析后仍是字符串，说明是双层编码，再loads一次
        if isinstance(data, str):
            data = json.loads(data)

        # 统一抛给上层（网关发布 ROS2 topic）
        if self.on_event:
            self.on_event("raw", data)

        # === 保持 main.py 的“唤醒->我在/休眠->idle”行为 ===
        if data.get("type") == "aiui_event":
            event_type = data.get("content", {}).get("eventType")
            if self.on_event:
                self.on_event("aiui_event", {"eventType": event_type, "raw": data})

            if event_type == 4:
                if self.status == "idle":
                    self.status = "active"
                    log("检测到唤醒词：小勤小勤")
                    tts_engine.push("我在")  
            elif event_type == 5:  # 休眠事件
                log("设备休眠，回到等待状态")
                self.status = "idle"
            elif event_type == 8:  # 会话结束
                log("会话结束，回到等待状态")
                self.status = "idle"

        # === 原版分流：iat/nlp/cbm_semantic ===
        sub = self.get_aiui_type(data)

        if sub == "iat":
            text, status = self.get_iat_result(data)
            if self.on_event:
                self.on_event("iat", {"text": text, "status": status, "raw": data})
        elif sub == "nlp":
            text, status = self.get_nlp_result(data)
            if self.on_event:
                self.on_event("nlp", {"text": text, "status": status, "raw": data})
        elif sub == "cbm_semantic":
            intent = self.get_intent_result(data)
            if intent is not None and self.on_event:
                self.on_event("intent", {"intent": intent, "raw": data})


class AIUIGateway(Node):
    def __init__(self):
        super().__init__('aiui_gateway')

        # TTS 引擎（内部 connect/warmup/worker 都在这里）
        self.tts_engine = StableTTSEngine(AIUI_PORT)

        # ROS2
        self.sub_tts = self.create_subscription(String, TOPIC_TTS, self._on_tts, 10)
        self.pub_event = self.create_publisher(String, TOPIC_AIUI_EVENT, 10)
        self.pub_iat = self.create_publisher(String, TOPIC_AIUI_IAT, 10)
        self.pub_nlp = self.create_publisher(String, TOPIC_AIUI_NLP, 10)
        self.pub_intent = self.create_publisher(String, TOPIC_AIUI_INTENT, 10)

        self._stop = Event()

        # “原版解析器”对象：用 main.py 的函数实现
        self.demo = SocketDemoLike(on_event=self._handle_event)

        self._rx_thread = Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

        self.get_logger().info(f"AIUI Gateway started. port={AIUI_PORT}")

    def _on_tts(self, msg: String):
        text = (msg.data or "").strip()
        if text:
            self.tts_engine.push(text)

    def _handle_event(self, etype: str, payload):
        """把原版处理结果发布成 ROS2 topic"""
        def pub(publisher, obj):
            m = String()
            m.data = json.dumps(obj, ensure_ascii=False)
            publisher.publish(m)

        if etype == "aiui_event":
            pub(self.pub_event, payload)
        elif etype == "iat":
            pub(self.pub_iat, payload)
        elif etype == "nlp":
            pub(self.pub_nlp, payload)
        elif etype == "intent":
            pub(self.pub_intent, payload)
        else:
            # raw/其他
            pub(self.pub_event, {"type": etype, "payload": payload})

    def _get_socket(self):
        """
        复用引擎内部的 socket：避免再建立第二条连接抢占 19199
        引擎里一般是：self.aiui_client.client_socket
        """
        c = getattr(self.tts_engine, "aiui_client", None)
        if c is None:
            return None
        return getattr(c, "client_socket", None)

    def _rx_loop(self):
        self.get_logger().info("RX loop started.")
        time.sleep(0.2)

        while rclpy.ok() and not self._stop.is_set():
            try:
                sock = self._get_socket()
                if sock is None:
                    time.sleep(0.1)
                    continue

                # 防止永久卡死
                try:
                    sock.settimeout(2.0)
                except Exception:
                    pass

                try:
                    self.demo.process_once(sock, self.tts_engine)
                except (json.JSONDecodeError, ValueError, AttributeError, KeyError, TypeError) as e:
                    self.get_logger().warn(f"RX parse/logic error (no connect): {e}")
                

            except socket.timeout:
                # 超时是正常的：表示这段时间没数据，不要重连
                continue

            except (json.JSONDecodeError, ValueError, AttributeError, KeyError, TypeError) as e:
                # 解析/字段错误：不要重连（否则就会 warmup 风暴）
                #self.get_logger().warn(f"RX parse/logic error (no reconnect): {e}")
                continue

            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError) as e:
                # 只有这些才当断线处理
                self.get_logger().warn(f"RX connection error -> reconnect: {e}")
                try:
                    with self.tts_engine._conn_lock:
                        if self.tts_engine.aiui_client:
                            try:
                                self.tts_engine.aiui_client.close()
                            except Exception:
                                pass
                        self.tts_engine.aiui_client = None
                except Exception:
                    pass
                time.sleep(0.2)
                continue
            

        self.get_logger().info("RX loop exited.")

    def destroy_node(self):
        self._stop.set()
        try:
            self.tts_engine.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AIUIGateway()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()