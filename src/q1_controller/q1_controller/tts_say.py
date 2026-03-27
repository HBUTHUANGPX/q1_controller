#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import struct
import time
import json
from threading import Thread, Event, Lock
#from socket import socket, AF_INET, SOCK_STREAM, create_connection, SOCK_DGRAM
import socket

import subprocess
import sys
import ipaddress
from concurrent.futures import ThreadPoolExecutor, as_completed



USE_STATIC_IP = False        # WiFi+MAC发现 => False；固定IP直连 => True
TARGET_PORT = 19199
TARGET_IP = "170.20.10.2"  # 改成设置的的开发板IP；推荐固定租约后用 192.168.50.x
TRUSTED_MAC = "fc:23:cd:f3:c1:6c".lower()      #无线mac
SCAN_TIMEOUT = 8
MAX_WORKERS = 50

# ============================
# 连接/发送参数（可调）
# ============================
SOCKET_TIMEOUT_S = 5        # socket超时
RECONNECT_DELAY_S = 1.0     # 重连间隔

# ============================
# 播放稳定性关键参数（可调）
# ============================
STOP_SETTLE_S = 0.15        # stop 后等待 AIUI 状态切换（稳定关键，建议 0.12~0.20）
# 冷启动阶段更稳一点：更长 settle + start 补发一次
COLD_START_WINDOW_S = 12.0  # 冷启动阶段持续时长（秒）
COLD_STOP_SETTLE_S = 0.28   # 冷启动阶段 stop 后等待（更长更稳）
COLD_START_BOOST_DELAY_S = 0.10  # 冷启动阶段补发 start 前等待

# ==========================================================
# [WARMUP] 启动热身/就绪门禁参数（可调）
# ==========================================================
AUTO_WARMUP_ON_START = True   # [WARMUP] 节点启动后自动连接并热身（不依赖第一次按键）
READY_DELAY_S = 2.0           # [WARMUP] warm-up 后至少等这么久再放行真实播报（建议 1.5~4.0）

# [WARMUP] 强力热身：推荐用更明显的内容，"嗯" 太短经常听不到
WARMUP_TEXT = "1"             # [WARMUP] 可选： "测试" / "一" / "1"
WARMUP_REPEAT = 4             # [WARMUP] 连发次数（建议 2~4）
WARMUP_GAP_S = 0.6           # [WARMUP] 每次 start 后等待，让对端有机会真正开始出声
WARMUP_STOP_BETWEEN = True    # [WARMUP] 前几次之间是否 stop（建议 True）
WARMUP_STOP_GAP_S = 0.08      # [WARMUP] stop 后等待

# ==========================================================
# [BOOST] 刚 warm-up/刚重连阶段补发一次 start（比每次retry温和）
# ==========================================================
ENABLE_BOOST = True
BOOST_WINDOW_S = 8.0          # [BOOST] warm-up 完成后的 N 秒内启用补发
BOOST_DELAY_S = 0.06          # [BOOST] 补发前短等待


#==============连接开发板============
# 获取本机网段
def get_local_network():
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))
            local_ip = s.getsockname()[0]
        network = ipaddress.IPv4Network(f"{local_ip}/24", strict=False)
        print(f"本机 IP: {local_ip} | 扫描网段: {network}")
        return network, local_ip
    except Exception as e:
        print(f"获取本机网络失败: {e}，使用默认 192.168.1.0/24")
        return ipaddress.IPv4Network("192.168.1.0/24"), "192.168.1.1"


# 从 ARP 表获取 MAC（Linux）
def get_mac_by_ip(ip):
    try:
        with open("/proc/net/arp", "r") as f:
            for line in f:
                parts = line.split()
                if len(parts) >= 4 and parts[0] == ip:
                    mac = parts[3].lower()
                    if mac != "00:00:00:00:00:00":
                        return mac
    except Exception as e:
        print(f" 读取 ARP 表出错: {e}")
    return None


# 探测主机：用 ping 触发 ARP，再校验 MAC
def probe_host(ip_str):
    #print(f"尝试探测 {ip_str} ...", end="\r")
    # 策略 1: 先尝试 ping
    try:
        result = subprocess.run(["ping", "-c", "1", "-W", "1", ip_str],
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if result.returncode == 0:
            time.sleep(0.1)
            mac = get_mac_by_ip(ip_str)
            if mac == TRUSTED_MAC:
                print(f"ping匹配到目标 MAC! IP={ip_str}, MAC={mac}")
                return ip_str
    except:
        pass

    # 策略 2: ping 失败，尝试 TCP 连接 19199
    try:
        with socket.create_connection((ip_str, TARGET_PORT), timeout=0.5):
            pass  # 触发 ARP
        time.sleep(0.1)
        mac = get_mac_by_ip(ip_str)
        if mac == TRUSTED_MAC:
            print(f"TCP匹配到目标 MAC! IP={ip_str}, MAC={mac}")
            return ip_str
    except Exception as e:
        #print(f"TCP probe {ip_str} failed: {e}")
        pass

    return None


# [NEW STATIC IP] 新增：固定 IP 可达性检查（不改变业务，只用于决定是否可以直连）
def check_static_ip_reachable(ip_str, port=TARGET_PORT):
    """
    仅用于：固定 IP 模式下，启动前快速判断是否可连。
    - 先试 TCP 端口（最快且最贴近真实需求）
    - TCP 不通再 ping（有些设备禁 ping，ping 失败不代表不可用）
    """
    # 1) TCP 探测
    try:
        with socket.create_connection((ip_str, port), timeout=1.0):
            return True
    except Exception:
        pass

    # 2) Ping 探测（可选）
    try:
        result = subprocess.run(["ping", "-c", "1", "-W", "1", ip_str],
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        # ping 通只能说明网络通，不代表端口通
        return result.returncode == 0
    except Exception:
        return False
    
def resolve_target_ip():
    # 1) 固定 IP 模式：直接用 TARGET_IP
    if USE_STATIC_IP:
        # 可选：复用 main 的 check_static_ip_reachable
        # if not check_static_ip_reachable(TARGET_IP, port=TARGET_PORT):
        #     raise RuntimeError(f"STATIC_IP unreachable: {TARGET_IP}:{TARGET_PORT}")
        return TARGET_IP

    # 2) 扫描模式：复用 main 的扫描代码（ThreadPoolExecutor + probe_host）
    network, local_ip = get_local_network()
    hosts = [str(ip) for ip in network.hosts() if str(ip) != local_ip]

    found = None
    with ThreadPoolExecutor(max_workers=MAX_WORKERS) as executor:
        futures = {executor.submit(probe_host, ip): ip for ip in hosts}
        try:
            for future in as_completed(futures, timeout=SCAN_TIMEOUT):
                result = future.result()
                if result:
                    found = result
                    return result
        except Exception as e:
            raise RuntimeError(f"scan timeout/error: {e}")

    if not found:
        raise RuntimeError(f"MAC scan failed, TRUSTED_MAC={TRUSTED_MAC}")

    return found




class AIUIClient:
    def __init__(self, server_ip, server_port=TARGET_PORT):
        self.server_ip_port = (server_ip, server_port)
        self.client_socket = None
        self.stop_event = Event()
        self.msg_id = 1
        self._send_lock = Lock()  # 串行化发送，避免并发写 socket 造成包错乱
        self.connect()

    def connect(self):
        while not self.stop_event.is_set():
            try:
                if self.client_socket:
                    try:
                        self.client_socket.close()
                    except Exception:
                        pass
 
                self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.client_socket.settimeout(SOCKET_TIMEOUT_S)
                self.client_socket.connect(self.server_ip_port)

                # 初始化配置
                self.send_control_message({"type": "voice", "content": {"enable_voice": True}})
                self.send_control_message({"type": "voice", "content": {"vol_value": 15}})
                '''
                self.send_control_message({
                    "type": "aiui_msg",
                    "content": {"msg_type": 9, "arg1": 30000, "arg2": 5000, "params": "timeout_config"}
                })
                self.send_control_message({
                    "type": "aiui_msg",
                    "content": {"msg_type": 8, "arg1": 0, "arg2": 0, "params": "", "data": ""}
                })
                '''
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

        with self._send_lock:
            self.client_socket.send(packet)

    # 抢占停止
    def tts_stop(self):
        self.send_control_message({"type": "tts", "content": {"action": "stop"}})

    # 保持原来的 TTS start 格式
    def tts_start(self, text: str):
        self.send_control_message({
            "type": "tts",
            "content": {
                "action": "start",
                "text": text,
                "parameters": {
                    "vcn": "x4_lingxiaoying_em_v2",
                    "data_type": "text",
                    "scene": "IFLYTEK.tts",
                    "emot": "happy"
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

        # 最新一句 + 触发计数（同一句也能重复播）
        self._lock = Lock()
        self._latest_text = ""
        self._latest_id = 0

        # 连接/热身互斥锁：避免 init 线程和 worker 线程并发 connect/warm-up
        self._conn_lock = Lock()

        # [WARMUP] 就绪门禁 + boost 窗口
        self._ready_at = 0.0
        self._boost_until = 0.0

        # 冷启动窗口（启动后前 N 秒更稳一点）
        self._startup_until = time.time() + COLD_START_WINDOW_S

        # 工作线程：负责发送（统一出口，避免乱序）
        self.worker = Thread(target=self._worker_loop, daemon=True)
        self.worker.start()

        # [WARMUP] 启动后自动连接并热身（不依赖第一次按键）
        if AUTO_WARMUP_ON_START:
            self._init_thread = Thread(target=self._auto_init_aiui, daemon=True)
            self._init_thread.start()

        # 订阅 TTS 话题
        self.subscription = self.create_subscription(
            String, '/tts_say', self.tts_callback, 10
        )

        self.get_logger().info(f"TTS Node started. static={USE_STATIC_IP}, Port={TARGET_PORT}")

    def tts_callback(self, msg: String):
        text = (msg.data or "").strip()
        if not text:
            return

        with self._lock:
            self._latest_text = text
            self._latest_id += 1  # 即使文本相同也算一次新触发

        self.get_logger().info(f"[RECV] {text}")

    def _auto_init_aiui(self):
        """
        [WARMUP] 程序启动后自动连接 AIUI 并完成 warm-up
        """
        self.get_logger().info("[AUTO_INIT] start")
        try:
            self._ensure_connected()
            self.get_logger().info("[AUTO_INIT] done")
        except Exception as e:
            self.get_logger().warn(f"[AUTO_INIT] failed: {e}")

    def _ensure_connected(self):
        """
        确保 TCP 已连接；首次连接 / 重连后做 warm-up 并设置就绪门禁
        """
        with self._conn_lock:  # 互斥：禁止并发 connect/warm-up
            if self.aiui_client is not None:
                return

            self.get_logger().info("Connecting AIUI...")
            ip = resolve_target_ip()
            self.get_logger().info(f"AIUI target ip = {ip} (static={USE_STATIC_IP})")
            self.aiui_client = AIUIClient(ip, TARGET_PORT)

            self.get_logger().info("AIUI connected.")

            # ==========================
            # [WARMUP] 连接成功后热身（强力 xN）
            # ==========================
            self._ready_at = time.time() + READY_DELAY_S
            try:
                self.get_logger().info(f"Warming up TTS (x{WARMUP_REPEAT})...")

                for i in range(WARMUP_REPEAT):
                    self.aiui_client.tts_start(WARMUP_TEXT)
                    self.get_logger().info(f"[WARMUP] start {i+1}/{WARMUP_REPEAT} sent")
                    time.sleep(WARMUP_GAP_S)

                    # 前几次之间 stop 一下，避免叠加；最后一次通常不 stop，让它有机会真正播出来
                    if WARMUP_STOP_BETWEEN and i < WARMUP_REPEAT - 1:
                        self.aiui_client.tts_stop()
                        self.get_logger().info(f"[WARMUP] stop {i+1}/{WARMUP_REPEAT} sent")
                        time.sleep(WARMUP_STOP_GAP_S)

                self.get_logger().info("TTS warm-up done.")
            except Exception as e:
                self.get_logger().warn(f"TTS warm-up failed: {e}")

            # [BOOST] warm-up 完成后短窗口内启用补发
            if ENABLE_BOOST:
                self._boost_until = time.time() + BOOST_WINDOW_S
            else:
                self._boost_until = 0.0

            self.get_logger().info(
                f"[WARMUP] ready_at={self._ready_at:.3f}, boost_until={self._boost_until:.3f}"
            )

    def _worker_loop(self):
        """
        发送线程（稳定版）：
        - 只播最新触发（通过 latest_id 判断新触发）
        - 每次新触发都 stop -> wait -> start（打断旧播报）
        - 启动/重连后有就绪门禁，避免“发了但无声”的冷启动阶段
        - 启动/重连后的短窗口内补发一次 start，提高可靠性
        - 冷启动窗口（程序刚启动前 N 秒）额外更稳：更长 settle + 再补发一次
        """
        last_seen_id = 0

        while rclpy.ok() and not self.shutdown_event.is_set():
            with self._lock:
                text = self._latest_text
                tid = self._latest_id

            if not text or tid == last_seen_id:
                time.sleep(0.01)
                continue

            try:
                self._ensure_connected()

                # [WARMUP] 就绪门禁：未到时间就先不播
                # 注意：这里不能更新 last_seen_id，否则会把触发吞掉
                if time.time() < self._ready_at:
                    time.sleep(0.01)
                    continue

                # stop -> wait -> start
                self.aiui_client.tts_stop()

                # 冷启动窗口：给更长 settle，减少吞 start / 无声
                settle = COLD_STOP_SETTLE_S if time.time() < self._startup_until else STOP_SETTLE_S
                time.sleep(settle)

                self.aiui_client.tts_start(text)


                # 只有成功发送后才更新 last_seen_id
                last_seen_id = tid
                self.get_logger().info(f"[SEND] {text}")

            except Exception as e:
                self.get_logger().warn(f"TTS error: {e} -> reconnect")
                try:
                    if self.aiui_client:
                        self.aiui_client.close()
                except Exception:
                    pass
                self.aiui_client = None

                # 重连后会重新 warm-up / 重新设置 ready_at / boost_until
                self._ready_at = 0.0
                self._boost_until = 0.0

                time.sleep(0.5)

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