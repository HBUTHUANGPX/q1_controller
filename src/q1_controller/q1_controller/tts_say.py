#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import struct
import time
import json
from threading import Thread, Event, Lock
import socket

import subprocess
import sys
import ipaddress
from concurrent.futures import ThreadPoolExecutor, as_completed


USE_STATIC_IP = False       # 多网口扫描端口 => False；测试用 固定IP直连 => True
TARGET_PORT = 19199
TARGET_IP = "192.168.50.8"  # 改成设置的的开发板IP；推荐固定租约后用 192.168.50.x
TRUSTED_MAC = "2a:de:ef:f7:8d:6b".lower()      #无线mac
SCAN_TIMEOUT = 8
MAX_WORKERS = 50

# ============================
# 连接/发送参数（可调）
# ============================
SOCKET_TIMEOUT_S = 5        # socket超时
RECONNECT_DELAY_S = 1.0     # 重连间隔

# ============================
# 【简化方案】空闲重建参数（核心改动）
# ============================
IDLE_RECONNECT_THRESHOLD = 90.0  # 空闲超过90秒就强制重建连接（避免Broken pipe）
# 根据你的日志，5分钟会出问题，所以设为1.5分钟保守一点

# ============================
# 播放稳定性关键参数（可调）
# ============================
STOP_SETTLE_S = 0.15        # stop 后等待 AIUI 状态切换（稳定关键，建议 0.12~0.20）
# 冷启动阶段更稳一点：更长 settle + start 补发一次
COLD_START_WINDOW_S = 6.0   # 【简化】从12秒减到6秒，因为重建频率高了
COLD_STOP_SETTLE_S = 0.28   # 冷启动阶段 stop 后等待（更长更稳）
COLD_START_BOOST_DELAY_S = 0.10  # 冷启动阶段补发 start 前等待

# ==========================================================
# [WARMUP] 启动热身/就绪门禁参数（可调）
# ==========================================================
AUTO_WARMUP_ON_START = True   # [WARMUP] 节点启动后自动连接并热身（不依赖第一次按键）
READY_DELAY_S = 1.0           # 【简化】从2.0减到1.0秒，加快响应

# [WARMUP] 强力热身：推荐用更明显的内容，"嗯" 太短经常听不到
WARMUP_TEXT = "1"             # [WARMUP] 可选： "测试" / "一" / "1"
WARMUP_REPEAT = 2             # 【简化】从4减到2次，加快启动
WARMUP_GAP_S = 0.4            # 【简化】从0.6减到0.4秒
WARMUP_STOP_BETWEEN = True    # [WARMUP] 前几次之间是否 stop（建议 True）
WARMUP_STOP_GAP_S = 0.08      # [WARMUP] stop 后等待

# ==========================================================
# [BOOST] 刚 warm-up/刚重连阶段补发一次 start（比每次retry温和）
# ==========================================================
ENABLE_BOOST = True
BOOST_WINDOW_S = 5.0          # 【简化】从8.0减到5.0秒
BOOST_DELAY_S = 0.06          # [BOOST] 补发前短等待


#==============连接开发板============
def _get_default_route_if() -> str | None:
    """返回默认路由接口名（公司网口/上网口），用于排除"""
    try:
        out = subprocess.check_output(["ip", "route", "show", "default"], text=True).strip()
        # 例：default via 10.130.96.1 dev eno1 proto dhcp metric 100
        for line in out.splitlines():
            parts = line.split()
            if "dev" in parts:
                return parts[parts.index("dev") + 1]
    except Exception:
        pass
    return None


def _carrier_is_up(ifname: str) -> bool:
    try:
        with open(f"/sys/class/net/{ifname}/carrier", "r") as f:
            return f.read().strip() == "1"
    except Exception:
        return False


def _list_control_networks() -> list[tuple[str, ipaddress.IPv4Network, str]]:
    """
    返回控制口列表：[(ifname, network, local_ip), ...]
    过滤规则：
      - 排除 lo / wlan* / docker* / veth* / br-* 等
      - 排除默认路由口
      - 必须 carrier=1（网线插着）
      - 必须存在 IPv4 地址
    """
    default_if = _get_default_route_if()

    try:
        j = subprocess.check_output(["ip", "-j", "addr", "show"], text=True)
        data = json.loads(j)
    except Exception as e:
        print(f"[DISCOVERY] ip -j addr show failed: {e}")
        return []

    ctrl = []
    for it in data:
        ifname = it.get("ifname", "")
        if not ifname:
            continue

        # quick exclude
        if ifname == "lo" or ifname.startswith("wl") or ifname.startswith("docker") or ifname.startswith("veth") or ifname.startswith("br-"):
            continue
        if default_if and ifname == default_if:
            continue
        if not _carrier_is_up(ifname):
            continue

        addrs = it.get("addr_info", [])
        for a in addrs:
            if a.get("family") != "inet":
                continue
            local_ip = a.get("local")
            prefixlen = a.get("prefixlen")
            if not local_ip or prefixlen is None:
                continue

            # 例如 local_ip=192.168.50.1, prefixlen=24
            net = ipaddress.IPv4Network(f"{local_ip}/{prefixlen}", strict=False)
            ctrl.append((ifname, net, local_ip))
            break  # 一个接口取一个 IPv4 即可

    return ctrl


def _try_connect(ip_str: str, port: int, timeout_s: float = 0.25) -> bool:
    try:
        with socket.create_connection((ip_str, port), timeout=timeout_s):
            return True
    except Exception:
        return False


def _scan_19199_on_network(net: ipaddress.IPv4Network, local_ip: str, port: int) -> str | None:
    # 只扫可用主机，跳过本机 IP
    hosts = [str(ip) for ip in net.hosts() if str(ip) != local_ip]

    with ThreadPoolExecutor(max_workers=MAX_WORKERS) as ex:
        futs = {ex.submit(_try_connect, ip, port): ip for ip in hosts}
        for fut in as_completed(futs, timeout=SCAN_TIMEOUT):
            if fut.result():
                return futs[fut]
    return None

def resolve_target_ip():
    # 1) 固定 IP 模式：保留给调试用
    if USE_STATIC_IP:
        return TARGET_IP

    # 2) 多控制口扫描：每个控制口子网内找 19199
    ctrl_nets = _list_control_networks()
    if not ctrl_nets:
        raise RuntimeError("[DISCOVERY] No control interfaces found (check carrier/ip).")

    print("[DISCOVERY] candidate control nets:")
    for ifname, net, local_ip in ctrl_nets:
        print(f"  - {ifname}: {local_ip} in {net}")

    # 逐网口扫描（你也可以改成并发扫多个网段；先串行更稳、日志更清楚）
    for ifname, net, local_ip in ctrl_nets:
        print(f"[DISCOVERY] scanning {ifname} {net} for port {TARGET_PORT} ...")
        found = _scan_19199_on_network(net, local_ip, TARGET_PORT)
        if found:
            print(f"[DISCOVERY] FOUND {found}:{TARGET_PORT} on {ifname} ({net})")
            return found

    raise RuntimeError("[DISCOVERY] no device with port 19199 found on control nets.")


class AIUIClient:
    def __init__(self, server_ip, server_port=TARGET_PORT):
        self.server_ip_port = (server_ip, server_port)
        self.client_socket = None
        self.stop_event = Event()
        self.msg_id = 1
        self._send_lock = Lock()  # 串行化发送，避免并发写 socket 造成包错乱
        self._last_send_mono = 0.0   # 上次成功 sendall 的时间
        
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

                # 减少小包延迟，避免影响 stop->start 的快速响应
                self.client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

                # 启用 TCP keepalive，帮助发现"死连接"
                self.client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
                try:
                    self.client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 10)
                    self.client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 3)
                    self.client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 3)
                except Exception:
                    pass  # 某些系统不支持这些选项也没关系

                # 初始化配置
                self.send_control_message({"type": "voice", "content": {"enable_voice": True}})
                self.send_control_message({"type": "voice", "content": {"vol_value": 15}})
                
                return
            except Exception as e:
                print(f"[AIUI] 连接失败: {e}. {RECONNECT_DELAY_S}秒后重试...")
                time.sleep(RECONNECT_DELAY_S)

    def _is_socket_alive(self) -> bool:
        """简单检查：尝试非阻塞 recv，只对端主动关闭才返回 False"""
        if not self.client_socket:
            return False
        try:
            data = self.client_socket.recv(1, socket.MSG_PEEK | socket.MSG_DONTWAIT)
            if data == b'':
                return False
        except BlockingIOError:
            return True
        except Exception:
            return False
        return True

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
            # 检查主连接是否被对端关闭
            if not self._is_socket_alive():
                raise ConnectionError("socket not alive (peer closed)")

            try:
                self.client_socket.sendall(packet)
                self._last_send_mono = time.monotonic()
            except (socket.timeout, BrokenPipeError, ConnectionResetError, OSError) as e:
                raise ConnectionError(f"send failed: {e}")

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
                    "vcn": "yifeng",
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

        # 【简化方案】统计信息
        self._send_count = 0
        self._reconnect_count = 0

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

        self.get_logger().info(f"TTS Node started. static={USE_STATIC_IP}, Port={TARGET_PORT}, idle_threshold={IDLE_RECONNECT_THRESHOLD}s")

    def tts_callback(self, msg: String):
        text = (msg.data or "").strip()
        if not text:
            return

        with self._lock:
            self._latest_text = text
            self._latest_id += 1  # 即使文本相同也算一次新触发

        self.get_logger().info(f"[RECV] {text}")

    def _auto_init_aiui(self):
        self.get_logger().info("[AUTO_INIT] start")
        while not self.shutdown_event.is_set():
            try:
                self._ensure_connected()
                self.get_logger().info("[AUTO_INIT] done")
                return
            except Exception as e:
                self.get_logger().warn(f"[AUTO_INIT] failed: {e} (retry in 2s)")
                time.sleep(2.0)

    def _ensure_connected(self):
        """
        确保 TCP 已连接；首次连接 / 重连后做 warm-up 并设置就绪门禁
        """
        with self._conn_lock:  # 互斥：禁止并发 connect/warm-up
            if self.aiui_client is not None:
                # 健康检查：如果 socket 已死，关闭重建
                try:
                    if not self.aiui_client._is_socket_alive():
                        self.get_logger().warn("AIUI socket not alive -> reconnect")
                        self.aiui_client.close()
                        self.aiui_client = None
                    else:
                        return
                except Exception:
                    self.aiui_client = None

            self._reconnect_count += 1
            self.get_logger().info(f"Connecting AIUI... (reconnect_count={self._reconnect_count})")
            ip = resolve_target_ip()
            self.get_logger().info(f"AIUI target ip = {ip} (static={USE_STATIC_IP})")
            self.aiui_client = AIUIClient(ip, TARGET_PORT)

            self.get_logger().info("AIUI connected.")

            # ==========================
            # [WARMUP] 连接成功后热身（简化版 x2）
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
        last_seen_id = 0
        last_activity_time = time.time()  # 【统一】上次任何活动时间（发送或连接）

        while rclpy.ok() and not self.shutdown_event.is_set():
            with self._lock:
                text = self._latest_text
                tid = self._latest_id

            if not text or tid == last_seen_id:
                time.sleep(0.01)
                continue

            idle_time = time.time() - last_activity_time
            
            try:
                if self.aiui_client and idle_time > IDLE_RECONNECT_THRESHOLD:
                    self.get_logger().info(
                        f"[IDLE_DETECT] Idle {idle_time:.1f}s > {IDLE_RECONNECT_THRESHOLD}s, "
                        f"force reconnect (sends={self._send_count}, reconnects={self._reconnect_count})"
                    )
                    try:
                        self.aiui_client.close()
                    except Exception:
                        pass
                    self.aiui_client = None
                    self._ready_at = 0.0
                    self._boost_until = 0.0

                self._ensure_connected()
                last_activity_time = time.time()  # 【关键】连接成功后立即更新

                if time.time() < self._ready_at:
                    time.sleep(0.01)
                    continue

                self.aiui_client.tts_stop()
                settle = COLD_STOP_SETTLE_S if time.time() < self._startup_until else STOP_SETTLE_S
                time.sleep(settle)
                self.aiui_client.tts_start(text)

                last_seen_id = tid
                last_activity_time = time.time()  # 【关键】发送成功也更新
                self._send_count += 1
                self.get_logger().info(f"[SEND] {text} (send_count={self._send_count})")

            except Exception as e:
                self.get_logger().warn(f"TTS error: {e} -> reconnect")
                try:
                    if self.aiui_client:
                        self.aiui_client.close()
                except Exception:
                    pass
                self.aiui_client = None
                self._ready_at = 0.0
                self._boost_until = 0.0
                last_activity_time = 0  # 【关键】出错重置，下次强制重建
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