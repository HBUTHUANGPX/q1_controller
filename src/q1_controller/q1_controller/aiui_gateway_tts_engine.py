import time, json, struct
from threading import Thread, Event, Lock
from socket import socket, AF_INET, SOCK_STREAM
from aiui_gateway_config import (AIUI_PORT)
from utils_log import log
#import logging

# ====== 直接沿用tts_say的参数 ======
SOCKET_TIMEOUT_S = 5
RECONNECT_DELAY_S = 1.0

STOP_SETTLE_S = 0.15
COLD_START_WINDOW_S = 12.0
COLD_STOP_SETTLE_S = 0.28
COLD_START_BOOST_DELAY_S = 0.10

AUTO_WARMUP_ON_START = True
READY_DELAY_S = 2.0

WARMUP_TEXT = "_"  #若要听见可以改成“1”
WARMUP_REPEAT = 4
WARMUP_GAP_S = 0.6
WARMUP_STOP_BETWEEN = True
WARMUP_STOP_GAP_S = 0.08

ENABLE_BOOST = True
BOOST_WINDOW_S = 8.0
BOOST_DELAY_S = 0.06


class AIUIClient:
    def __init__(self, server_ip, server_port=AIUI_PORT):
        #self.logger = logging.getLogger("AIUIClient")
        #self.logger.setLevel(logging.INFO)

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
 
                self.client_socket = socket(AF_INET, SOCK_STREAM)
                self.client_socket.settimeout(SOCKET_TIMEOUT_S)
                self.client_socket.connect(self.server_ip_port)
                log(f"成功连接到 {self.server_ip_port}")

                # 初始化配置
                self.send_control_message({"type": "voice", "content": {"enable_voice": True}})
                self.send_control_message({"type": "voice", "content": {"vol_value": 15}})
                
                self.send_control_message({
                    "type": "aiui_msg",
                    "content": {"msg_type": 8, "arg1": 0, "arg2": 0, "params": "", "data": ""}
                })
                
                return
            except Exception as e:
                log(f"[AIUI] 连接失败: {e}. {RECONNECT_DELAY_S}秒后重试...")
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
                log("Socket closed")
            except Exception:
                pass




class StableTTSEngine:
    """
    纯 TTS 发送引擎：
    - 维持唯一 TCP 连接
    - warm-up + ready 门禁
    - stop->settle->start
    - 只播最新触发（latest_id）
    """
    def __init__(self, ip: str, port: int):
        self.ip = ip
        self.port = port
        self.aiui_client = None

        self.shutdown_event = Event()
        self._conn_lock = Lock()
        self._lock = Lock()
        self._latest_text = ""
        self._latest_id = 0

        self._ready_at = 0.0
        self._boost_until = 0.0
        self._startup_until = time.time() + COLD_START_WINDOW_S

        # worker thread
        self.worker = Thread(target=self._worker_loop, daemon=True)
        self.worker.start()

        if AUTO_WARMUP_ON_START:
            Thread(target=self._auto_init_aiui, daemon=True).start()

    def push(self, text: str):
        text = (text or "").strip()
        if not text:
            return
        with self._lock:
            self._latest_text = text
            self._latest_id += 1

    def _auto_init_aiui(self):
        try:
            self._ensure_connected()
        except Exception:
            pass

    def _ensure_connected(self):
        with self._conn_lock:
            if self.aiui_client is not None:
                return
            self.aiui_client = AIUIClient(self.ip, self.port)

            # warm-up
            self._ready_at = time.time() + READY_DELAY_S
            try:
                for i in range(WARMUP_REPEAT):
                    self.aiui_client.tts_start(WARMUP_TEXT)
                    log(f"[WARMUP] start {i+1}/{WARMUP_REPEAT} sent")
                    time.sleep(WARMUP_GAP_S)
                    if WARMUP_STOP_BETWEEN and i < WARMUP_REPEAT - 1:
                        self.aiui_client.tts_stop()
                        log(f"[WARMUP] stop {i+1}/{WARMUP_REPEAT} sent")
                        time.sleep(WARMUP_STOP_GAP_S)
                log("TTS warm-up done.")
            except Exception as e:
                log(f"TTS warm-up failed: {e}")
                pass

            self._boost_until = time.time() + BOOST_WINDOW_S if ENABLE_BOOST else 0.0

    def _worker_loop(self):
        last_seen_id = 0
        while not self.shutdown_event.is_set():
            with self._lock:
                text = self._latest_text
                tid = self._latest_id
            if not text or tid == last_seen_id:
                time.sleep(0.01)
                continue
            try:
                self._ensure_connected()
                if time.time() < self._ready_at:
                    time.sleep(0.01)
                    continue

                self.aiui_client.tts_stop()
                settle = COLD_STOP_SETTLE_S if time.time() < self._startup_until else STOP_SETTLE_S
                time.sleep(settle)
                self.aiui_client.tts_start(text)

                last_seen_id = tid
                log(f"[SEND] {text}")
            except Exception as e:
                log(f"TTS error: {e} -> reconnect")
                try:
                    if self.aiui_client:
                        self.aiui_client.close()
                except Exception:
                    pass
                self.aiui_client = None
                self._ready_at = 0.0
                self._boost_until = 0.0
                time.sleep(0.5)

    def close(self):
        self.shutdown_event.set()
        try:
            if self.aiui_client:
                self.aiui_client.close()
        except Exception:
            pass
