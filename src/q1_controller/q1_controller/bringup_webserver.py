#!/usr/bin/env python3
import threading
from fastapi import FastAPI, Request, HTTPException
from fastapi.responses import HTMLResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # 替换为您的自定义 JoystickMsg 类型
from std_msgs.msg import Float32MultiArray  # 用于摇杆数据
from std_msgs.msg import Int32  # 用于状态话题，但自定义为 Header + int state
from q1_controller.srv import StateTransition  
import uvicorn
from datetime import datetime

class WebServerNode(Node):
    def __init__(self):
        super().__init__('web_server_node')
        self.app = FastAPI(title="局域网数据控制面板")

        # 挂载静态文件和模板（确保 templates 和 static 目录存在）
        self.app.mount("/static", StaticFiles(directory="static"), name="static")
        self.templates = Jinja2Templates(directory="templates")

        # 全局状态（新增 robot_state）
        self.all_messages = []  # 合并消息日志
        self.device_status = {
            "online": False,
            "last_seen": None,
            "info": "",
            "left_joystick": {"x": 0, "y": 0},
            "right_joystick": {"x": 0, "y": 0},
            "robot_state": "init"  # 初始状态
        }

        # ROS 2 发布器
        self.joystick_publisher = self.create_publisher(Float32MultiArray, '/joystick_data', 10)
        self.state_publisher = self.create_publisher(Int32, '/robot_state', 10)  # 假设自定义为 Int32，实际需 Header + int
        
        # ROS 2 服务客户端（自定义服务）
        self.state_transition_client = self.create_client(StateTransition, '/state_transition')  # 替换为自定义 StateTransition.srv
        while not self.state_transition_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待状态切换服务...')

        # 新增：ROS 2 订阅者，接收 C++ 发布的 /robot_state 话题
        self.state_subscriber = self.create_subscription(
            Int32,
            '/robot_state',
            self.state_callback,
            10
        )

        # 定义 FastAPI 路由
        @self.app.get("/api/status")
        async def get_status():
            return {
                "messages": self.all_messages[-20:],
                "status": self.device_status,
                "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            }

        @self.app.post("/api/send_joystick")
        async def send_joystick(request: Request):
            try:
                data = await request.json()
                side = data.get('side')
                x = float(data.get('x'))
                y = float(data.get('y'))

                if side == 'left':
                    self.device_status["left_joystick"] = {"x": x, "y": y}
                elif side == 'right':
                    self.device_status["right_joystick"] = {"x": x, "y": y}

                # 发布到 ROS 2 话题
                left_x = self.device_status["left_joystick"]["x"]
                left_y = self.device_status["left_joystick"]["y"]
                right_x = self.device_status["right_joystick"]["x"]
                right_y = self.device_status["right_joystick"]["y"]
                
                msg = Float32MultiArray()
                msg.data.append(left_x)
                msg.data.append(left_y)
                msg.data.append(right_x)
                msg.data.append(right_y)
                self.joystick_publisher.publish(msg)

                message = f"JOYSTICK_{side.upper()}:{x},{y}"
                self.process_message(message, "->")

                return JSONResponse({"status": "success", "message": message})
            except Exception as e:
                raise HTTPException(status_code=400, detail=str(e))

        @self.app.post("/api/set_state")
        async def set_state(request: Request):
            try:
                data = await request.json()
                new_state = data.get('new_state')

                if new_state not in ['init', 'default', 'rl']:
                    raise ValueError("无效状态")

                current_state = self.device_status["robot_state"]
                if current_state == 'err':
                    raise ValueError("err 状态不可切换")

                # 验证切换逻辑
                allowed_transitions = {
                    'init': ['default'],
                    'default': ['init', 'rl'],
                    'rl': ['init', 'default']
                }
                if new_state not in allowed_transitions.get(current_state, []):
                    raise ValueError(f"从 {current_state} 不可切换到 {new_state}")

                req = StateTransition.Request()  # 替换为自定义请求
                req.target_state = self.state_to_int(new_state)  # 示例：将状态转换为 int
                future = self.state_transition_client.call_async(req)
                rclpy.spin_until_future_complete(self, future)
                response = future.result()
                if response and response.success == 1:  # 示例：假设 success 为 sum == 1
                    self.device_status["robot_state"] = new_state
                    self.process_message(f"STATE_CHANGE:{new_state}", "->")

                    # 发布状态话题（自定义格式：Header + int state）
                    state_msg = Int32()
                    state_msg.data = self.state_to_int(new_state)
                    self.state_publisher.publish(state_msg)
                    self.get_logger().info(f'发布状态: {new_state}')
                else:
                    raise ValueError("状态切换失败")

                return JSONResponse({"status": "success", "new_state": new_state})
            except Exception as e:
                raise HTTPException(status_code=400, detail=str(e))

        @self.app.get("/", response_class=HTMLResponse)
        async def index(request: Request):
            return self.templates.TemplateResponse("index.html", {
                "request": request,
                "messages": self.all_messages[-20:],
                "status": self.device_status
            })

        # 启动 Uvicorn 服务器作为后台线程
        threading.Thread(target=self.run_web_server, daemon=True).start()

    def state_callback(self, msg: Int32):
        state_int = msg.data
        new_state = self.int_to_state(state_int)
        if new_state != self.device_status["robot_state"]:
            self.device_status["robot_state"] = new_state
            self.process_message(f"STATE_UPDATE:{new_state} (from topic)", "<-")
            self.get_logger().info(f'从话题接收到新状态: {new_state}')
    
    def int_to_state(self, state_int: int) -> str:
        mapping = {
            -1: 'err',
            0: 'init',
            1: 'default',
            10: 'rl'
        }
        return mapping.get(state_int, 'unknown')  # 默认返回 'unknown' 以防无效值
    
    def process_message(self, message: str, direction: str = "->"):
        now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        log = f"[{now}] {direction} {message}"
        self.all_messages.append(log)
        if len(self.all_messages) > 100:
            self.all_messages.pop(0)

        self.device_status["last_seen"] = now
        self.get_logger().info(log)

    def run_web_server(self):
        uvicorn.run(self.app, host="0.0.0.0", port=8000, log_level=40)

    def state_to_int(self, state: str) -> int:
        mapping = {'err': -1, 'init': 0, 'default': 1, 'rl': 10}
        return mapping.get(state, 0)
    

def main(args=None):
    rclpy.init(args=args)
    node = WebServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("用户中断，关闭节点。")
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()

if __name__ == "__main__":
    main()