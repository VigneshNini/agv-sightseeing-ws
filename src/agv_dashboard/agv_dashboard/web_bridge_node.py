#!/usr/bin/env python3
"""Web Bridge Node - WebSocket server bridging ROS 2 topics to browser dashboard."""
import asyncio
import json
import threading
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from agv_msgs.msg import AGVStatus, ObstacleArray, TourStop
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped


class WebBridgeNode(Node):
    def __init__(self):
        super().__init__('web_bridge_node')
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 9090)
        self.declare_parameter('publish_rate', 10.0)

        self.host = self.get_parameter('host').value
        self.port = self.get_parameter('port').value

        self.clients = set()
        self.state = {
            'battery_percent': 100.0,
            'speed_mps': 0.0,
            'heading_deg': 0.0,
            'mode': 'IDLE',
            'current_stop': '',
            'emergency_stop': False,
            'latitude': 0.0,
            'longitude': 0.0,
            'system_status': 'OK',
            'obstacles': [],
            'pose_x': 0.0,
            'pose_y': 0.0,
            'behavior_state': 'IDLE',
            'safety_level': 'SAFE',
        }
        self.loop = asyncio.new_event_loop()
        self._setup_subscriptions()

        rate = self.get_parameter('publish_rate').value
        self.create_timer(1.0/rate, self.broadcast_state)

        ws_thread = threading.Thread(target=self._run_ws_server, daemon=True)
        ws_thread.start()
        self.get_logger().info(f'Web Bridge started at ws://{self.host}:{self.port}')

    def _setup_subscriptions(self):
        self.create_subscription(AGVStatus, '/agv/status', self.status_cb, 10)
        self.create_subscription(ObstacleArray, '/agv/tracked_obstacles', self.obs_cb, 10)
        self.create_subscription(String, '/agv/behavior_state', self.state_cb, 10)
        self.create_subscription(String, '/agv/safety_level', self.safety_cb, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/agv/pose', self.pose_cb, 10)
        self.cmd_sub = self.create_subscription(
            String, '/agv/dashboard_cmd', self.dashboard_cmd_cb, 10)
        self.estop_pub = self.create_publisher(Bool, '/agv/emergency_stop_cmd', 10)
        self.tour_start_pub = self.create_publisher(Bool, '/agv/start_tour', 10)

    def status_cb(self, msg: AGVStatus):
        self.state['battery_percent'] = round(float(msg.battery_percent), 1)
        self.state['speed_mps'] = round(float(msg.speed_mps), 2)
        self.state['heading_deg'] = round(float(msg.heading_deg), 1)
        self.state['mode'] = msg.mode
        self.state['current_stop'] = msg.current_stop
        self.state['emergency_stop'] = msg.emergency_stop
        self.state['latitude'] = round(float(msg.latitude), 6)
        self.state['longitude'] = round(float(msg.longitude), 6)
        self.state['system_status'] = msg.system_status

    def obs_cb(self, msg: ObstacleArray):
        self.state['obstacles'] = [
            {'x': round(float(o.position.x), 2), 'y': round(float(o.position.y), 2),
             'type': o.type, 'id': o.id}
            for o in msg.obstacles[:20]
        ]

    def state_cb(self, msg: String):
        self.state['behavior_state'] = msg.data

    def safety_cb(self, msg: String):
        self.state['safety_level'] = msg.data

    def pose_cb(self, msg: PoseWithCovarianceStamped):
        self.state['pose_x'] = round(float(msg.pose.pose.position.x), 2)
        self.state['pose_y'] = round(float(msg.pose.pose.position.y), 2)
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y**2+q.z**2))
        self.state['heading_deg'] = round(math.degrees(yaw), 1)

    def dashboard_cmd_cb(self, msg: String):
        try:
            cmd = json.loads(msg.data)
            if cmd.get('type') == 'emergency_stop':
                b = Bool(); b.data = cmd.get('value', True)
                self.estop_pub.publish(b)
            elif cmd.get('type') == 'start_tour':
                b = Bool(); b.data = True
                self.tour_start_pub.publish(b)
            elif cmd.get('type') == 'stop_tour':
                b = Bool(); b.data = False
                self.tour_start_pub.publish(b)
        except Exception as e:
            self.get_logger().warn(f'Dashboard cmd error: {e}')

    def broadcast_state(self):
        if not self.clients:
            return
        msg = json.dumps({'type': 'state', 'data': self.state})
        asyncio.run_coroutine_threadsafe(self._broadcast(msg), self.loop)

    async def _broadcast(self, msg):
        if self.clients:
            dead = set()
            for ws in list(self.clients):
                try:
                    await ws.send(msg)
                except Exception:
                    dead.add(ws)
            self.clients -= dead

    async def _handler(self, websocket):
        self.clients.add(websocket)
        self.get_logger().info(f'Dashboard client connected: {websocket.remote_address}')
        try:
            await websocket.send(json.dumps({'type': 'state', 'data': self.state}))
            async for message in websocket:
                try:
                    cmd = json.loads(message)
                    cmd_msg = String()
                    cmd_msg.data = message
                    self.get_logger().info(f'Dashboard cmd: {cmd.get("type", "unknown")}')
                except Exception:
                    pass
        except Exception:
            pass
        finally:
            self.clients.discard(websocket)
            self.get_logger().info('Dashboard client disconnected')

    def _run_ws_server(self):
        asyncio.set_event_loop(self.loop)
        async def serve():
            try:
                import websockets
                async with websockets.serve(self._handler, self.host, self.port):
                    self.get_logger().info(f'WebSocket server running on {self.host}:{self.port}')
                    await asyncio.Future()
            except ImportError:
                self.get_logger().warn('websockets library not installed. Install with: pip install websockets')
            except Exception as e:
                self.get_logger().error(f'WebSocket server error: {e}')
        self.loop.run_until_complete(serve())


def main(args=None):
    rclpy.init(args=args)
    node = WebBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
