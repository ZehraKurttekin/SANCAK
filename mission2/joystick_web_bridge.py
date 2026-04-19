#!/usr/bin/env python3
"""
joystick_web_bridge.py — WebSocket → ROS2 köprüsü

HTML arayüzündeki sanal joystick komutlarını ws://localhost:8765 üzerinden
alır, /swarm/joystick_cmd topic'ine JSON string olarak yayınlar.

Kullanım:
    python3 joystick_web_bridge.py

Bağımlılık:
    pip3 install websockets --break-system-packages
"""

import asyncio
import json
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
import websockets

WS_HOST = '0.0.0.0'
WS_PORT = 8765


class BridgeNode(Node):
    def __init__(self):
        super().__init__('joystick_web_bridge')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=5)

        self.cmd_pub = self.create_publisher(
            String, '/swarm/joystick_cmd', qos)
        self.feedback_pub = self.create_publisher(
            String, '/swarm/joystick_feedback', qos)

        # Feedback subscriber — mission'dan gelen mesajları web'e ilet
        self.ws_clients = set()
        self._lock = threading.Lock()

        self.create_subscription(
            String, '/swarm/joystick_feedback',
            self._feedback_cb, qos)

        self.get_logger().info(
            f'Bridge hazır: ws://{WS_HOST}:{WS_PORT} → /swarm/joystick_cmd')

    def publish_cmd(self, payload_dict):
        msg = String()
        msg.data = json.dumps(payload_dict, ensure_ascii=False)
        self.cmd_pub.publish(msg)

    def _feedback_cb(self, msg):
        """Mission'dan gelen log mesajını tüm web istemcilerine ilet."""
        asyncio.run_coroutine_threadsafe(
            self._broadcast_to_clients(msg.data),
            asyncio.get_event_loop()
        ) if asyncio._get_running_loop() else None

    async def _broadcast_to_clients(self, text):
        if not self.ws_clients:
            return
        disconnected = set()
        for ws in list(self.ws_clients):
            try:
                await ws.send(json.dumps({'log': text}))
            except Exception:
                disconnected.add(ws)
        self.ws_clients -= disconnected


# Global bridge node referansı (async handler'lardan erişim için)
bridge_node = None
# Son control mesajını sadece değiştiğinde yayınla (spam önleme)
_last_control_sig = None


async def ws_handler(websocket):
    """Her WebSocket bağlantısı için çalışan handler."""
    global bridge_node, _last_control_sig

    client_addr = websocket.remote_address
    print(f"[BRIDGE] Web istemci bağlandı: {client_addr}")
    bridge_node.ws_clients.add(websocket)

    try:
        async for raw in websocket:
            try:
                msg = json.loads(raw)
            except Exception:
                continue

            mtype = msg.get('type', '')

            # Control mesajları 20 Hz geliyor — sadece değiştiyse yayınla
            if mtype == 'control':
                sig = (
                    msg.get('mode', 'HAREKET'),
                    msg.get('formation', 'OKBASI'),
                    round(msg.get('pitch', 0), 2),
                    round(msg.get('roll', 0), 2),
                    round(msg.get('yaw', 0), 2),
                    round(msg.get('throttle', 0), 2),
                )
                if sig == _last_control_sig:
                    continue
                _last_control_sig = sig

            bridge_node.publish_cmd(msg)

            if mtype != 'control':
                print(f"[BRIDGE] → {mtype}: "
                      f"{json.dumps({k:v for k,v in msg.items() if k != 'ts'})}")

    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        bridge_node.ws_clients.discard(websocket)
        print(f"[BRIDGE] Web istemci koptu: {client_addr}")


async def main_ws():
    async with websockets.serve(ws_handler, WS_HOST, WS_PORT):
        print(f"[BRIDGE] WebSocket server hazır: ws://{WS_HOST}:{WS_PORT}")
        await asyncio.Future()


def ros_spin_thread(node):
    """ROS2 executor ayrı thread'de — async loop ile çakışmasın."""
    rclpy.spin(node)


def main():
    global bridge_node
    rclpy.init()
    bridge_node = BridgeNode()

    t = threading.Thread(target=ros_spin_thread, args=(bridge_node,), daemon=True)
    t.start()

    try:
        asyncio.run(main_ws())
    except KeyboardInterrupt:
        print("\n[BRIDGE] Kapatılıyor...")
    finally:
        bridge_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
