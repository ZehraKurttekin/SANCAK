#!/usr/bin/env python3
"""
position_monitor.py
Ayrı terminalde çalıştır — tüm drone konumlarını gösterir.
Drone'lar arası mesafeyi de hesaplar, çarpışma uyarısı verir.

Kullanım:
  source /opt/ros/humble/setup.bash
  source ~/Desktop/SANCAK/px4_ws/install/setup.bash
  python3 position_monitor.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleLocalPosition
import math
import time
import os

DRONE_COUNT   = 3
MIN_SAFE_DIST = 3.0   # metre — bu altına inerse uyarı

class PositionMonitor(Node):
    def __init__(self):
        super().__init__('position_monitor')
        self.positions = {}

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        for i in range(1, DRONE_COUNT + 1):
            ns = f'/px4_{i}'
            self.create_subscription(
                VehicleLocalPosition,
                f'{ns}/fmu/out/vehicle_local_position_v1',
                lambda msg, drone_id=i: self._pos_cb(msg, drone_id),
                qos
            )

        # 2 Hz yazdır
        self.create_timer(0.5, self._print_positions)

    def _pos_cb(self, msg, drone_id):
        self.positions[drone_id] = (msg.x, msg.y, -msg.z)

    def _print_positions(self):
        os.system('clear')
        print("=" * 60)
        print(f"  DRONE KONUMLARI  [{time.strftime('%H:%M:%S')}]")
        print("=" * 60)

        for i in range(1, DRONE_COUNT + 1):
            if i in self.positions:
                x, y, z = self.positions[i]
                tag = " ← LİDER" if i == 1 else ""
                print(f"  Drone {i}{tag}: NED({x:6.2f}, {y:6.2f}) | İrtifa: {z:.2f}m")
            else:
                print(f"  Drone {i}: veri yok")

        print()
        print("  DRONE ARASI MESAFELER:")
        print("  " + "-" * 40)

        ids = list(self.positions.keys())
        has_collision = False

        for i in range(len(ids)):
            for j in range(i + 1, len(ids)):
                id1, id2 = ids[i], ids[j]
                x1, y1, z1 = self.positions[id1]
                x2, y2, z2 = self.positions[id2]
                dist = math.sqrt((x1-x2)**2 + (y1-y2)**2 + (z1-z2)**2)
                dist_xy = math.sqrt((x1-x2)**2 + (y1-y2)**2)

                warn = ""
                if dist < MIN_SAFE_DIST:
                    warn = " ⚠️  ÇARPIŞMA TEHLİKESİ!"
                    has_collision = True
                elif dist < MIN_SAFE_DIST * 1.5:
                    warn = " ⚡ Yakın!"

                print(f"  D{id1}↔D{id2}: 3D={dist:.2f}m  XY={dist_xy:.2f}m{warn}")

        if has_collision:
            print()
            print("  🚨 UYARI: DRONE'LAR ÇARPIŞMA MESAFESİNDE!")

        print("=" * 60)


def main():
    rclpy.init()
    node = PositionMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
