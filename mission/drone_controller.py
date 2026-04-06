#!/usr/bin/env python3
"""
drone_controller.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Her drone için bağımsız PX4 kontrol düğümü.
Dağıtık mimari: Her drone kendi ROS2 node'unda çalışır.

Topic adları PX4 v1.14 ile doğrulanmıştır:
  - vehicle_local_position_v1  ✓
  - vehicle_status_v3          ✓ (v2 DEĞİL!)
  - trajectory_setpoint        ✓
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import time
import math

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)


class DroneController(Node):

    def __init__(self, drone_id: int):
        super().__init__(f'drone_controller_{drone_id}')

        self.drone_id = drone_id
        ns = f'/px4_{drone_id}'

        # PX4 v1.14 için doğru QoS
        qos_pub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        qos_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publisher'lar
        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            f'{ns}/fmu/in/offboard_control_mode',
            qos_pub)

        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            f'{ns}/fmu/in/trajectory_setpoint',
            qos_pub)

        self.command_pub = self.create_publisher(
            VehicleCommand,
            f'{ns}/fmu/in/vehicle_command',
            qos_pub)

        # Subscriber'lar — doğru topic adları
        self.local_pos = None
        self.vehicle_status = None

        self.create_subscription(
            VehicleLocalPosition,
            f'{ns}/fmu/out/vehicle_local_position_v1',  # v1 doğru
            self._pos_cb,
            qos_sub)

        self.create_subscription(
            VehicleStatus,
            f'{ns}/fmu/out/vehicle_status_v3',  # v3 doğru! (v2 değil)
            self._status_cb,
            qos_sub)

        # Hedef pozisyon
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = -8.0  # NED: negatif = yukarı
        self.target_yaw = 0.0

        # Offboard heartbeat (10 Hz — PX4 min 2 Hz istiyor)
        self.heartbeat_count = 0
        self.timer = self.create_timer(0.1, self._heartbeat)

        self.get_logger().info(f'DroneController {drone_id} başlatıldı (ns={ns})')

    def _pos_cb(self, msg):
        self.local_pos = msg

    def _status_cb(self, msg):
        self.vehicle_status = msg

    def _heartbeat(self):
        """10 Hz'de offboard control mode + setpoint yayınla."""
        ts = int(self.get_clock().now().nanoseconds / 1000)

        # Offboard control mode
        ocm = OffboardControlMode()
        ocm.position = True
        ocm.velocity = False
        ocm.acceleration = False
        ocm.attitude = False
        ocm.body_rate = False
        ocm.timestamp = ts
        self.offboard_pub.publish(ocm)

        # Trajectory setpoint
        sp = TrajectorySetpoint()
        sp.position = [self.target_x, self.target_y, self.target_z]
        sp.yaw = self.target_yaw
        sp.timestamp = ts
        self.setpoint_pub.publish(sp)

        self.heartbeat_count += 1

    def _send_command(self, command, **kwargs):
        msg = VehicleCommand()
        msg.command = command
        msg.target_system = self.drone_id + 1  # px4_1 → system 2
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        for k, v in kwargs.items():
            setattr(msg, k, float(v))
        self.command_pub.publish(msg)

    def arm(self):
        self._send_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0
        )

    def disarm(self):
        self._send_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=0.0
        )

    def set_offboard_mode(self):
        self._send_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=6.0
        )

    def prepare_takeoff(self, altitude_m=8.0):
        """
        Kalkış öncesi hedefi ayarla — beklemeden.
        Eş zamanlı kalkış için tüm drone'lara çağrılır.
        """
        if self.local_pos:
            self.target_x = self.local_pos.x
            self.target_y = self.local_pos.y
        else:
            self.target_x = 0.0
            self.target_y = float(self.drone_id - 1) * 3.0  # spawn offset
        self.target_z = -abs(altitude_m)
        self.get_logger().info(
            f'Drone {self.drone_id}: Kalkış hedefi ayarlandı → '
            f'({self.target_x:.1f}, {self.target_y:.1f}, {altitude_m:.1f}m)'
        )

    def arm_and_offboard(self):
        """Offboard mod + arm — eş zamanlı kalkış için."""
        self.set_offboard_mode()
        time.sleep(0.3)
        self.arm()
        time.sleep(0.3)
        self.arm()  # İki kez gönder (güvenilirlik için)

    def takeoff(self, altitude_m=8.0):
        """Tek drone kalkışı (eski uyumluluk için)."""
        self.prepare_takeoff(altitude_m)
        time.sleep(3.0)
        self.arm_and_offboard()

    def goto_local(self, x, y, z_m, yaw=0.0):
        """NED koordinatlarında hedefe git."""
        self.target_x = float(x)
        self.target_y = float(y)
        self.target_z = -abs(float(z_m))
        self.target_yaw = float(yaw)

    def land(self):
        self._send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def get_position(self):
        """Mevcut NED pozisyonunu döndür: (x, y, z_metre_yukarı)."""
        if self.local_pos:
            return (
                self.local_pos.x,
                self.local_pos.y,
                -self.local_pos.z  # NED z negatif = yukarı
            )
        return (0.0, float(self.drone_id - 1) * 3.0, 0.0)

    def distance_to_target(self):
        x, y, z = self.get_position()
        target_z_up = -self.target_z  # target_z NED → pozitif yukarı
        return math.sqrt(
            (x - self.target_x) ** 2 +
            (y - self.target_y) ** 2 +
            (z - target_z_up) ** 2
        )

    def wait_until_reached(self, threshold=2.0, timeout=60.0):
        """Hedefe ulaşana kadar bekle."""
        start = time.time()
        while time.time() - start < timeout:
            time.sleep(0.2)
            if self.local_pos and self.distance_to_target() < threshold:
                self.get_logger().info(
                    f'Drone {self.drone_id}: Hedefe ulaşıldı '
                    f'(mesafe={self.distance_to_target():.2f}m)'
                )
                return True
        self.get_logger().warn(
            f'Drone {self.drone_id}: Hedefe ulaşılamadı! (timeout={timeout}s)'
        )
        return False

    def is_armed(self):
        if self.vehicle_status:
            return self.vehicle_status.arming_state == 2  # ARMING_STATE_ARMED
        return False

    def is_in_offboard(self):
        if self.vehicle_status:
            return self.vehicle_status.nav_state == 14  # NAVIGATION_STATE_OFFBOARD
        return False
