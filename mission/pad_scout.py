#!/usr/bin/env python3
"""
pad_scout.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Sürekli pad tarama ve koordinat paylaşım sistemi.

Şartname gereği:
  - Sürü rota boyunca uçarken tüm kameralar sürekli tarar
  - Herhangi bir drone mavi/kırmızı pad görürse koordinatını kaydeder
  - Koordinat /swarm/pad_found topic'ine yayınlanır
  - Tüm drone'lar bu koordinatı kaydeder
  - Sürüden ayrılma gelince kaydedilmiş koordinata gidilir

Koordinat dönüşümü:
  Kamera piksel sapması → NED metre cinsinden konum tahmini
  drone_pos + (cx * altitude * tan(fov/2), cy * altitude * tan(fov/2))
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import math
import threading


# Kamera FOV (iris.sdf.jinja'dan: 1.3962634 rad = 80°)
CAMERA_FOV = 1.3962634  # radyan

# Renk HSV aralıkları
HSV_RED_LOWER1 = np.array([0, 120, 70])
HSV_RED_UPPER1 = np.array([10, 255, 255])
HSV_RED_LOWER2 = np.array([170, 120, 70])
HSV_RED_UPPER2 = np.array([180, 255, 255])

HSV_BLUE_LOWER = np.array([100, 120, 70])
HSV_BLUE_UPPER = np.array([130, 255, 255])

MIN_PAD_AREA = 300  # piksel cinsinden minimum alan


class PadScout(Node):
    """
    Tek bir drone'un kamerasını dinler, pad tespit edince
    /swarm/pad_found topic'ine koordinat yayınlar.
    """

    def __init__(self, drone_id: int, uav_namespace: str,
                 get_drone_pos_fn, altitude_fn):
        """
        drone_id: 1, 2, 3
        uav_namespace: 'uav2', 'uav3', 'uav4'
        get_drone_pos_fn: () → (x, y, z) NED pozisyonu döndüren fonksiyon
        altitude_fn: () → float mevcut irtifa
        """
        super().__init__(f'pad_scout_{drone_id}')
        self.drone_id = drone_id
        self.get_pos = get_drone_pos_fn
        self.get_alt = altitude_fn
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.active = True

        qos_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        qos_pub = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Kamera topic'ini dinle
        topic = f'/{uav_namespace}/down_camera/down_camera/image_raw'
        self.create_subscription(Image, topic, self._image_cb, qos_sub)

        # Pad bulununca yayınla
        self.pub = self.create_publisher(String, '/swarm/pad_found', qos_pub)

        self.get_logger().info(
            f'PadScout {drone_id}: {topic} dinleniyor'
        )

    def _image_cb(self, msg):
        if not self.active:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        except Exception as e:
            return

        h, w = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Kırmızı pad tespiti
        red_mask1 = cv2.inRange(hsv, HSV_RED_LOWER1, HSV_RED_UPPER1)
        red_mask2 = cv2.inRange(hsv, HSV_RED_LOWER2, HSV_RED_UPPER2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        self._check_pad(red_mask, 'KIRMIZI', w, h)

        # Mavi pad tespiti
        blue_mask = cv2.inRange(hsv, HSV_BLUE_LOWER, HSV_BLUE_UPPER)
        self._check_pad(blue_mask, 'MAVI', w, h)

    def _check_pad(self, mask, renk: str, w: int, h: int):
        """Maskede pad varsa koordinatını hesapla ve yayınla."""
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        if not contours:
            return

        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)

        if area < MIN_PAD_AREA:
            return

        # Merkez piksel
        M = cv2.moments(largest)
        if M['m00'] == 0:
            return

        cx_px = M['m10'] / M['m00']
        cy_px = M['m01'] / M['m00']

        # Normalize piksel sapması [-1, 1]
        cx_norm = (cx_px - w / 2) / (w / 2)
        cy_norm = (cy_px - h / 2) / (h / 2)

        # Drone pozisyonu ve irtifa
        drone_x, drone_y, drone_z = self.get_pos()
        altitude = abs(drone_z) if drone_z < 0 else drone_z
        if altitude < 0.5:
            altitude = self.get_alt()

        # Piksel sapmasından metre cinsinden offset hesapla
        # tan(fov/2) * altitude = görüntü kenarına olan mesafe (metre)
        half_width_m = math.tan(CAMERA_FOV / 2) * altitude

        # NED koordinat sistemi:
        # cx_norm pozitif → sağ → NED Y+
        # cy_norm pozitif → aşağı → NED X- (kamera aşağı bakıyor)
        pad_x = drone_x - cy_norm * half_width_m
        pad_y = drone_y + cx_norm * half_width_m

        self.get_logger().info(
            f'PadScout {self.drone_id}: {renk} pad tespit edildi! '
            f'Alan: {area:.0f}px² → NED({pad_x:.2f}, {pad_y:.2f})'
        )

        # Yayınla
        data = {
            'renk': renk,
            'x': pad_x,
            'y': pad_y,
            'alan': area,
            'drone_id': self.drone_id,
        }
        msg = String()
        msg.data = json.dumps(data)
        self.pub.publish(msg)

    def deactivate(self):
        self.active = False

    def activate(self):
        self.active = True


class PadCoordinator(Node):
    """
    /swarm/pad_found topic'ini dinler, koordinatları kaydeder.
    Tüm drone'lara ortak pad veritabanı sağlar.
    """

    def __init__(self):
        super().__init__('pad_coordinator')
        self.found_pads = {}  # {'KIRMIZI': (x, y), 'MAVI': (x, y)}
        self.lock = threading.Lock()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.create_subscription(
            String, '/swarm/pad_found', self._pad_cb, qos
        )
        self.get_logger().info('PadCoordinator: /swarm/pad_found dinleniyor')

    def _pad_cb(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return

        renk = data.get('renk')
        x = data.get('x')
        y = data.get('y')
        alan = data.get('alan', 0)

        if not renk or x is None or y is None:
            return

        with self.lock:
            # Daha büyük alan görüldüyse güncelle (daha yakın = daha güvenilir)
            if renk not in self.found_pads or \
               alan > self.found_pads[renk].get('alan', 0):
                self.found_pads[renk] = {'x': x, 'y': y, 'alan': alan}
                self.get_logger().info(
                    f'PadCoordinator: {renk} pad kaydedildi → '
                    f'NED({x:.2f}, {y:.2f})'
                )

    def get_pad(self, renk: str):
        """Kaydedilmiş pad koordinatını döndür."""
        with self.lock:
            if renk in self.found_pads:
                p = self.found_pads[renk]
                return (p['x'], p['y'])
        return None

    def has_pad(self, renk: str) -> bool:
        with self.lock:
            return renk in self.found_pads

    def all_pads(self):
        with self.lock:
            return dict(self.found_pads)
