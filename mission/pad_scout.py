#!/usr/bin/env python3
"""
pad_scout.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Sürekli pad tarama ve koordinat paylaşım sistemi.

İyileştirmeler:
- Pad'in tam orta noktasını bulmak için çoklu ölçüm ortalaması
- Kesinleşince aramayı durdur
- Koordinat stabilitesi: son N ölçümün ortalamasını al
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
from collections import deque

# Kamera FOV
CAMERA_FOV = 1.3962634  # radyan = 80°

# Renk HSV aralıkları
HSV_RED_LOWER1 = np.array([0, 120, 70])
HSV_RED_UPPER1 = np.array([10, 255, 255])
HSV_RED_LOWER2 = np.array([170, 120, 70])
HSV_RED_UPPER2 = np.array([180, 255, 255])

HSV_BLUE_LOWER = np.array([100, 120, 70])
HSV_BLUE_UPPER = np.array([130, 255, 255])

MIN_PAD_AREA = 300       # Minimum piksel alanı
CONFIRMED_AREA = 3000    # Bu eşiği geçince koordinat biriktirilmeye başlanır
STABLE_SAMPLES = 5       # Kaç ölçüm ortalaması alınsın
STABILITY_THRESHOLD = 1.0  # Metre — ölçümler bu kadar yakınsa kararlı sayılır


class PadScout(Node):
    """
    Tek drone kamerasını dinler, pad tespit edince
    /swarm/pad_found topic'ine koordinat yayınlar.
    Çoklu ölçüm ortalamasıyla pad merkezi tespit edilir.
    """

    def __init__(self, drone_id: int, uav_namespace: str,
                 get_drone_pos_fn, altitude_fn):
        super().__init__(f'pad_scout_{drone_id}')
        self.drone_id = drone_id
        self.get_pos = get_drone_pos_fn
        self.get_alt = altitude_fn
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.active = True

        # Her renk için koordinat birikimi
        self.measurements = {'KIRMIZI': deque(maxlen=STABLE_SAMPLES),
                             'MAVI': deque(maxlen=STABLE_SAMPLES)}
        self.found_colors = set()
        self.last_pos = (0.0, 0.0, 0.0)
        self.last_pos_time = 0.0
        self.drone_speed = 0.0

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

        topic = f'/{uav_namespace}/down_camera/down_camera/image_raw'
        self.create_subscription(Image, topic, self._image_cb, qos_sub)
        self.pub = self.create_publisher(String, '/swarm/pad_found', qos_pub)
        self.get_logger().info(f'PadScout {drone_id}: {topic} dinleniyor')

    def _image_cb(self, msg):
        if not self.active:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        except Exception:
            return

        # Zaten tüm renkler bulunduysa hiç işleme
        with self.lock:
            if len(self.found_colors) == 2:
                return

        # Drone hızını hesapla — hareket halindeyken ölçüm alma
        import time as _time
        now = _time.time()
        cur_pos = self.get_pos()
        if self.last_pos_time > 0:
            dt = now - self.last_pos_time
            if dt > 0:
                dist = math.sqrt(sum((a-b)**2 for a,b in zip(cur_pos, self.last_pos)))
                self.drone_speed = dist / dt
        self.last_pos = cur_pos
        self.last_pos_time = now

        # Drone 0.5 m/s'den hızlı hareket ediyorsa ölçüm alma
        MAX_SPEED_FOR_MEASURE = 0.5
        if self.drone_speed > MAX_SPEED_FOR_MEASURE:
            return

        h, w = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        red1 = cv2.inRange(hsv, HSV_RED_LOWER1, HSV_RED_UPPER1)
        red2 = cv2.inRange(hsv, HSV_RED_LOWER2, HSV_RED_UPPER2)
        self._check_pad(cv2.bitwise_or(red1, red2), 'KIRMIZI', w, h)
        self._check_pad(cv2.inRange(hsv, HSV_BLUE_LOWER, HSV_BLUE_UPPER), 'MAVI', w, h)

    def _check_pad(self, mask, renk: str, w: int, h: int):
        with self.lock:
            if renk in self.found_colors:
                return

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return

        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        if area < MIN_PAD_AREA:
            return

        M = cv2.moments(largest)
        if M['m00'] == 0:
            return

        cx_px = M['m10'] / M['m00']
        cy_px = M['m01'] / M['m00']

        cx_norm = (cx_px - w / 2) / (w / 2)
        cy_norm = (cy_px - h / 2) / (h / 2)

        drone_x, drone_y, drone_z = self.get_pos()
        altitude = abs(drone_z) if drone_z < 0 else drone_z
        if altitude < 0.5:
            altitude = self.get_alt()

        half_width_m = math.tan(CAMERA_FOV / 2) * altitude

        pad_x = drone_x - cy_norm * half_width_m
        pad_y = drone_y + cx_norm * half_width_m

        # Yeterince büyükse ölçüm biriktir
        if area >= CONFIRMED_AREA:
            with self.lock:
                self.measurements[renk].append((pad_x, pad_y, area))
                meas = list(self.measurements[renk])

            # Yeterli ölçüm birikmişse ortalama al
            if len(meas) >= STABLE_SAMPLES:
                xs = [m[0] for m in meas]
                ys = [m[1] for m in meas]
                areas = [m[2] for m in meas]

                # Kararlılık kontrolü — ölçümler birbirine yakın mı?
                x_std = np.std(xs)
                y_std = np.std(ys)

                if x_std < STABILITY_THRESHOLD and y_std < STABILITY_THRESHOLD:
                    # Ağırlıklı ortalama (büyük alan = daha yakın = daha güvenilir)
                    total_area = sum(areas)
                    avg_x = sum(x * a for x, a in zip(xs, areas)) / total_area
                    avg_y = sum(y * a for y, a in zip(ys, areas)) / total_area
                    avg_area = sum(areas) / len(areas)

                    with self.lock:
                        self.found_colors.add(renk)

                    self.get_logger().info(
                        f'PadScout {self.drone_id}: {renk} pad KESİNLEŞTİ → '
                        f'NED({avg_x:.2f}, {avg_y:.2f}) [std={x_std:.2f},{y_std:.2f}]' 
                    )

                    data = {'renk': renk, 'x': avg_x, 'y': avg_y,
                            'alan': avg_area, 'drone_id': self.drone_id,
                            'kesin': True}
                    msg = String()
                    msg.data = json.dumps(data)
                    self.pub.publish(msg)
                    return

        # Henüz kesinleşmedi — ham koordinatı yayınla (güncelleme için)
        data = {'renk': renk, 'x': pad_x, 'y': pad_y,
                'alan': area, 'drone_id': self.drone_id, 'kesin': False}
        msg = String()
        msg.data = json.dumps(data)
        self.pub.publish(msg)

    def deactivate(self):
        self.active = False

    def activate(self):
        self.active = True
        with self.lock:
            self.found_colors = set()
            for k in self.measurements:
                self.measurements[k].clear()


class PadCoordinator(Node):
    """
    /swarm/pad_found dinler, koordinatları kaydeder.
    Kesinleşmiş koordinat üzerine yazılmaz.
    """

    def __init__(self):
        super().__init__('pad_coordinator')
        self.found_pads = {}
        self.lock = threading.Lock()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.create_subscription(String, '/swarm/pad_found', self._pad_cb, qos)
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
        kesin = data.get('kesin', False)

        if not renk or x is None or y is None:
            return

        with self.lock:
            # Zaten kesinleşmişse güncelleme yapma
            if renk in self.found_pads and self.found_pads[renk].get('kesin', False):
                return

            # Kesin veya daha büyük alan ise güncelle
            if renk not in self.found_pads or                kesin or alan > self.found_pads[renk].get('alan', 0):
                self.found_pads[renk] = {'x': x, 'y': y, 'alan': alan, 'kesin': kesin}
                if kesin:
                    self.get_logger().info(
                        f'PadCoordinator: {renk} pad KESİNLEŞTİ → NED({x:.2f}, {y:.2f})'
                    )

    def get_pad(self, renk: str):
        with self.lock:
            if renk in self.found_pads:
                p = self.found_pads[renk]
                return (p['x'], p['y'])
        return None

    def has_pad(self, renk: str) -> bool:
        with self.lock:
            return renk in self.found_pads

    def is_confirmed(self, renk: str) -> bool:
        with self.lock:
            return renk in self.found_pads and self.found_pads[renk].get('kesin', False)

    def all_pads(self):
        with self.lock:
            return dict(self.found_pads)
