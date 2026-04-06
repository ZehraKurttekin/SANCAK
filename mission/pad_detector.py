#!/usr/bin/env python3
"""
pad_detector.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Drone kamerasından mavi/kırmızı pad tespiti.
HSV renk uzayında renk tespiti yapar.
Pad koordinatı yarışmada VERİLMEYECEK — drone kendi tespit eder.

Çalışma prensibi:
  1. Aşağı bakan kamera görüntüsünü al
  2. HSV'ye çevir
  3. Mavi veya kırmızı maske uygula
  4. Merkez koordinatını piksel cinsinden hesapla
  5. Görüntü merkezinden sapma → yön komutu üret
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import math


# HSV renk aralıkları (Gazebo'daki modeller için ayarlanmıştır)
HSV_BLUE_LOWER = np.array([100, 100, 50])
HSV_BLUE_UPPER = np.array([130, 255, 255])

HSV_RED_LOWER1 = np.array([0, 100, 50])
HSV_RED_UPPER1 = np.array([10, 255, 255])
HSV_RED_LOWER2 = np.array([170, 100, 50])
HSV_RED_UPPER2 = np.array([180, 255, 255])

# Minimum pad alanı (piksel cinsinden) — küçük gürültüleri yok say
MIN_PAD_AREA = 500


class PadDetector(Node):
    def __init__(self, uav_namespace: str, target_color: str, callback):
        """
        uav_namespace: 'uav2', 'uav3' vb.
        target_color: 'MAVI' veya 'KIRMIZI'
        callback: pad bulunduğunda çağrılır → callback(cx_norm, cy_norm, area)
                  cx_norm, cy_norm: [-1, 1] aralığında normalize koordinat
                  (0,0) = merkez, pozitif x = sağ, pozitif y = aşağı
        """
        super().__init__(f'pad_detector_{uav_namespace}')
        self.bridge = CvBridge()
        self.target_color = target_color.upper()
        self.callback = callback
        self.lock = threading.Lock()
        self.pad_found = False
        self.last_cx = 0.0
        self.last_cy = 0.0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        topic = f'/{uav_namespace}/down_camera/down_camera/image_raw'
        self.create_subscription(Image, topic, self._image_cb, qos)
        self.get_logger().info(
            f'PadDetector: {topic} dinleniyor, hedef renk: {target_color}'
        )

    def _image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CV bridge hatası: {e}')
            return

        h, w = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Renk maskesi oluştur
        if self.target_color == 'MAVI':
            mask = cv2.inRange(hsv, HSV_BLUE_LOWER, HSV_BLUE_UPPER)
        elif self.target_color == 'KIRMIZI':
            mask1 = cv2.inRange(hsv, HSV_RED_LOWER1, HSV_RED_UPPER1)
            mask2 = cv2.inRange(hsv, HSV_RED_LOWER2, HSV_RED_UPPER2)
            mask = cv2.bitwise_or(mask1, mask2)
        else:
            return

        # Morfolojik işlem — gürültü temizleme
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Kontur bul
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        if not contours:
            with self.lock:
                self.pad_found = False
            return

        # En büyük konturu al
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)

        if area < MIN_PAD_AREA:
            with self.lock:
                self.pad_found = False
            return

        # Merkez koordinatı
        M = cv2.moments(largest)
        if M['m00'] == 0:
            return

        cx_px = M['m10'] / M['m00']
        cy_px = M['m01'] / M['m00']

        # Normalize et: [-1, 1]
        cx_norm = (cx_px - w / 2) / (w / 2)
        cy_norm = (cy_px - h / 2) / (h / 2)

        with self.lock:
            self.pad_found = True
            self.last_cx = cx_norm
            self.last_cy = cy_norm

        self.get_logger().info(
            f'{self.target_color} pad bulundu! '
            f'Merkez: ({cx_norm:.2f}, {cy_norm:.2f}), Alan: {area:.0f}px²'
        )

        self.callback(cx_norm, cy_norm, area)

    def is_centered(self, threshold=0.15):
        """Pad görüntü merkezinde mi? threshold: normalize birim."""
        with self.lock:
            if not self.pad_found:
                return False
            return (abs(self.last_cx) < threshold and
                    abs(self.last_cy) < threshold)

    def get_offset(self):
        """Pad'in görüntü merkezinden sapması (normalize)."""
        with self.lock:
            return (self.last_cx, self.last_cy, self.pad_found)

    def reset(self):
        with self.lock:
            self.pad_found = False
            self.last_cx = 0.0
            self.last_cy = 0.0
