#!/usr/bin/env python3
"""
semi_autonomous_mission.py — SANCAK Görev 2

Hakem web tabanlı sanal joystick'ten komut verir;
/swarm/joystick_cmd topic'inden okunup 3 drone senkronize kontrol edilir.

Kontrol modları:
  HAREKET — sürü merkezi kayar, formasyon korunur
  MANEVRA — merkez sabit, formasyon pitch/roll ekseninde eğilir

Komutlar (JSON):
  {type: "control",   mode, formation, pitch, roll, yaw, throttle}
  {type: "takeoff"}
  {type: "land"}
  {type: "mode_change", mode: "HAREKET"|"MANEVRA"}
  {type: "formation_change", formation: "OKBASI"|"V"|"CIZGI"}

Joystick değerleri [-1, +1] aralığında gelir.
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String

import threading
import time
import math
import json
import sys
import os
import signal

# Görev 1 modüllerini re-use et
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "mission"))
from drone_controller import DroneController
from formation import formation_positions, MIN_SAFE_DIST
from distributed_follower import DistributedFollower, LeaderStatePublisher

# ═══ SABİTLER ══════════════════════════════════════════
DRONE_COUNT = 3

# Başlangıç (Görev 1 ile aynı)
SPAWN_NED = {1: (0.0, 0.0), 2: (0.0, -4.0), 3: (0.0, 4.0)}
SPAWN_CENTER = (0.0, 0.0)
TAKEOFF_ALT = 8.0
FORMATION_DIST = 7.0

# ═══ KONTROL HASSASIYETİ ════════════════════════════════
# Joystick değeri [-1, +1] → fiziksel birime çarpılır.
# Yumuşak başlangıç, hakem rahat takip etsin diye düşük tuttum.

# HAREKET modu: sürü komple hareket eder
HAREKET_MAX_VEL = 2.0  # m/s (pitch/roll yönünde max hız)
HAREKET_MAX_V_VEL = 1.5  # m/s (throttle — dikey)
HAREKET_MAX_YAW_RATE = math.radians(30.0)  # rad/s (30°/s)

# MANEVRA modu: formasyon eğilir
MANEVRA_MAX_ANGLE = math.radians(25.0)  # rad (25° eğim — sunumda net görünür)

# Güncelleme frekansı
UPDATE_HZ = 10.0  # sürü state 10 Hz güncellenir

# Deadzone — joystick merkezde olduğunda minik titremeler yayılmasın
DEADZONE = 0.05

UAV_NS = {1: "uav1", 2: "uav2", 3: "uav3"}


class SemiAutonomousMission(Node):

    def __init__(self):
        super().__init__("semi_autonomous_mission")
        # Drone kontrolcüleri
        self.drones = {}
        for i in range(1, DRONE_COUNT+1):
            try:
                self.drones[i] = DroneController(i)
            except Exception as e:
                print(f"🚨 DroneController {i} oluşturulamadı:", e)
                self.drones[i] = None

        # --- EXECUTOR ÖNCE OLUŞTURULUR ---
        self.swarm_executor = MultiThreadedExecutor()

        # 1. Drone'ları ekle
        for i, d in self.drones.items():
            if d is not None:
                self.swarm_executor.add_node(d)
                print(f"✅ DroneController {i} eklendi")
            else:
                print(f"🚨 DroneController {i} bulunamadı!")

        # 2. Leader
        if self.drones[1] is None:
            raise RuntimeError("DroneController 1 yok — görev başlatılamaz.")
        self.leader_pub = LeaderStatePublisher(self.drones[1].get_position)
        self.swarm_executor.add_node(self.leader_pub)

        # 3. Follower'lar
        self.followers = {}
        for i in range(2, DRONE_COUNT+1):
            if self.drones[i] is not None:
                self.followers[i] = DistributedFollower(i, self.drones[i])
                self.swarm_executor.add_node(self.followers[i])
                print(f"✅ Follower {i} eklendi")
            else:
                print(f"🚨 Follower {i} atlandı — drone yok")

        # 4. Ana node'u ekle ve executor'ı başlat
        self.swarm_executor.add_node(self)
        threading.Thread(target=self.swarm_executor.spin, daemon=True).start()
        
        # 3. Follower'lar

        # --- BURADAN SONRA Subscriber BAŞLAR ---
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(
            String, "/swarm/joystick_cmd", self._on_joystick_msg, qos
        )

        # Feedback publisher (web arayüzüne log döner)
        self.feedback_pub = self.create_publisher(
            String, "/swarm/joystick_feedback", qos
        )

        # Durum
        self.mode = "HAREKET"
        self.formation = "OKBASI"
        self.formation_dist = FORMATION_DIST
        self.current_yaw = 0.0
        self.current_alt = TAKEOFF_ALT

        # Sürü merkez pozisyonu (hareket modunda kayar)
        self.center_x = SPAWN_CENTER[0]
        self.center_y = SPAWN_CENTER[1]

        # Joystick anlık değerleri
        self.j_pitch = 0.0
        self.j_roll = 0.0
        self.j_yaw = 0.0
        self.j_throttle = 0.0

        # Uçuş durumu
        self.is_airborne = False
        self.shutting_down = False
        self._last_cmd_ts = 0.0

        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

        print("=" * 60)
        print("  SANCAK — GÖREV 2 · YARI OTONOM SÜRÜ KONTROL")
        print("  /swarm/joystick_cmd dinleniyor")
        print("  Web arayüz için: python3 -m http.server 8080 (mission2/)")
        print("  Ctrl+C → Güvenli iniş")
        print("=" * 60)

        # Sürekli güncelleme döngüsü
        self.update_timer = self.create_timer(1.0 / UPDATE_HZ, self._update_loop)

    # ─── Ortak yardımcılar ─────────────────────────────
    def _feedback(self, msg):
        m = String()
        m.data = msg
        self.feedback_pub.publish(m)
        self.get_logger().info(msg)

    def _deadzone(self, v):
        return 0.0 if abs(v) < DEADZONE else v

    def _active_drones(self):
        return list(range(1, DRONE_COUNT + 1))

    # ─── Joystick komut callback'i ─────────────────────
    def _on_joystick_msg(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f"Geçersiz JSON: {e}")
            return

        mtype = data.get("type", "")

        if mtype == "control":
            self.j_pitch = self._deadzone(float(data.get("pitch", 0)))
            self.j_roll = self._deadzone(float(data.get("roll", 0)))
            self.j_yaw = self._deadzone(float(data.get("yaw", 0)))
            self.j_throttle = self._deadzone(float(data.get("throttle", 0)))
            new_mode = data.get("mode", self.mode)
            new_form = data.get("formation", self.formation)
            if new_mode != self.mode:
                self.mode = new_mode
            if new_form != self.formation:
                self._change_formation(new_form)
            self._last_cmd_ts = time.time()

        elif mtype == "takeoff":
            threading.Thread(target=self._do_takeoff, daemon=True).start()

        elif mtype == "land":
            threading.Thread(target=self._do_land, daemon=True).start()

        elif mtype == "mode_change":
            self.mode = data.get("mode", self.mode)
            self._feedback(f"Mod → {self.mode}")

        elif mtype == "formation_change":
            self._change_formation(data.get("formation", self.formation))

    # ─── Ana güncelleme döngüsü (10 Hz) ────────────────
    def _update_loop(self):
        if self.shutting_down:
            return
        if not self.is_airborne:
            return

        dt = 1.0 / UPDATE_HZ
        active = self._active_drones()

        if self.mode == "HAREKET":
            self._update_hareket(dt, active)
        else:
            self._update_manevra(dt, active)

    # ─── HAREKET MODU ───────────────────────────────────
    # Sürü merkezi joystick yönünde kayar, formasyon korunur.
    def _update_hareket(self, dt, active):
        # Yaw rate uygula
        self.current_yaw += self.j_yaw * HAREKET_MAX_YAW_RATE * dt
        # Yaw normalize [-pi, pi]
        while self.current_yaw > math.pi:
            self.current_yaw -= 2 * math.pi
        while self.current_yaw < -math.pi:
            self.current_yaw += 2 * math.pi

        # Pitch/Roll → merkez pozisyon hızı (body frame → world frame)
        # Pitch +: ileri (dronun baktığı yön)
        # Roll +: sağa
        vx_body = self.j_pitch * HAREKET_MAX_VEL
        vy_body = self.j_roll * HAREKET_MAX_VEL

        # Body → NED dönüşümü (yaw matrisi)
        cos_y = math.cos(self.current_yaw)
        sin_y = math.sin(self.current_yaw)
        vx_ned = vx_body * cos_y - vy_body * sin_y
        vy_ned = vx_body * sin_y + vy_body * cos_y

        # Merkezi kaydır
        self.center_x += vx_ned * dt
        self.center_y += vy_ned * dt

        # Throttle → irtifa
        self.current_alt += self.j_throttle * HAREKET_MAX_V_VEL * dt
        # 1m ile 30m arası clamp
        self.current_alt = max(1.0, min(30.0, self.current_alt))

        # Formasyon pozisyonlarını hesapla (düz, eğim yok)
        positions = formation_positions(
            self.center_x,
            self.center_y,
            self.current_alt,
            self.formation,
            len(active),
            self.formation_dist,
            self.current_yaw,
        )

        # Her drone'a goto ver — lider ilk, takipçiler sonra
        if positions:
            lx, ly, lz = positions[0]
            self.drones[1].goto_local(lx, ly, lz, yaw=self.current_yaw, max_speed=3.0)
            # Leader state → takipçiler reaktif
            self.leader_pub.update(
                yaw=self.current_yaw,
                formation_type=self.formation,
                formation_dist=self.formation_dist,
                active_drones=active,
            )

    # ─── MANEVRA MODU ───────────────────────────────────
    # Merkez sabit. Formasyon pitch/roll ekseninde eğilir.
    # Yaw ve throttle hâlâ aktiftir.
    def _update_manevra(self, dt, active):
        # Yaw ve throttle hareket modu ile aynı
        self.current_yaw += self.j_yaw * HAREKET_MAX_YAW_RATE * dt
        while self.current_yaw > math.pi:
            self.current_yaw -= 2 * math.pi
        while self.current_yaw < -math.pi:
            self.current_yaw += 2 * math.pi

        self.current_alt += self.j_throttle * HAREKET_MAX_V_VEL * dt
        self.current_alt = max(1.0, min(30.0, self.current_alt))

        # Pitch/Roll açıya dönüşür (merkez sabit)
        pitch_rad = self.j_pitch * MANEVRA_MAX_ANGLE
        roll_rad = self.j_roll * MANEVRA_MAX_ANGLE

        # Önce düz formasyon pozisyonlarını hesapla
        positions = formation_positions(
            self.center_x,
            self.center_y,
            self.current_alt,
            self.formation,
            len(active),
            self.formation_dist,
            self.current_yaw,
        )

        # Her pozisyona pitch/roll uygula
        # Body frame'de: pitch → x ekseninde öne giden dronlar aşağı
        #                roll  → y ekseninde sağa giden dronlar aşağı
        cos_y = math.cos(self.current_yaw)
        sin_y = math.sin(self.current_yaw)

        tilted = []
        for x, y, z in positions:
            # Merkezden offset
            dx = x - self.center_x
            dy = y - self.center_y

            # NED offset'i body frame'e çevir (ters yaw rotasyonu)
            dx_body = dx * cos_y + dy * sin_y
            dy_body = -dx * sin_y + dy * cos_y

            # Pitch: body x'te öndeki alçalır → dz = -dx_body * sin(pitch)
            # Roll:  body y'de sağdaki alçalır → dz = -dy_body * sin(roll)
            dz = -(dx_body * math.sin(pitch_rad) + dy_body * math.sin(roll_rad))

            tilted.append((x, y, z + dz))

        # Lider ilk, takipçiler sonra
        if tilted:
            lx, ly, lz = tilted[0]
            self.drones[1].goto_local(lx, ly, lz, yaw=self.current_yaw, max_speed=3.0)

            # Takipçiler — leader_state ile reaktif olamazlar (eğim özel),
            # doğrudan komut gönderelim
            for idx, did in enumerate(active[1:], 1):
                if idx < len(tilted):
                    fx, fy, fz = tilted[idx]
                    self.drones[did].goto_local(
                        fx, fy, fz, yaw=self.current_yaw, max_speed=3.0
                    )

            # Leader state — formasyon bilgisi güncel kalsın
            self.leader_pub.update(
                yaw=self.current_yaw,
                formation_type=self.formation,
                formation_dist=self.formation_dist,
                active_drones=[1],
            )  # takipçiler manuel yönetiliyor

    # ─── Formasyon değiştirme ──────────────────────────
    def _change_formation(self, new_form):
        if new_form not in ("OKBASI", "V", "CIZGI"):
            self._feedback(f"Bilinmeyen formasyon: {new_form}")
            return
        self.formation = new_form
        self._feedback(f"Formasyon → {new_form}")

    # ─── Kalkış ────────────────────────────────────────
    def _do_takeoff(self):
        if self.is_airborne:
            self._feedback("Zaten havada")
            return

        self._feedback("KALKIŞ başlatılıyor...")

        # Yaw varsayılan — kuzeye bak
        init_yaw = 0.0
        self.current_yaw = init_yaw
        self.current_alt = TAKEOFF_ALT
        self.center_x = SPAWN_CENTER[0]
        self.center_y = SPAWN_CENTER[1]

        # Her drone hazırla
        for i, drone in self.drones.items():
            drone.prepare_takeoff(TAKEOFF_ALT, initial_yaw=init_yaw)
            sx, sy = SPAWN_NED[i]
            drone.target_x = sx
            drone.target_y = sy
            drone.target_yaw = init_yaw

        # Leader state yayına başla (active=[] → takipçi hareket etmez)
        self.leader_pub.update(
            yaw=init_yaw,
            formation_type=self.formation,
            formation_dist=self.formation_dist,
            active_drones=[],
        )

        time.sleep(3.0)

        # Arm + offboard
        for drone in self.drones.values():
            drone.arm_and_offboard()
            time.sleep(0.1)

        # Dikey yükselmeyi bekle
        time.sleep(12.0)
        self._feedback("Dikey kalkış tamam ✓")

        # Formasyon aktif
        self.leader_pub.update(
            yaw=init_yaw,
            formation_type=self.formation,
            formation_dist=self.formation_dist,
            active_drones=self._active_drones(),
        )

        time.sleep(3.0)
        self.is_airborne = True
        self._feedback("Sürü hazır — joystick komutları dinleniyor")

    # ─── İniş ──────────────────────────────────────────
    def _do_land(self):
        if not self.is_airborne:
            self._feedback("Zaten yerde")
            return
        self._feedback("İNİŞ başlatılıyor...")
        self.is_airborne = False

        # Her drone SPAWN noktasına dönsün
        for i in range(1, DRONE_COUNT + 1):
            sx, sy = SPAWN_NED[i]
            self.drones[i].goto_local(
                sx, sy, TAKEOFF_ALT, yaw=self.current_yaw, max_speed=2.0
            )
        time.sleep(8.0)

        # Alçalma
        for i in range(1, DRONE_COUNT + 1):
            sx, sy = SPAWN_NED[i]
            self.drones[i].goto_local(sx, sy, 0.5, yaw=self.current_yaw, max_speed=0.5)
        time.sleep(5.0)

        # Land komutu
        for i in range(1, DRONE_COUNT + 1):
            self.drones[i].land()
            time.sleep(0.1)

        # Disarm bekle
        threads = [
            threading.Thread(
                target=self.drones[i].wait_for_disarm, args=(30.0,), daemon=True
            )
            for i in range(1, DRONE_COUNT + 1)
        ]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        self._feedback("Tüm dronlar indi ✓")

    # ─── Güvenli kapanış ───────────────────────────────
    def _signal_handler(self, sig, frame):
        if self.shutting_down:
            return
        self.shutting_down = True
        print("\n[MISSION2] Ctrl+C — güvenli iniş...")
        for drone in self.drones.values():
            try:
                drone.emergency_land()
            except Exception:
                pass
        time.sleep(2.0)
        try:
            rclpy.shutdown()
        except Exception:
            pass
        sys.exit(0)


def main():
    rclpy.init()
    node = SemiAutonomousMission()
    try:
        # Ana thread'de uyuyalım — spin zaten executor thread'inde
        while rclpy.ok() and not node.shutting_down:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
