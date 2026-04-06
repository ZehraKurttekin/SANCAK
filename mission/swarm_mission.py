#!/usr/bin/env python3
"""
swarm_mission.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
SANCAK Sürü İHA Görev Koordinatörü — DAĞITIK MİMARİ
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Dağıtık mimari:
  - Lider (px4_1/uav2): QR okur, /swarm/command topic'ine yayınlar
  - Takipçiler (px4_2/uav3, px4_3/uav4): /swarm/command dinler,
    kendi formasyon pozisyonunu bağımsız hesaplar

Koordinat sistemi:
  Gazebo: X=Doğu, Y=Kuzey
  PX4 NED: X=Kuzey, Y=Doğu
  Dönüşüm: NED_x = Gazebo_y, NED_y = Gazebo_x

Pad tespiti:
  Mavi/kırmızı pad koordinatları yarışmada VERİLMEYECEK.
  Drone kendi kamerasıyla tespit eder (pad_detector.py).

Çalıştırmak için:
  source /opt/ros/humble/setup.bash
  source ~/Desktop/SANCAK/px4_ws/install/setup.bash
  cd ~/Desktop/SANCAK/mission
  python3 swarm_mission.py
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

sys.path.insert(0, os.path.dirname(__file__))
from drone_controller import DroneController
from qr_reader import QRReader
from formation import formation_positions
from pad_scout import PadScout, PadCoordinator

# ─── AYARLAR ─────────────────────────────────────────────
DRONE_COUNT = 3
TAKEOFF_ALTITUDE = 8.0      # Metre
CRUISE_ALTITUDE = 8.0       # Metre
QR_SCAN_ALTITUDE = 3.0      # QR okuma irtifası (alçalma)
QR_SCAN_TIMEOUT = 20.0      # Saniye
PAD_SEARCH_ALTITUDE = 5.0   # Pad ararken irtifa
PAD_LAND_TIMEOUT = 30.0     # Saniye

# ─── KOORDİNATLAR (NED) ──────────────────────────────────
# Gazebo koordinatları NED'e çevrilmiştir:
#   NED_x = Gazebo_y,  NED_y = Gazebo_x
# Örnek: QR1 Gazebo(35,0) → NED(0, 35)
QR_POSITIONS = {
    1: (0.0,    35.0),   # Gazebo(35,  0.00) → NED
    2: (8.66,   25.0),   # Gazebo(25,  8.66) → NED
    3: (8.66,   15.0),   # Gazebo(15,  8.66) → NED
    4: (0.0,    10.0),   # Gazebo(10,  0.00) → NED
    5: (-8.66,  15.0),   # Gazebo(15, -8.66) → NED
    6: (-8.66,  25.0),   # Gazebo(25, -8.66) → NED
}
# NOT: Pad koordinatları hardcoded değil — kamera ile tespit edilecek!
# PAD_POSITIONS kaldırıldı.

# UAV namespace (sitl_multiple_run uav2'den başlatıyor)
UAV_NS = {1: 'uav2', 2: 'uav3', 3: 'uav4'}
TEAM_ID = 'team_1'  # QR'daki sonraki_qr key'i
# ─────────────────────────────────────────────────────────


class SwarmCommandPublisher(Node):
    """
    Lider drone'un sürü komutlarını yayınladığı düğüm.
    Takipçiler bu topic'i dinleyerek kendi pozisyonlarını hesaplar.
    """
    def __init__(self):
        super().__init__('swarm_command_publisher')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.pub = self.create_publisher(String, '/swarm/command', qos)

    def publish_command(self, cmd: dict):
        msg = String()
        msg.data = json.dumps(cmd)
        self.pub.publish(msg)


class FollowerNode(Node):
    """
    Takipçi drone'un bağımsız düğümü.
    /swarm/command topic'ini dinler, formasyon pozisyonunu kendi hesaplar.
    """
    def __init__(self, drone_id: int, drone_ctrl: DroneController):
        super().__init__(f'follower_node_{drone_id}')
        self.drone_id = drone_id
        self.ctrl = drone_ctrl
        self.drone_index = drone_id - 1  # 0=lider, 1=takipçi1, 2=takipçi2

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.create_subscription(String, '/swarm/command', self._cmd_cb, qos)
        self.get_logger().info(f'FollowerNode {drone_id}: /swarm/command dinleniyor')

    def _cmd_cb(self, msg):
        try:
            cmd = json.loads(msg.data)
        except Exception:
            return

        cmd_type = cmd.get('type')

        if cmd_type == 'formation':
            lx = cmd['leader_x']
            ly = cmd['leader_y']
            alt = cmd['altitude']
            tip = cmd['formation_type']
            mesafe = cmd['distance']
            n = cmd['n_drones']
            active = cmd.get('active_drones', list(range(1, n + 1)))

            if self.drone_id not in active:
                return  # Bu drone aktif değil

            # Kendi indexini hesapla (aktif listede kaçıncı)
            my_index = active.index(self.drone_id)
            positions = formation_positions(lx, ly, alt, tip, len(active), mesafe)

            if my_index < len(positions):
                x, y, z = positions[my_index]
                self.ctrl.goto_local(x, y, z)
                self.get_logger().info(
                    f'Drone {self.drone_id}: Formasyon hedefi → '
                    f'({x:.1f}, {y:.1f}, {z:.1f}m)'
                )

        elif cmd_type == 'goto':
            if self.drone_id in cmd.get('active_drones', []):
                x = cmd['x']
                y = cmd['y']
                alt = cmd['altitude']
                self.ctrl.goto_local(x, y, alt)

        elif cmd_type == 'land':
            if self.drone_id in cmd.get('target_drones', []):
                self.ctrl.land()

        elif cmd_type == 'altitude':
            if self.drone_id in cmd.get('active_drones', []):
                # Mevcut x,y koru, sadece irtifa değiştir
                x, y, _ = self.ctrl.get_position()
                self.ctrl.goto_local(x, y, cmd['altitude'])


class SwarmMission:
    def __init__(self):
        rclpy.init()

        # Drone kontrolcüleri
        self.drones = {}
        for i in range(1, DRONE_COUNT + 1):
            self.drones[i] = DroneController(i)

        # QR okuyucu — sadece lider
        self.qr_data = None
        self.qr_event = threading.Event()
        self.qr_reader = QRReader(UAV_NS[1], self._on_qr_read)

        # Sürü komut yayıncısı (dağıtık mimari)
        self.cmd_pub = SwarmCommandPublisher()

        # Takipçi düğümleri (dağıtık — kendi formasyon hesabı)
        self.followers = {}
        for i in range(2, DRONE_COUNT + 1):
            self.followers[i] = FollowerNode(i, self.drones[i])

        # Pad coordinator — tüm uçuş boyunca pad koordinatlarını toplar
        self.pad_coordinator = PadCoordinator()

        # Pad scout'lar — her drone'un kamerası sürekli tarar
        self.pad_scouts = {}
        for i in range(1, DRONE_COUNT + 1):
            scout = PadScout(
                drone_id=i,
                uav_namespace=UAV_NS[i],
                get_drone_pos_fn=self.drones[i].get_position,
                altitude_fn=lambda: self.current_altitude
            )
            self.pad_scouts[i] = scout

        # Executor
        self.executor = MultiThreadedExecutor()
        for d in self.drones.values():
            self.executor.add_node(d)
        self.executor.add_node(self.qr_reader)
        self.executor.add_node(self.cmd_pub)
        self.executor.add_node(self.pad_coordinator)
        for f in self.followers.values():
            self.executor.add_node(f)
        for s in self.pad_scouts.values():
            self.executor.add_node(s)

        self.exec_thread = threading.Thread(
            target=self.executor.spin, daemon=True
        )
        self.exec_thread.start()

        # Durum takibi
        self.separated = {}   # drone_id: True/False
        self.current_altitude = TAKEOFF_ALTITUDE

        print("=" * 55)
        print("  SANCAK Sürü İHA Sistemi — Dağıtık Mimari")
        print("=" * 55)

    def _on_qr_read(self, data: dict):
        self.qr_data = data
        self.qr_event.set()

    def _on_pad_found(self, cx, cy, area):
        self.pad_offset = (cx, cy)
        self.pad_event.set()

    def log(self, msg: str):
        print(f"[SANCAK] {msg}")

    def spin(self, seconds: float):
        time.sleep(seconds)

    # ─── EŞ ZAMANLI KALKIŞ ───────────────────────────────
    def takeoff_all(self):
        """
        Tüm drone'ları EŞ ZAMANLI kaldır.
        Adım 1: Hepsinin hedefini ayarla
        Adım 2: Heartbeat yerleşsin (3 sn)
        Adım 3: Hepsini aynı anda arm et
        """
        self.log(f"Eş zamanlı kalkış → {TAKEOFF_ALTITUDE}m")

        # 1. Hepsinin hedefini ayarla (beklemeden)
        for i, drone in self.drones.items():
            drone.prepare_takeoff(TAKEOFF_ALTITUDE)
            self.log(f"  Drone {i}: kalkış hedefi ayarlandı")

        # 2. Heartbeat PX4'e ulaşsın
        self.log("Heartbeat bekleniyor (3s)...")
        self.spin(3.0)

        # 3. Hepsini aynı anda arm et + offboard moda al
        self.log("ARM + OFFBOARD (eş zamanlı)...")
        for i, drone in self.drones.items():
            drone.arm_and_offboard()
            time.sleep(0.1)  # Minimal gecikme (0.5s değil!)

        # 4. Kalkış tamamlanana kadar bekle
        self.log("Kalkış bekleniyor (15s)...")
        self.spin(15.0)
        self.log("Kalkış tamamlandı ✓")

    # ─── FORMASYON KOMUTU (DAĞITIK) ──────────────────────
    def send_formation(self, leader_x, leader_y, altitude,
                       tip, mesafe, active_drones=None):
        """
        Lider /swarm/command'a formasyon komutu yayınlar.
        Takipçiler kendi pozisyonlarını bağımsız hesaplar.
        """
        if active_drones is None:
            active_drones = list(self.drones.keys())

        cmd = {
            'type': 'formation',
            'leader_x': leader_x,
            'leader_y': leader_y,
            'altitude': altitude,
            'formation_type': tip,
            'distance': mesafe,
            'n_drones': DRONE_COUNT,
            'active_drones': active_drones,
        }
        self.cmd_pub.publish_command(cmd)

        # Lider kendi pozisyonunu da ayarla (index 0)
        positions = formation_positions(
            leader_x, leader_y, altitude, tip, len(active_drones), mesafe
        )
        if positions:
            x, y, z = positions[0]
            self.drones[1].goto_local(x, y, z)

        self.log(
            f"Formasyon: {tip} | Mesafe: {mesafe}m | "
            f"Hedef: ({leader_x:.1f}, {leader_y:.1f}, {altitude:.1f}m) | "
            f"Aktif: {active_drones}"
        )

        # Lider hedefe ulaşsın
        self.drones[1].wait_until_reached(threshold=2.0, timeout=30)

    # ─── QR TARAMA ───────────────────────────────────────
    def navigate_to_qr(self, qr_id: int):
        """Sürüyü QR noktasına götür (formasyon koruyarak)."""
        if qr_id not in QR_POSITIONS:
            self.log(f"QR {qr_id} bilinmiyor!")
            return

        qr_x, qr_y = QR_POSITIONS[qr_id]
        active = [i for i in range(1, DRONE_COUNT + 1)
                  if not self.separated.get(i, False)]

        self.log(f"QR #{qr_id} hedefine gidiliyor → NED({qr_x}, {qr_y})")

        # Mevcut formasyonu koru, QR noktasına git
        self.send_formation(
            qr_x, qr_y, self.current_altitude,
            'OKBASI', 5.0, active
        )

    def scan_qr(self, qr_id: int) -> dict:
        """
        Sadece lider drone alçalarak QR okur.
        Takipçiler cruise irtifasında bekler.
        """
        qr_x, qr_y = QR_POSITIONS[qr_id]

        # Lider QR üstüne tam gel
        self.drones[1].goto_local(qr_x, qr_y, self.current_altitude)
        self.drones[1].wait_until_reached(threshold=0.3, timeout=30)

        # Sadece lider alçalır (takipçiler yerinde kalır)
        self.log(f"Lider QR #{qr_id} için alçalıyor → {QR_SCAN_ALTITUDE}m")
        self.drones[1].goto_local(qr_x, qr_y, QR_SCAN_ALTITUDE)
        self.drones[1].wait_until_reached(threshold=0.3, timeout=15)

        self.qr_event.clear()
        self.qr_reader.reset()
        self.log(f"QR #{qr_id} okunuyor... (timeout={QR_SCAN_TIMEOUT}s)")

        if self.qr_event.wait(timeout=QR_SCAN_TIMEOUT):
            self.log(f"QR #{qr_id} başarıyla okundu! ✓")
            # Lider cruise irtifasına dön
            self.drones[1].goto_local(qr_x, qr_y, self.current_altitude)
            self.spin(2.0)
            return self.qr_data
        else:
            self.log(f"UYARI: QR #{qr_id} okunamadı!")
            self.drones[1].goto_local(qr_x, qr_y, self.current_altitude)
            return None

    # ─── GÖREV UYGULAMA ──────────────────────────────────
    def execute_mission(self, qr_data: dict, qr_id: int):
        """
        QR verisinden görevi adım adım uygula.
        Şartname sırası (madde 14):
          1. Formasyon değişimi
          2. Pitch/Roll manevrası
          3. İrtifa değişimi
          4. Bekleme süresi
          5. Sürüden birey ayrılma
        """
        gorev = qr_data.get('gorev', {})
        self.log(f"─── Görev #{qr_id} uygulanıyor ───")

        active = [i for i in range(1, DRONE_COUNT + 1)
                  if not self.separated.get(i, False)]

        # 1. Formasyon değişimi (şartname 1. adım)
        form = gorev.get('formasyon', {})
        if form.get('aktif'):
            tip = form.get('tip', 'OKBASI')
            mesafe = float(form.get('ajanlar_arasi_mesafe_m', 5))
            lx, ly, _ = self.drones[1].get_position()
            self.log(f"Formasyon değişiyor → {tip}, mesafe={mesafe}m")
            self.send_formation(lx, ly, self.current_altitude, tip, mesafe, active)

        # 2. Pitch/Roll manevrası (şartname 2. adım)
        man = gorev.get('manevra_pitch_roll', {})
        if man.get('aktif'):
            pitch = float(man.get('pitch_deg', 0))
            roll = float(man.get('roll_deg', 0))
            self.log(f"Manevra: pitch={pitch}° roll={roll}°")
            self._execute_pitch_roll(pitch, roll, active)

        # 3. İrtifa değişimi (şartname 3. adım)
        irt = gorev.get('irtifa_degisim', {})
        if irt.get('aktif') and irt.get('deger'):
            self.current_altitude = float(irt['deger'])
            self.log(f"İrtifa değişiyor → {self.current_altitude}m")
            cmd = {
                'type': 'altitude',
                'altitude': self.current_altitude,
                'active_drones': active,
            }
            self.cmd_pub.publish_command(cmd)
            x, y, _ = self.drones[1].get_position()
            self.drones[1].goto_local(x, y, self.current_altitude)
            self.spin(5.0)

        # 4. Bekleme süresi (şartname 4. adım)
        bekleme = gorev.get('bekleme_suresi_s', 0)
        if bekleme and float(bekleme) > 0:
            self.log(f"Bekleme: {bekleme}s")
            self.spin(float(bekleme))

        # 5. Sürüden birey ayrılma (şartname 5. adım)
        ayrilma = qr_data.get('suruden_ayrilma', {})
        self.log(f"Ayrılma kontrolü: aktif={ayrilma.get('aktif')}, data={ayrilma}")
        if ayrilma.get('aktif'):
            self._execute_separation(ayrilma, active)

    def _execute_pitch_roll(self, pitch: float, roll: float, active: list):
        """
        Pitch/Roll manevrası — şartname:
        Pitch: sürü merkezi sabit, öndeki alçalır arkadakiler yükselir (veya tersi)
        Roll: sürü merkezi sabit, sağdakiler yükselir soldakiler alçalır (veya tersi)
        """
        if len(active) < 2:
            return

        lx, ly, lz = self.drones[1].get_position()
        mesafe = 5.0  # mevcut formasyon mesafesi tahmini

        if pitch != 0:
            # Pitch: lider alçalır/yükselir, takipçiler ters yönde
            pitch_rad = abs(pitch) * 3.14159 / 180
            delta_z = mesafe * math.sin(pitch_rad)
            self.log(f"Pitch manevrası: ±{delta_z:.1f}m irtifa değişimi")

            if pitch < 0:  # Negatif pitch: lider alçalır
                self.drones[1].goto_local(lx, ly, lz - delta_z)
                for i, did in enumerate(active[1:], 1):
                    x, y, z = self.drones[did].get_position()
                    self.drones[did].goto_local(x, y, z + delta_z * (i / len(active)))
            else:  # Pozitif pitch: lider yükselir
                self.drones[1].goto_local(lx, ly, lz + delta_z)
                for i, did in enumerate(active[1:], 1):
                    x, y, z = self.drones[did].get_position()
                    self.drones[did].goto_local(x, y, z - delta_z * (i / len(active)))

            cmd = {'type': 'formation', 'leader_x': lx, 'leader_y': ly,
                   'altitude': self.current_altitude, 'formation_type': 'OKBASI',
                   'distance': mesafe, 'n_drones': DRONE_COUNT, 'active_drones': active}
            self.cmd_pub.publish_command(cmd)
            self.spin(3.0)

            # Manevra sonrası normal irtifaya dön
            self.spin(2.0)
            self.drones[1].goto_local(lx, ly, self.current_altitude)

    def _execute_separation(self, ayrilma: dict, active: list):
        """
        Sürüden ayrılma görevi.
        Şartname: pad koordinatı önceden kamera ile tespit edilmiş olmalı.
        Koordinat yoksa drone arama yapar.
        """
        drone_key = ayrilma.get('ayrilacak_drone_id', '')
        hedef_renk = ayrilma.get('hedef_renk', '')
        bekleme_s = float(ayrilma.get('bekleme_suresi_s', 5))

        try:
            sep_id = int(drone_key.split('_')[1])
        except Exception:
            self.log(f"Geçersiz drone_id: {drone_key}")
            return

        if sep_id not in self.drones:
            return

        self.log(f"Drone {sep_id} sürüden ayrılıyor → {hedef_renk} pad")
        self.separated[sep_id] = True

        # 1. Önceden tespit edilmiş pad koordinatı var mı?
        pad_coord = self.pad_coordinator.get_pad(hedef_renk)

        if pad_coord:
            pad_x, pad_y = pad_coord
            self.log(
                f"Drone {sep_id}: {hedef_renk} pad koordinatı bulundu → "
                f"NED({pad_x:.2f}, {pad_y:.2f}) — direkt gidiliyor"
            )
            # Pad'e git
            self.drones[sep_id].goto_local(pad_x, pad_y, PAD_SEARCH_ALTITUDE)
            self.drones[sep_id].wait_until_reached(threshold=1.0, timeout=30)
            # Alçal ve in
            self.drones[sep_id].goto_local(pad_x, pad_y, 0.3)
            self.spin(3.0)
            self.drones[sep_id].land()
            self.log(f"Drone {sep_id}: {hedef_renk} pad'e indi! {bekleme_s}s bekleniyor...")
            self.spin(bekleme_s)
        else:
            # Pad koordinatı yok — arama yap
            self.log(
                f"UYARI: {hedef_renk} pad koordinatı henüz kaydedilmemiş! "
                f"Drone {sep_id} arama yapıyor..."
            )
            cur_x, cur_y, _ = self.drones[sep_id].get_position()
            self.drones[sep_id].goto_local(cur_x, cur_y, PAD_SEARCH_ALTITUDE)
            self.spin(3.0)

            # Arama deseni
            search_offsets = [
                (0, 0), (3, 0), (-3, 0), (0, 3), (0, -3),
                (5, 0), (-5, 0), (0, 5), (0, -5),
                (5, 5), (-5, 5), (5, -5), (-5, -5),
            ]
            found = False
            start = time.time()

            for ox, oy in search_offsets:
                if time.time() - start > PAD_LAND_TIMEOUT:
                    break
                self.drones[sep_id].goto_local(
                    cur_x + ox, cur_y + oy, PAD_SEARCH_ALTITUDE
                )
                self.drones[sep_id].wait_until_reached(threshold=0.5, timeout=8)
                self.spin(1.0)

                # Pad tespit edildi mi?
                pad_coord = self.pad_coordinator.get_pad(hedef_renk)
                if pad_coord:
                    pad_x, pad_y = pad_coord
                    self.log(f"Drone {sep_id}: Arama sırasında {hedef_renk} pad bulundu!")
                    self.drones[sep_id].goto_local(pad_x, pad_y, PAD_SEARCH_ALTITUDE)
                    self.drones[sep_id].wait_until_reached(threshold=1.0, timeout=20)
                    self.drones[sep_id].goto_local(pad_x, pad_y, 0.3)
                    self.spin(3.0)
                    self.drones[sep_id].land()
                    self.log(f"Drone {sep_id}: {hedef_renk} pad'e indi! {bekleme_s}s bekleniyor...")
                    self.spin(bekleme_s)
                    found = True
                    break

            if not found:
                self.log(f"UYARI: Drone {sep_id} {hedef_renk} pad'i bulamadı, hover'da bekliyor")
                self.spin(bekleme_s)

        # Sürüye yeniden katıl
        self.log(f"Drone {sep_id} sürüye yeniden katılıyor...")
        lx, ly, _ = self.drones[1].get_position()
        self.drones[sep_id].arm_and_offboard()
        self.spin(2.0)
        self.drones[sep_id].goto_local(lx, ly + 5.0, self.current_altitude)
        self.spin(5.0)
        self.separated[sep_id] = False
        self.log(f"Drone {sep_id}: Sürüye katıldı ✓")

    # ─── ANA GÖREV DÖNGÜSÜ ───────────────────────────────
    def run(self):
        try:
            # 1. Eş zamanlı kalkış
            self.takeoff_all()

            # 2. QR zincirini takip et
            current_qr = 1
            visited = set()

            while current_qr != 0 and current_qr not in visited:
                visited.add(current_qr)
                self.log(f"\n{'═'*45}")
                self.log(f"  Hedef: QR #{current_qr}")
                self.log(f"{'═'*45}")

                # Sürüyü QR'a götür
                self.navigate_to_qr(current_qr)

                # Lider QR'ı tara
                qr_data = self.scan_qr(current_qr)

                if qr_data is None:
                    self.log("QR okunamadı, görev durduruluyor!")
                    break

                # Görevi uygula
                self.execute_mission(qr_data, current_qr)

                # Sonraki QR
                sonraki = qr_data.get('sonraki_qr', {})
                current_qr = sonraki.get(TEAM_ID, 0)
                self.log(
                    f"Sonraki QR: "
                    f"{current_qr if current_qr != 0 else 'YOK (görev bitti)'}"
                )

            # 3. Görev tamamlandı — eve dön
            self.log("\n🏁 Tüm görevler tamamlandı!")
            self.log("Drone'lar başlangıç noktasına dönüyor...")

            active = [i for i in range(1, DRONE_COUNT + 1)
                      if not self.separated.get(i, False)]
            self.send_formation(0.0, 0.0, TAKEOFF_ALTITUDE, 'CIZGI', 3.0, active)
            self.spin(10.0)

            # Hepsini indir
            for i in active:
                self.drones[i].land()
                time.sleep(1.0)

            self.log("Tüm drone'lar indi. Görev başarıyla tamamlandı! ✅")

        except KeyboardInterrupt:
            self.log("\nGörev kullanıcı tarafından durduruldu!")
        finally:
            self.shutdown()

    def shutdown(self):
        for drone in self.drones.values():
            try:
                drone.destroy_node()
            except Exception:
                pass
        try:
            self.qr_reader.destroy_node()
            self.cmd_pub.destroy_node()
        except Exception:
            pass
        for f in self.followers.values():
            try:
                f.destroy_node()
            except Exception:
                pass
        for s in self.pad_scouts.values():
            try:
                s.destroy_node()
            except Exception:
                pass
        try:
            self.pad_coordinator.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()
        self.log("Sistem kapatıldı.")


def main():
    mission = SwarmMission()
    mission.run()


if __name__ == '__main__':
    main()
