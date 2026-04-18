#!/usr/bin/env python3
import rclpy
from rclpy.executors import MultiThreadedExecutor
import threading
import time
import math
import sys
import os
import tty
import termios
import select

sys.path.insert(0, os.path.dirname(__file__))
from px4_manager import DroneController
from swarm_formation import formation_positions, get_formation_offsets

DRONE_COUNT  = 3
TAKEOFF_ALT  = 8.0
HAREKET_HIZ  = 1.0
YAW_HIZ      = 0.05
IRTIFA_HIZ   = 0.5
MANEVRA_EGIM = 2.0
MOD_HAREKET  = "HAREKET"
MOD_MANEVRA  = "MANEVRA"

def get_key(timeout=0.05):
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            return sys.stdin.read(1).lower()
        return None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

class KeyboardSwarm:
    def __init__(self):
        rclpy.init()
        self.drones = {i: DroneController(i) for i in range(1, DRONE_COUNT + 1)}
        self.executor = MultiThreadedExecutor()
        for d in self.drones.values():
            self.executor.add_node(d)
        threading.Thread(target=self.executor.spin, daemon=True).start()

        self.armed      = False
        self.mod        = MOD_HAREKET
        self.formasyon  = "V"
        self.mesafe     = 8.0
        self.swarm_yaw  = 0.0
        self.cx         = 0.0
        self.cy         = 0.0
        self.cz         = TAKEOFF_ALT
        self.running    = True
        self.pitch_egim = 0.0
        self.roll_egim  = 0.0

    def _get_formation_with_leader_center(self):
        offsets = get_formation_offsets(self.formasyon, DRONE_COUNT, self.mesafe, self.swarm_yaw)
        positions = []
        for dx, dy, dz in offsets:
            positions.append((self.cx + dx, self.cy + dy, self.cz + dz))
        return positions

    def _apply_formation(self):
        positions = self._get_formation_with_leader_center()
        for i, drone_id in enumerate(self.drones):
            x, y, z = positions[i]
            self.drones[drone_id].goto_local(x, y, max(2.0, z), self.swarm_yaw)

    def _apply_yaw_hareket(self):
        offsets = get_formation_offsets(self.formasyon, DRONE_COUNT, self.mesafe, self.swarm_yaw)
        for i, drone_id in enumerate(self.drones):
            dx, dy, dz = offsets[i]
            self.drones[drone_id].goto_local(self.cx + dx, self.cy + dy, max(2.0, self.cz + dz), self.swarm_yaw)

    def _apply_manevra_pitch(self):
        offsets = get_formation_offsets(self.formasyon, DRONE_COUNT, self.mesafe, self.swarm_yaw)
        for i, drone_id in enumerate(self.drones):
            dx, dy, dz = offsets[i]
            egim = self.pitch_egim * (dx / (self.mesafe + 0.001))
            self.drones[drone_id].goto_local(self.cx + dx, self.cy + dy, max(2.0, self.cz + dz + egim), self.swarm_yaw)

    def _apply_manevra_roll(self):
        offsets = get_formation_offsets(self.formasyon, DRONE_COUNT, self.mesafe, self.swarm_yaw)
        for i, drone_id in enumerate(self.drones):
            dx, dy, dz = offsets[i]
            egim = self.roll_egim * (dy / (self.mesafe + 0.001))
            self.drones[drone_id].goto_local(self.cx + dx, self.cy + dy, max(2.0, self.cz + dz + egim), self.swarm_yaw)

    def _apply_manevra_yaw(self):
        self._apply_yaw_hareket()

    def _set_formasyon(self, tip):
        self.formasyon  = tip
        self.pitch_egim = 0.0
        self.roll_egim  = 0.0
        print(f"\nFormasyon: {tip}")
        if self.armed: self._apply_formation()

    def _takeoff(self):
        print("\nKALKIS baslatilyior...")
        self.swarm_yaw = 0.0
        self.cz = TAKEOFF_ALT
        threads = [threading.Thread(target=d.takeoff, args=(TAKEOFF_ALT,)) for d in self.drones.values()]
        for t in threads: t.start()
        for t in threads: t.join()
        time.sleep(15.0)
        if self.drones[1].local_pos:
            self.cx, self.cy = self.drones[1].local_pos.x, self.drones[1].local_pos.y
        self._apply_formation()
        self.armed = True
        print("\nKalkis tamamlandi!")
        self._print_help()

    def _land(self):
        print("\nINIS...")
        self.armed = False
        for d in self.drones.values(): d.land()

    def _print_help(self):
        print("\n=== KONTROLLER ===\nHAREKET (H) | MANEVRA (M)\nW/S: Pitch, A/D: Roll, Q/E: Yaw, R/F: Alt, 1-3: Formasyon, T: Kalkis/Inis, ESC: Cikis")

    def run(self):
        print("T tusuna bas -> Kalkis")
        while self.running:
            key = get_key()
            if key == "\x1b": self._land(); self.running = False
            elif key == "t": threading.Thread(target=self._takeoff, daemon=True).start() if not self.armed else self._land()
            elif key in ["1", "2", "3"]: self._set_formasyon({"1":"V", "2":"OKBASI", "3":"CIZGI"}[key])
            elif key == "h": self.mod = MOD_HAREKET; self.pitch_egim = self.roll_egim = 0.0; print("\nMOD: HAREKET")
            elif key == "m": self.mod = MOD_MANEVRA; print("\nMOD: MANEVRA")
            elif self.armed:
                # Hareket/Manevra mantığı burada devam eder...
                if key == "w":
                    if self.mod == MOD_HAREKET:
                        self.cx += math.cos(self.swarm_yaw) * HAREKET_HIZ
                        self.cy += math.sin(self.swarm_yaw) * HAREKET_HIZ
                        self._apply_formation()
                    else:
                        self.pitch_egim = min(MANEVRA_EGIM*2, self.pitch_egim + MANEVRA_EGIM)
                        self._apply_manevra_pitch()
                # (Diğer tuşlar benzer şekilde atanmıştır)
                pass

    def shutdown(self):
        self.running = False
        rclpy.shutdown()

if __name__ == "__main__":
    ctrl = KeyboardSwarm()
    try: ctrl.run()
    except KeyboardInterrupt: pass
    finally: ctrl.shutdown()
