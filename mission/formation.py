#!/usr/bin/env python3
"""
formation.py - Formasyon Hesaplama Modülü
NED koordinat sisteminde çalışır.
Yaw açısına duyarlı rotasyon destekler.

Koordinat sistemi:
  NED: X=Kuzey, Y=Doğu, Z=Aşağı
  yaw=0: Kuzey, yaw=pi/2: Doğu

Formasyon düzeni (yaw=0, kuzey):
  OKBASI: Lider önde (kuzey), takipçiler sağ/sol arkada
  V:      Lider önde, iki drone arka çaprazlarda (V şekli)
  CIZGI:  Hepsi yan yana (doğu-batı hattında)
"""
import math
from typing import List, Tuple


def rotate(dx: float, dy: float, yaw: float) -> Tuple[float, float]:
    """Offset'i yaw açısına göre döndür."""
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)
    return (
        dx * cos_y - dy * sin_y,
        dx * sin_y + dy * cos_y
    )


def get_formation_offsets(
    tip: str,
    n_drones: int,
    mesafe: float,
    yaw: float = 0.0
) -> List[Tuple[float, float, float]]:
    """
    Formasyon offsetlerini hesapla (lider dahil).
    yaw: radyan — sürünün hareket yönü (NED kuzeyden saat yönüne)
    Lider index 0, offset (0,0,0)
    Tüm offsetler yaw yönüne göre döndürülür.
    """

    if tip == "OKBASI":
        # Referans (yaw=0, kuzey):
        # Lider: (0, 0)
        # Drone 2: sağ arka  → (-mesafe*sin45, +mesafe*cos45) ≈ (-0.707m, +0.707m) * mesafe
        # Drone 3: sol arka  → (-mesafe*sin45, -mesafe*cos45) ≈ (-0.707m, -0.707m) * mesafe
        raw = [(0.0, 0.0)]
        diag = mesafe * math.sin(math.pi / 4)  # = mesafe * 0.707
        raw.append((-diag, +diag))   # Sağ arka
        raw.append((-diag, -diag))   # Sol arka

    elif tip == "V":
        # V formasyonu: lider önde ortada, diğerleri arkada iki yana açılır
        # Referans (yaw=0, kuzey):
        # Drone 2: sağa ve arkaya
        # Drone 3: sola ve arkaya
        raw = [(0.0, 0.0)]
        for i in range(1, n_drones):
            side = 1 if i % 2 == 1 else -1   # +1 sağ, -1 sol
            step = (i + 1) // 2
            back = -mesafe * step             # Arkaya (NED X eksi = güney)
            side_dist = mesafe * step         # Yana
            raw.append((back, side * side_dist))

    elif tip == "CIZGI":
        # Yan yana, hareket yönüne dik eksende
        # Referans (yaw=0, kuzey): hepsi aynı X'te, Y'de aralıklı
        raw = [(0.0, 0.0)]
        for i in range(1, n_drones):
            side = 1 if i % 2 == 1 else -1
            step = (i + 1) // 2
            raw.append((0.0, side * mesafe * step))

    else:
        # Bilinmeyen: irtifa farkıyla güvenli
        raw = [(0.0, 0.0, float(i) * 2.0) for i in range(n_drones)]
        return raw[:n_drones]

    # Yaw açısına göre döndür
    offsets = []
    for i, r in enumerate(raw[:n_drones]):
        if len(r) == 2:
            rdx, rdy = r
            rdx_rot, rdy_rot = rotate(rdx, rdy, yaw)
            offsets.append((rdx_rot, rdy_rot, 0.0))
        else:
            offsets.append(r)

    return offsets


def formation_positions(
    leader_x: float,
    leader_y: float,
    leader_z: float,
    tip: str,
    n_drones: int,
    mesafe: float,
    yaw: float = 0.0
) -> List[Tuple[float, float, float]]:
    """
    Lider pozisyonuna göre tüm drone hedef pozisyonlarını hesapla.
    """
    offsets = get_formation_offsets(tip, n_drones, mesafe, yaw)
    return [(leader_x + dx, leader_y + dy, leader_z + dz)
            for dx, dy, dz in offsets]


def compute_yaw_to_target(
    current_x: float, current_y: float,
    target_x: float, target_y: float
) -> float:
    """
    İki NED nokta arasındaki yön açısını hesapla (radyan).
    NED'de: X=Kuzey, Y=Doğu
    atan2(dy, dx) doğru yönü verir.
    """
    dx = target_x - current_x
    dy = target_y - current_y
    return math.atan2(dy, dx)


if __name__ == '__main__':
    print("=== FORMASYON TESTİ ===\n")
    yaw_kuzey = 0.0
    yaw_dogu = math.pi / 2

    for tip in ['OKBASI', 'V', 'CIZGI']:
        print(f"--- {tip} | yaw=0° (Kuzey) ---")
        pos = formation_positions(0, 0, 8, tip, 3, 5.0, yaw_kuzey)
        for i, (x, y, z) in enumerate(pos):
            print(f"  Drone {i+1}: NED({x:.2f}, {y:.2f}, {z:.2f})")
        print()

    print("--- QR1→QR2 yönünde OKBASI ---")
    yaw = compute_yaw_to_target(0, 35, 8.66, 25)
    print(f"Yaw: {math.degrees(yaw):.1f}°")
    pos = formation_positions(8.66, 25, 8, 'OKBASI', 3, 5.0, yaw)
    for i, (x, y, z) in enumerate(pos):
        print(f"  Drone {i+1}: NED({x:.2f}, {y:.2f}, {z:.2f})")
