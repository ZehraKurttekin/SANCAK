#!/usr/bin/env python3

import math
from typing import List, Tuple


def get_formation_offsets(tip: str, n_drones: int, mesafe: float, yaw: float = 0.0):

    offsets = [(0.0, 0.0, 0.0)]

    if tip == "OKBASI":

        angles = [
            (math.pi + math.pi / 4),
            (math.pi - math.pi / 4),
        ]

        for i in range(min(n_drones - 1, len(angles))):
            angle = angles[i] + yaw
            dx = mesafe * math.cos(angle)
            dy = mesafe * math.sin(angle)
            offsets.append((dx, dy, 0.0))

    elif tip == "V":

        for i in range(1, n_drones):
            side = 1 if i % 2 == 1 else -1
            step = (i + 1) // 2
            dx = -mesafe * step * 0.7
            dy = mesafe * step * side
            offsets.append((dx, dy, 0.0))

    elif tip == "CIZGI":

        for i in range(1, n_drones):
            side = 1 if i % 2 == 1 else -1
            step = (i + 1) // 2
            perp_angle = yaw + math.pi / 2
            dx = mesafe * step * side * math.cos(perp_angle)
            dy = mesafe * step * side * math.sin(perp_angle)
            offsets.append((dx, dy, 0.0))

    else:
        for i in range(1, n_drones):
            offsets.append((0.0, 0.0, float(i)))

    return offsets[:n_drones]


def formation_positions(lx, ly, lz, tip, n, mesafe, yaw=0.0):
    offsets = get_formation_offsets(tip, n, mesafe, yaw)
    return [(lx + dx, ly + dy, lz + dz) for dx, dy, dz in offsets]
