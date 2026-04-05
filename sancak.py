import asyncio
from mavsdk import System
from mavsdk.offboard import PositionNedYaw, OffboardError

async def drone_gorev(d_id, port, off_x, off_y):
    drone = System()
    await drone.connect(system_address=f"udp://:{port}")
    
    print(f"[Drone {d_id}] Bağlantı bekleniyor...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"[Drone {d_id}] Bağlandı!")
            break

    # Sensörlerin (EKF2/GPS) oturmasını bekleme
    print(f"[Drone {d_id}] Sensörler kontrol ediliyor...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print(f"[Drone {d_id}] Hazır!")
            break

    await drone.action.arm()
    await drone.action.takeoff()
    await asyncio.sleep(8)

    # V Formasyonu için Offboard Başlatma
    await drone.offboard.set_position_ned(PositionNedYaw(off_x, off_y, -5.0, 0.0))
    try:
        await drone.offboard.start()
        print(f"[Drone {d_id}] Formasyona geçti.")
    except OffboardError as e:
        print(f"[Drone {d_id}] Hata: {e}")
        return

    while True:
        await drone.offboard.set_position_ned(PositionNedYaw(off_x, off_y, -5.0, 0.0))
        await asyncio.sleep(0.1)

async def main():
    # Lider (0,0), Takipçi 1 (-4, -4), Takipçi 2 (-4, 4)
    tasks = [
        drone_gorev(1, 14540, 0, 0),
        drone_gorev(2, 14541, -4, -4),
        drone_gorev(3, 14542, -4, 4)
    ]
    await asyncio.gather(*tasks)

if __name__ == "__main__":
    asyncio.run(main())
