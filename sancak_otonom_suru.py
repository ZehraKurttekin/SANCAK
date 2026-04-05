import asyncio
from mavsdk import System
from mavsdk.offboard import PositionNedYaw, OffboardError
from mavsdk.action import ActionError

async def drone_baslat(d_id, port, off_x, off_y):
    drone = System()
    await drone.connect(system_address=f"udp://:{port}")
    
    print(f"[Drone {d_id}] Bağlantı ve sağlık kontrolü bekleniyor...")
    
    # 1. Sağlık Kontrolü (GPS ve Pozisyonun oturmasını bekle)
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print(f"[Drone {d_id}] Sensörler hazır! Kalkışa geçiliyor...")
            break

    # 2. Güvenli Arm (Reddedilirse tekrar dene)
    armed = False
    while not armed:
        try:
            await drone.action.arm()
            armed = True
            print(f"[Drone {d_id}] Motorlar çalıştırıldı.")
        except ActionError:
            print(f"[Drone {d_id}] Motor çalıştırma reddedildi, tekrar deneniyor...")
            await asyncio.sleep(2)

    # 3. Kalkış
    await drone.action.takeoff()
    await asyncio.sleep(10)

    # 4. Offboard ve Formasyon
    await drone.offboard.set_position_ned(PositionNedYaw(off_x, off_y, -5.0, 0.0))
    try:
        await drone.offboard.start()
        print(f"[Drone {d_id}] Formasyon (V) pozisyonuna geçildi.")
    except OffboardError as error:
        print(f"[Drone {d_id}] Offboard başlatılamadı: {error}")
        return

    while True:
        await drone.offboard.set_position_ned(PositionNedYaw(off_x, off_y, -5.0, 0.0))
        await asyncio.sleep(0.1)

async def main():
    tasks = [
        drone_baslat(1, 14540, 0, 0),    # Lider
        drone_baslat(2, 14541, -4, -4), # Sol Kanat
        drone_baslat(3, 14542, -4, 4)   # Sağ Kanat
    ]
    await asyncio.gather(*tasks)

if __name__ == "__main__":
    asyncio.run(main())
