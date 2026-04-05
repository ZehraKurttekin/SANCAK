import asyncio
from mavsdk import System
from mavsdk.offboard import PositionNedYaw

# Sürüdeki dronların adresleri
DRONES = {
    "drone_1": "udp://:14540",
    "drone_2": "udp://:14541",
    "drone_3": "udp://:14542"
}

async def connect_drone(name, url):
    drone = System()
    await drone.connect(system_address=url)
    print(f"{name} bağlandı!")
    return drone

async def main():
    # Tüm dronlara bağlan
    tasks = [connect_drone(name, url) for name, url in DRONES.items()]
    vehicles = await asyncio.gather(*tasks)
    
    # Buraya senin paylaştığın JSON verilerini işleyecek mantığı kuracağız
    print("Sürü hazır! Kalkış bekleniyor...")
    
    # Örnek: Tümünü kaldır
    for drone in vehicles:
        await drone.action.arm()
        await drone.action.takeoff()

    # QR 2 GÖREVİ SİMÜLASYONU (Ayrılma testi)
    print("QR 2 okundu: Drone 2 ayrılıyor!")
    # Sadece 2. drona (Vehicle 1 index) komut gönder
    await vehicles[1].offboard.set_position_ned(PositionNedYaw(12.0, -3.0, -5.0, 0.0))

if __name__ == "__main__":
    asyncio.run(main())
