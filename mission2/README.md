# SANCAK — Görev 2 · Yarı Otonom Sürü Kontrol

Web tabanlı sanal joystick ile 3 drone'luk sürüyü senkronize kontrol eder.

## Mimari

```
┌──────────────────────┐
│  Tarayıcı            │  ws://localhost:8765
│  joystick.html       │◄──────────┐
└──────────┬───────────┘           │
           │ WebSocket             │
           ▼                       │
┌──────────────────────┐           │
│ joystick_web_bridge  │ ROS2      │
│  (WS → /swarm/       │───────────┘
│   joystick_cmd)      │
└──────────┬───────────┘
           │ /swarm/joystick_cmd
           ▼
┌──────────────────────────────────┐
│ semi_autonomous_mission.py       │
│  • 3 drone senkron kontrol       │
│  • HAREKET / MANEVRA modu        │
│  • Formasyon: OKBASI / V / CIZGI │
│  • Kalkış / İniş                 │
└──────────────────────────────────┘
```

## Kontrol Haritası

| Giriş | HAREKET Modu | MANEVRA Modu |
|-------|--------------|---------------|
| **Sol joystick Y (throttle)** | İrtifa ±1.5 m/s | İrtifa ±1.5 m/s |
| **Sol joystick X (yaw)** | Sürü dönüşü ±30°/s | Sürü dönüşü ±30°/s |
| **Sağ joystick Y (pitch)** | Sürü ileri/geri ±2 m/s | Formasyon **eğimi** ±25° |
| **Sağ joystick X (roll)** | Sürü sağ/sol ±2 m/s | Formasyon **eğimi** ±25° |
| **Mod butonu** | HAREKET ↔ MANEVRA | — |
| **OKBASI / V / CIZGI** | Formasyon değiştir | Formasyon değiştir |
| **KALKIŞ** | Dikey 8 m yükseliş | — |
| **İNİŞ** | Spawn noktalarına iniş | — |

## Bağımlılıklar

```bash
pip3 install websockets --break-system-packages
```

`nipplejs` (sanal joystick kütüphanesi) HTML içinden CDN üzerinden yüklenir, kurulum gerektirmez.

## Başlatma

**Terminal 1 — MicroXRCEAgent:**
```bash
MicroXRCEAgent udp4 -p 8888
```

**Terminal 2 — PX4 SITL + Gazebo:**
```bash
cd ~/Desktop/SANCAK/PX4-Autopilot
source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export LIBGL_ALWAYS_SOFTWARE=1
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/Desktop/SANCAK/models
Tools/simulation/gazebo-classic/sitl_multiple_run.sh -n 3 -m iris -w sancak_suru
```

**Terminal 3 — Bridge + HTTP server + tarayıcı:**
```bash
cd ~/Desktop/SANCAK/mission2
chmod +x run_mission2.sh
./run_mission2.sh
```

Bu komut otomatik olarak:
- `joystick_web_bridge.py` (WebSocket server, port 8765) başlatır
- `python3 -m http.server 8080` ile HTML servis eder
- Tarayıcıyı `http://localhost:8080/joystick.html` adresinde açar

**Terminal 4 — Yarı otonom görev düğümü:**
```bash
cd ~/Desktop/SANCAK/mission2
source /opt/ros/humble/setup.bash
source ~/Desktop/SANCAK/px4_ws/install/setup.bash
python3 semi_autonomous_mission.py
```

Sonra tarayıcıdan **KALKIŞ** butonuna bas, joystick'le kumanda et.

## Kullanım İpuçları

- **Önce kalkış ver**, sonra joystick girişleri etkilidir
- **Mod değişimi uçuş sırasında** yapılabilir
- **Formasyon değişimi uçuş sırasında** anlık uygulanır
- **Ctrl+C** (herhangi bir terminalde) güvenli iniş tetikler
- İnişten sonra tekrar KALKIŞ verebilirsin

## Teknik Ayrıntılar

### Kontrol Hassasiyeti
`semi_autonomous_mission.py` içinde ayarlanır:
```python
HAREKET_MAX_VEL      = 2.0    # m/s
HAREKET_MAX_V_VEL    = 1.5    # m/s (throttle)
HAREKET_MAX_YAW_RATE = 30°/s
MANEVRA_MAX_ANGLE    = 25°
```

### Güncelleme Frekansı
- Web → bridge: 20 Hz (değişim olduğunda)
- Bridge → ROS2: event-driven (sadece değişim)
- Mission update loop: 10 Hz

### Deadzone
Joystick merkezde olduğunda titremeler yayılmasın diye `|v| < 0.05` komutlar sıfırlanır.

## Sorun Giderme

**"WebSocket koptu" mesajı sürekli çıkıyor:**
- `joystick_web_bridge.py` çalışıyor mu? Port 8765 açık mı?
- `ss -lnt | grep 8765` ile kontrol et

**Dronlar tepki vermiyor:**
- Önce **KALKIŞ** butonuna bastınız mı?
- `semi_autonomous_mission.py` çalışıyor mu?
- `ros2 topic echo /swarm/joystick_cmd` ile komutların geldiğini doğrula

**Tarayıcı açılmadı:**
- Manuel aç: http://localhost:8080/joystick.html

**Dronlar Görev 1'deki gibi başlıyor ama:**
- Evet, `semi_autonomous_mission.py` başlangıç konumunu aynı tutar
- Spawn: Drone 1 (0,0), Drone 2 (0,-4), Drone 3 (0,4)
- Kalkış irtifası: 8m

## Şartname Uyumu

- ✅ Tek bir kumanda girdisi (web joystick)
- ✅ HAREKET modu: sürü formasyonunu koruyarak hareket
- ✅ MANEVRA modu: formasyon pitch/roll ekseninde eğilir
- ✅ Yaw: sürü döner
- ✅ Throttle: sürü irtifa değiştirir
- ✅ Formasyon geçişleri (OKBASI / V / CIZGI)
- ✅ Kalkış / iniş komutları kumandadan
- ✅ Ekranda komut değerleri canlı görünür (hakem takibi için)
