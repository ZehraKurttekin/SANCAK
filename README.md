# GM-SANCAK — TEKNOFEST 2026 Sürü İHA Yarışması

**Gümüşhane Üniversitesi** | Takım: GM-SANCAK | Başvuru ID: 5225815

---

## Takım

| İsim | Rol |
|------|-----|
| Zehra Kurttekin | Takım Kaptanı — MAVLink/DDS, Görev Yönetimi |
| Rafet Tunahan Kayahan | ROS2 Dağıtık Sistem Tasarımı, Çarpışma Önleme |
| Mehmet Emin Demir | Görüntü İşleme — QR Tespiti ve İniş Alanı Analizi |
| Hızır Demir | Donanım Sorumlusu — Aviyonik Entegrasyon |
| Ebubekir Cihangir | Otopilot Konfigürasyonu ve İtki Optimizasyonu |

---

## Proje Hakkında

TEKNOFEST 2026 Sürü İHA Yarışması kapsamında geliştirilen bu proje, 3 adet multikopter İHA'nın tamamen otonom şekilde sürü halinde görev icra etmesini sağlayan yazılım ve algoritmaları içermektedir.

Sistem şu yetenekleri sağlamaktadır:
- Eş zamanlı otonom kalkış ve iniş
- Ok Başı, V ve Çizgi formasyonları arası dinamik geçiş
- QR kod görsel tespiti ve görev çözümleme
- Formasyon rotasyonu ve pitch/roll manevraları
- İrtifa değişimi ve bekleme görevleri
- Renk tabanlı iniş alanı tespiti (görsel servoing)
- Sürüden birey ayrılma/katılma
- Artificial Potential Field (APF) tabanlı çarpışma önleme

---

## Donanım

| Bileşen | Model |
|---------|-------|
| Gövde | DJI F450 × 3 |
| Otopilot | Pixhawk 2.4.8 |
| Bilgisayar | Raspberry Pi 4 |
| Kamera | RPi Camera v2 (down-facing, 60° FOV) |
| Haberleşme | Dual-band USB Wi-Fi (2.4 / 5.8 GHz) |
| Firmware | PX4 v1.14 |

---

## Yazılım Mimarisi

**Platform:** Ubuntu 22.04 LTS + PX4 SITL + Gazebo Classic 11 + ROS2 Humble

### Dağıtık Mimari

Her drone bağımsız PX4 otopilotu + ROS2 düğümü olarak çalışır. Lider drone `/swarm/leader_state` topic'ine 10 Hz pozisyon ve formasyon parametresi yayınlar; takipçiler bu veriyi okuyup kendi offset'lerini bağımsız hesaplar. Merkezi bir kontrol düğümüne bağımlılık yoktur.

### Modüller

```
mission/
├── swarm_mission.py         — Ana görev koordinatörü, QR zincirini yönetir
├── drone_controller.py      — PX4 offboard kontrol arayüzü (her drone için)
├── distributed_follower.py  — Takipçi drone mantığı + APF destekli takip
├── formation.py             — OKBASI / V / CIZGI formasyon geometrisi
├── qr_reader.py             — QR kod okuma (pyzbar + OpenCV)
├── pad_scout.py             — Renk tabanlı pad tespiti, sürekli tarama
├── apf_repulsion.py         — Artificial Potential Field çarpışma önleme
├── set_px4_params.py        — PX4 parametre uyumlama (tuning)
├── position_monitor.py      — Drone konum + mesafe izleme aracı
└── keyboard_control.py      — Yarı otonom kontrol (Görev 2)

models/
├── iris/                    — Aşağı bakan kameralı drone modeli
├── qr_plaka_1..6/           — 120×120 cm QR plaka modelleri
├── blue_pad, red_pad/       — Renkli iniş alanları
└── mavlink/                 — MAVLink mesaj tanımları

worlds/
└── sancak_suru.world        — Yarışma sahası (6 QR + 2 iniş pedi)
```

### ROS2 Topic'leri

| Topic | Tür | Açıklama |
|-------|-----|----------|
| `/swarm/leader_state` | String (JSON) | Lider pozisyonu + formasyon parametreleri, 10 Hz |
| `/swarm/formation_params` | String (JSON) | Formasyon güncellemeleri |
| `/swarm/pad_found` | String (JSON) | Pad tespiti (her drone yayın yapar) |
| `/uavX/down_camera/down_camera/image_raw` | Image | Aşağı kamera görüntüsü |
| `/px4_X/fmu/in/trajectory_setpoint` | TrajectorySetpoint | PX4 hedef pozisyonu |
| `/px4_X/fmu/out/vehicle_local_position_v1` | VehicleLocalPosition | NED pozisyon |

---

## Temel Algoritmalar

### Formasyon Kontrolü (`formation.py`)
Lider NED koordinatı ve yaw'ı giriş olarak alınır, her takipçi için trigonometrik offset hesaplanır. Formasyon hareket yönüne göre otomatik rotasyon uygular (`atan2(Δy, Δx)`). Minimum güvenli mesafe 4 m olarak garanti edilir.

### Çarpışma Önleme (`apf_repulsion.py`)
Artificial Potential Field yöntemi ile her drone'un formasyon hedefine diğer dronlardan gelen itici kuvvet vektörü eklenir:

```
F_rep = K_REP × (1/d − 1/d_safe) / d²    if d < d_safe
F_rep = 0                                 if d ≥ d_safe
```

Parametreler: `SAFE_DIST = 5 m`, `INFLUENCE = 7 m`, `K_REP = 15`, `MAX_PUSH = 3 m`.

### Pad Tespiti ve Görsel Servoing (`pad_scout.py`)
HSV renk filtresi + kontur tespiti ile mavi/kırmızı pad merkezi anlık hesaplanır. Pixel koordinatı kamera FOV'u ve drone irtifası kullanılarak NED koordinatına dönüştürülür. İniş sırasında drone, pad'i gördüğü her an anlık merkeze kilitlenir — eski kayda saplanıp kalmaz. Pad kaybedilirse yükselerek spiral arama devreye girer (6m → 8m → 10m).

### QR Okuma (`qr_reader.py`)
OpenCV + pyzbar ile grayscale görüntüden JSON çözümler. Her QR bir sonraki hedefi ve icra edilecek görevi içerir:

```json
{
  "qr_id": 1,
  "gorev": {
    "formasyon": {"aktif": true, "tip": "OKBASI"},
    "manevra_pitch_roll": {"aktif": false, "pitch_deg": "0", "roll_deg": "0"},
    "irtifa_degisimi": {"aktif": true, "deger": 10},
    "bekleme_suresi_s": 10
  },
  "suruden_ayrilma": {"aktif": false, ...},
  "sonraki_qr": {"team_1": 2, ...}
}
```

---

## Görev Akışı

1. **Kalkış** → Her drone spawn konumunda dikey 8 m yükselir; formasyon kurulmaz
2. **QR1'e gidiş** → Dronlar spawn dizilimini koruyarak paralel hareket eder
3. **QR okuma** → Sadece lider 2.5 m'ye alçalır, takipçiler 8 m'de hover
4. **JSON çözümleme** → Log'a detaylı yazdırılır
5. **Görev icra** → Formasyon + Pitch/Roll + İrtifa + Bekleme + Ayrılma
6. **Sonraki QR** → `sonraki_qr.team_1` değerine göre rota
7. **Son QR** (team_1 = 0) → Her drone kendi spawn noktasına iner

---

## Simülasyon Kurulumu

### Gereksinimler

```bash
# ROS2 Humble, PX4 Autopilot v1.14+, Gazebo Classic 11 gerekli.

# Python bağımlılıkları
pip3 install opencv-python pyzbar qrcode pillow pymavlink --break-system-packages

# MicroXRCEAgent (PX4 <-> ROS2 köprüsü) kurulumu için:
# https://github.com/eProsima/Micro-XRCE-DDS-Agent
```

### Klasör Yerleşimi

```
~/Desktop/SANCAK/
├── mission/                 ← bu repo
├── models/                  ← bu repo
├── worlds/                  ← bu repo
├── PX4-Autopilot/           ← PX4 kaynak kodu (ayrı clone)
└── px4_ws/                  ← ROS2 workspace (px4_msgs içerir)
```

### Çalıştırma

**Terminal 1 — Micro XRCE-DDS Agent:**
```bash
MicroXRCEAgent udp4 -p 8888
```

**Terminal 2 — Gazebo + PX4 SITL:**
```bash
cd ~/Desktop/SANCAK/PX4-Autopilot
source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export LIBGL_ALWAYS_SOFTWARE=1
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/Desktop/SANCAK/models
Tools/simulation/gazebo-classic/sitl_multiple_run.sh -n 3 -m iris -w sancak_suru
```

**Terminal 3 — Görev başlat:**
```bash
source /opt/ros/humble/setup.bash
source ~/Desktop/SANCAK/px4_ws/install/setup.bash
cd ~/Desktop/SANCAK/mission
python3 swarm_mission.py
```

**Terminal 4 (opsiyonel) — Konum izleyici:**
```bash
source /opt/ros/humble/setup.bash
source ~/Desktop/SANCAK/px4_ws/install/setup.bash
cd ~/Desktop/SANCAK/mission
python3 position_monitor.py
```

---

## Koordinat Sistemi

Gazebo ENU (X=doğu, Y=kuzey) ↔ PX4 NED (X=kuzey, Y=doğu) dönüşümü:

```
NED_x = Gazebo_y + NED_X_OFFSET   (NED_X_OFFSET = -3.0, spawn kaymasını telafi eder)
NED_y = Gazebo_x
```

### Yarışma Sahası

Simülasyonda kullanılan QR plaka koordinatları:

| QR | Gazebo (x, y) | NED (x, y) |
|----|---------------|------------|
| QR1 | (25.00, 12.00) | (9.00, 25.00) |
| QR2 | (25.00, 20.66) | (17.66, 25.00) |
| QR3 | (15.00, 3.66) | (0.66, 15.00) |
| QR4 | (10.00, 12.00) | (9.00, 10.00) |
| QR5 | (15.00, 20.66) | (17.66, 15.00) |
| QR6 | (25.00, 3.66) | (0.66, 25.00) |

**Spawn Konumları (NED):**
- Drone 1 (Lider, ortada): (0, 0)
- Drone 2: (0, −4)
- Drone 3: (0, 4)

---

## Simülasyon Konfigürasyonu

### PX4 Tuning Parametreleri

Osilasyonu ve overshoot'u azaltmak için `ROMFS/px4fmu_common/init.d-posix/airframes/10016_gazebo-classic_iris` dosyasına SANCAK blokundaki parametreler eklenmiştir:

| Parametre | Değer | Amaç |
|-----------|-------|------|
| MPC_XY_P | 0.5 | Yumuşak pozisyon kontrolü |
| MPC_XY_VEL_MAX | 5.0 | Max yatay hız (m/s) |
| MPC_JERK_MAX | 8.0 | Max jerk |
| MPC_ACC_HOR | 4.0 | Yatay ivme limiti |
| MPC_Z_VEL_MAX_DN | 3.5 | Max aşağı hız |
| MPC_LAND_SPEED | 0.5 | Son iniş hızı (zıplama önleyici) |

### Hız Parametreleri (`swarm_mission.py`)

```python
CRUISE_SPEED    = 4.5   # Normal seyir
APPROACH_SPEED  = 2.5   # QR'a yaklaşma
FORMATION_SPEED = 3.5   # Formasyon geçişi
LAND_SPEED      = 0.8   # Pad iniş
FORMATION_DIST  = 7.0   # Drone'lar arası mesafe (m)
```

---

## Görev Parametreleri

| Parametre | Değer |
|-----------|-------|
| Kalkış irtifası | 8 m |
| QR okuma irtifası | 2.5 m |
| Pad yaklaşma irtifası | 5 m |
| QR okuma timeout | 20 s |
| Formasyon mesafesi | 7 m |

---

## Failsafe ve Güvenlik

- Her drone bağımsız PX4 failsafe mekanizmasına sahiptir
- YKİ bağlantısı kesilirse son hedef pozisyonunda hover
- `Ctrl+C` mevcut konumda güvenli iniş tetikler
- APF çarpışma önleme 4 m altına düşen yaklaşımlarda aktif

---

## Lisans

Bu proje TEKNOFEST 2026 yarışması kapsamında Gümüşhane Üniversitesi GM-SANCAK takımı tarafından geliştirilmiştir.
