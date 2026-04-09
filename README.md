# GM-SANCAK — TEKNOFEST 2026 Sürü İHA Yarışması

**Gümüşhane Üniversitesi** | Takım: GM-SANCAK

## Takım

| İsim | Rol |
|------|-----|
| Zehra Kurttekin | Takım Kaptanı — MAVLink/DDS |
| Rafet Tunahan Kayahan | ROS2 Yazılım |
| Mehmet Emin Demir | Görüntü İşleme |
| Hızır Demir | Donanım |
| Ebubekir Cihangir | Gömülü Sistemler |

---

## Proje Hakkında

TEKNOFEST 2026 Sürü İHA Yarışması kapsamında geliştirilen bu proje; 3 adet multikopter İHA'nın tamamen otonom şekilde sürü halinde görev icra etmesini sağlayan yazılım ve algoritmaları içermektedir.

Sistem; QR kod tespiti, dinamik formasyon kontrolü, renk tabanlı pad tespiti ve sürüden birey ekleme/çıkarma görevlerini otonom olarak gerçekleştirmektedir.

---

## Donanım

| Bileşen | Model |
|---------|-------|
| Gövde | DJI F450 × 3 |
| Otopilot | Pixhawk 2.4.8 |
| Bilgisayar | Raspberry Pi 4 |
| Kamera | RPi Camera v2 |
| Firmware | PX4 v1.14 |

---

## Yazılım Mimarisi

**Platform:** Ubuntu 22.04 + PX4 SITL + Gazebo Classic 11 + ROS2 Humble

**Dağıtık mimari:** Her drone bağımsız PX4 + ROS2 düğümü olarak çalışır. Lider drone `/swarm/command` topic'ine yayın yapar, takipçiler kendi formasyon pozisyonlarını bağımsız hesaplar.

```
swarm_mission.py     — Ana görev koordinatörü (lider)
drone_controller.py  — PX4 offboard kontrol arayüzü
formation.py         — OKBASI / V / CIZGI formasyon hesaplama
qr_reader.py         — QR kod okuma (pyzbar + OpenCV)
pad_scout.py         — Mavi/kırmızı pad tespiti ve koordinat paylaşımı
```

**ROS2 Topic'leri:**

| Topic | Açıklama |
|-------|----------|
| `/swarm/command` | Lider → Takipçiler formasyon komutu |
| `/swarm/pad_found` | Pad koordinatı paylaşımı (tüm drone'lar) |
| `/uavX/down_camera/.../image_raw` | Kamera görüntüsü |

---

## Görev Akışı

1. Tek komutla eş zamanlı kalkış → başlangıç irtifasına çıkış
2. Formasyon rotasyonu yaparak QR1'e ilerleme
3. QR kod okuma → görev JSON parse → görevi uygula
4. Görev sırası: **Formasyon → Pitch/Roll → İrtifa → Bekleme → Sürüden ayrılma**
5. Sonraki QR numarasına göre rota → zincir tamamlanana kadar tekrar
6. Son QR'da sonraki numara `0` → eve dön → CIZGI formasyonunda iniş

---

## Simülasyon Kurulumu

### Gereksinimler

```bash
# ROS2 Humble
# PX4 Autopilot v1.14
# Gazebo Classic 11
# Python: opencv-python, pyzbar, qrcode, pillow
pip3 install opencv-python pyzbar qrcode pillow --break-system-packages
```

### Çalıştırma

**Terminal 1 — Micro XRCE-DDS Agent:**
```bash
MicroXRCEAgent udp4 -p 8888
```

**Terminal 2 — Gazebo + PX4 SITL:**
```bash
cd ~/Desktop/SANCAK/PX4-Autopilot
source /usr/share/gazebo/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/Desktop/SANCAK/models
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/Desktop/SANCAK/worlds
Tools/simulation/gazebo-classic/sitl_multiple_run.sh -n 3 -m iris -w sancak_suru
```

**Terminal 3 — Görev başlat:**
```bash
source /opt/ros/humble/setup.bash
source ~/Desktop/SANCAK/px4_ws/install/setup.bash
cd ~/Desktop/SANCAK/mission
python3 swarm_mission.py
```

---

## Koordinat Sistemi

Gazebo → PX4 NED dönüşümü: `NED_x = Gazebo_y`, `NED_y = Gazebo_x`

| QR | NED (x, y) |
|----|------------|
| QR1 | (0.0, 35.0) |
| QR2 | (8.66, 25.0) |
| QR3 | (8.66, 15.0) |
| QR4 | (0.0, 10.0) |
| QR5 | (-8.66, 15.0) |
| QR6 | (-8.66, 25.0) |

---

## Lisans

Bu proje TEKNOFEST 2026 yarışması kapsamında geliştirilmiştir.
