#!/usr/bin/env bash
# run_mission2.sh — SANCAK Görev 2 başlatıcı
#
# Bu script:
#   1. WebSocket bridge'i başlatır (joystick_web_bridge.py)
#   2. HTTP server'ı başlatır (joystick.html servis için)
#   3. Tarayıcıyı otomatik açar (mümkünse)
#   4. Ctrl+C ile ikisini de kapatır
#
# NOT: SITL + MicroXRCEAgent önceden çalışıyor olmalı.

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# ROS2 ortamı
if [ -z "$ROS_DISTRO" ]; then
    source /opt/ros/humble/setup.bash
fi

# px4_ws varsa source et
if [ -f "$HOME/Desktop/SANCAK/px4_ws/install/setup.bash" ]; then
    source "$HOME/Desktop/SANCAK/px4_ws/install/setup.bash"
fi

# websockets kurulu mu?
if ! python3 -c "import websockets" 2>/dev/null; then
    echo "[HATA] 'websockets' kütüphanesi yok."
    echo "  Kurmak için: pip3 install websockets --break-system-packages"
    exit 1
fi

# Temiz çıkış için PID'leri sakla
BRIDGE_PID=""
HTTP_PID=""

cleanup() {
    echo ""
    echo "[run_mission2] Kapatılıyor..."
    [ -n "$BRIDGE_PID" ] && kill $BRIDGE_PID 2>/dev/null || true
    [ -n "$HTTP_PID" ]   && kill $HTTP_PID   2>/dev/null || true
    wait 2>/dev/null || true
    echo "[run_mission2] Kapandı."
    exit 0
}
trap cleanup SIGINT SIGTERM EXIT

echo "════════════════════════════════════════════════════"
echo "   SANCAK — GÖREV 2 · YARI OTONOM SÜRÜ KONTROL"
echo "════════════════════════════════════════════════════"
echo ""

# 1) WebSocket bridge
echo "[1/3] WebSocket bridge başlatılıyor (port 8765)..."
python3 joystick_web_bridge.py &
BRIDGE_PID=$!
sleep 1.5

# 2) HTTP server
echo "[2/3] HTTP server başlatılıyor (port 8080)..."
python3 -m http.server 8080 --bind 127.0.0.1 >/dev/null 2>&1 &
HTTP_PID=$!
sleep 1

# 3) Tarayıcıyı aç
URL="http://localhost:8080/joystick.html"
echo "[3/3] Tarayıcı açılıyor: $URL"
if command -v xdg-open >/dev/null 2>&1; then
    xdg-open "$URL" >/dev/null 2>&1 &
elif command -v gnome-open >/dev/null 2>&1; then
    gnome-open "$URL" >/dev/null 2>&1 &
else
    echo "  (Tarayıcı otomatik açılamadı — manuel açın: $URL)"
fi

echo ""
echo "════════════════════════════════════════════════════"
echo "  HAZIR — Şimdi AYRI bir terminalde görev düğümünü"
echo "  başlatın:"
echo ""
echo "    cd ~/Desktop/SANCAK/mission2"
echo "    python3 semi_autonomous_mission.py"
echo ""
echo "  Tarayıcıdan joystick ile kumanda edin."
echo "  Bu pencereyi Ctrl+C ile kapatın."
echo "════════════════════════════════════════════════════"

# Sonsuz bekle (çocuk processler çalışırken)
wait
