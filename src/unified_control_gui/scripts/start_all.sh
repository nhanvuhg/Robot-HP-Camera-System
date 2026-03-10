#!/usr/bin/env bash
set -uo pipefail

# ═══════════════════════════════════════════════════════════
# 🚀 START ALL — Cartridge System + GUI
# ═══════════════════════════════════════════════════════════
# Starts:
#   1. cartridge_providesystem_py  — Servo control (Festo CMMT-AS)
#   2. cartridge_gui.py            — HTML GUI (port 8080)
#   3. unified_control_gui         — QML GUI (HDMI)
#
# Usage: bash start_all.sh
# Stop:  Ctrl+C (kills all)
# ═══════════════════════════════════════════════════════════

WS="$HOME/ros2_ws"
export DISPLAY=${DISPLAY:-:0}

# ── Auto-detect XAUTHORITY (cần thiết khi chạy từ SSH) ──
if [ -z "${XAUTHORITY:-}" ]; then
    if [ -f "$HOME/.Xauthority" ]; then
        export XAUTHORITY="$HOME/.Xauthority"
    elif [ -f "/run/user/$(id -u)/gdm/Xauthority" ]; then
        export XAUTHORITY="/run/user/$(id -u)/gdm/Xauthority"
    fi
fi
echo "🖥️  Display: DISPLAY=$DISPLAY  XAUTHORITY=${XAUTHORITY:-<not set>}"

# ── Source ROS 2 ──
set +u
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$WS/fastdds_no_shm.xml"
[ -f /opt/ros/jazzy/setup.bash ] && source /opt/ros/jazzy/setup.bash || echo "⚠️  /opt/ros/jazzy/setup.bash not found"
[ -f "$WS/install/setup.bash" ]  && source "$WS/install/setup.bash"  || echo "⚠️  $WS/install/setup.bash not found — run: colcon build"
set -u

# Kiểm tra package có sẵn không (check binary trực tiếp — tránh AMENT_PREFIX_PATH stale)
if [ ! -f "$WS/install/unified_control_gui/lib/unified_control_gui/unified_control_gui" ]; then
    echo "❌ unified_control_gui binary not found. Chạy: cd ~/ros2_ws && colcon build --packages-select unified_control_gui"
    exit 1
fi

LOG_DIR="$WS/logs"
mkdir -p "$LOG_DIR"

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "🚀 Cartridge System — Full Launcher"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

# ─── Kill old processes bằng PID file (tránh pkill nhầm) ───
PIDFILE="/tmp/cartridge_system.pid"
echo "🔍 Killing old processes..."

if [ -f "$PIDFILE" ]; then
    OLD_PIDS=$(cat "$PIDFILE" 2>/dev/null || true)
    for pid in $OLD_PIDS; do
        if kill -0 "$pid" 2>/dev/null; then
            echo "   ⏹️  Stopping PID=$pid"
            kill -TERM "$pid" 2>/dev/null || true
        fi
    done
    sleep 2
    for pid in $OLD_PIDS; do
        kill -9 "$pid" 2>/dev/null || true
    done
    rm -f "$PIDFILE"
fi

# Fallback pkill nếu còn sót — dùng pattern khớp đúng tên process
pkill -9 -f "cartridge_providesystem_py" 2>/dev/null || true
pkill -9 -f "cartridge_gui.py" 2>/dev/null || true
pkill -9 -f "unified_control_gui/unified_control_gui" 2>/dev/null || true
sleep 1

# Chờ đến khi process cũ kết thúc (tối đa 12s)
_wait=0
while pgrep -f "cartridge_providesystem_py" > /dev/null 2>&1; do
    if [ $_wait -ge 12 ]; then
        echo "⚠️  Process vẫn còn sau 12s — force kill lần 2"
        pkill -9 -f "cartridge_providesystem_py" 2>/dev/null || true
        sleep 2
        break
    fi
    echo "   ⏳ Đợi process cũ kết thúc... ($_wait s)"
    sleep 1
    _wait=$((_wait + 1))
done

# Chờ TCP connections đến servo/IO đóng hết (tối đa 30s)
# CpxAp cycle_time=0.5s nên cần thêm 2-3s để flush connection
_wait=0
while ss -tn state established | grep -qE "192\.168\.27\.(24[89]|25[0-3]):502"; do
    if [ $_wait -ge 30 ]; then
        echo "⚠️  TCP connections vẫn còn sau 30s — tiếp tục"
        break
    fi
    echo "   ⏳ Đợi TCP connections đóng... ($_wait s)"
    sleep 1
    _wait=$((_wait + 1))
done

echo "✅ Clean slate"
echo ""

# ── PIDs ──
PID_PROVIDE=""
PID_WEB_GUI=""
PID_QML_GUI=""

_CLEANUP_DONE=0
cleanup() {
    [ "$_CLEANUP_DONE" -eq 1 ] && return
    _CLEANUP_DONE=1
    echo ""
    echo "🛑 Shutting down..."
    rm -f "$PIDFILE"
    for pid in "${PID_QML_GUI:-}" "${PID_WEB_GUI:-}" "${PID_PROVIDE:-}"; do
        [ -n "$pid" ] && kill "$pid" 2>/dev/null || true
    done
    sleep 1
    for pid in "${PID_QML_GUI:-}" "${PID_WEB_GUI:-}" "${PID_PROVIDE:-}"; do
        [ -n "$pid" ] && kill -9 "$pid" 2>/dev/null || true
    done
    pkill -9 -f "cartridge_gui.py" 2>/dev/null || true
    echo "✅ All processes stopped."
}
trap cleanup EXIT INT TERM

# ── [1/3] Cartridge Provide System Node (servo control) ──
LOG_PROVIDE="$LOG_DIR/cartridge_node.log"
CARTRIDGE_BIN="$WS/install/system_feed_cartridge/lib/system_feed_cartridge/cartridge_providesystem_py"
echo "  [1/3] 🔧 Cartridge Provide System Node..."
"$CARTRIDGE_BIN" > "$LOG_PROVIDE" 2>&1 &
PID_PROVIDE=$!
echo "        PID=$PID_PROVIDE  Log: $LOG_PROVIDE"
# Ghi PID ngay để lần sau kill đúng
echo "$PID_PROVIDE" > "$PIDFILE"
sleep 3

# ── [2/3] Web GUI (cartridge_gui.py — port 8080) — OPTIONAL ──
# Mặc định TẮT để tiết kiệm ~70MB RAM. Dùng: bash start_all.sh --web
LOG_WEB="$LOG_DIR/cartridge_web_gui.log"
WEB_GUI="$WS/src/system_feed_cartridge/scripts/cartridge_gui.py"
WEB_GUI_ENABLED=false
for arg in "$@"; do [ "$arg" = "--web" ] && WEB_GUI_ENABLED=true; done

if $WEB_GUI_ENABLED && [ -f "$WEB_GUI" ]; then
    echo "  [2/3] 🌐 Web GUI (port 8080)..."
    python3 "$WEB_GUI" > "$LOG_WEB" 2>&1 &
    PID_WEB_GUI=$!
    echo "        PID=$PID_WEB_GUI  Log: $LOG_WEB  Access: http://$(hostname -I | awk '{print $1}'):8080"
    echo "$PID_WEB_GUI" >> "$PIDFILE"
    sleep 1
else
    echo "  [2/3] ⏭️  Web GUI skipped (thêm --web để bật)"
fi

# ── [3/3] QML GUI (native, HDMI) ──
LOG_QML="$LOG_DIR/unified_gui.log"
QML_BIN="$WS/install/unified_control_gui/lib/unified_control_gui/unified_control_gui"
if [ -n "${DISPLAY:-}" ]; then
    echo "  [3/3] 🖥️  QML GUI (DISPLAY=$DISPLAY)..."
    "$QML_BIN" > "$LOG_QML" 2>&1 &
    PID_QML_GUI=$!
    echo "        PID=$PID_QML_GUI  Log: $LOG_QML"
    echo "$PID_QML_GUI" >> "$PIDFILE"
else
    echo "  [3/3] ⚠️  DISPLAY not set — skipping QML GUI"
fi

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "✅ All processes started!"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "📋 Logs:"
echo "  tail -f $LOG_PROVIDE"
echo "  tail -f $LOG_WEB"
[ -n "${PID_QML_GUI:-}" ] && echo "  tail -f $LOG_QML"
echo ""
echo "🌐 Web GUI: http://$(hostname -I | awk '{print $1}'):8080"
echo ""
echo "Press Ctrl+C to stop all"
echo ""

# Monitor — exit nếu cartridge node hoặc QML GUI chết
while true; do
    sleep 3
    if ! kill -0 "$PID_PROVIDE" 2>/dev/null; then
        echo "⚠️  Cartridge node exited — stopping all"
        break
    fi
    if [ -n "${PID_QML_GUI:-}" ] && ! kill -0 "$PID_QML_GUI" 2>/dev/null; then
        echo "[GUI] Closed — dừng hệ thống"
        break
    fi
done
