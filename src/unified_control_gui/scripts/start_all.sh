#!/usr/bin/env bash
set -uo pipefail

# ═══════════════════════════════════════════════════════════
# 🚀 START ALL — Full System Launcher
# ═══════════════════════════════════════════════════════════
# Starts:
#   1. cartridge_providesystem_py  — Servo control (Festo CMMT-AS)
#   2. dobot_bringup_v3            — Dobot Nova5 driver
#   3. robot_logic_node            — Robot pick-and-place logic
#   4. gripper_festo_node          — Festo gripper (venv)
#   5. dual_camera_system          — CSI cameras + YOLO
#   6. cartridge_gui.py            — HTML GUI (port 8080, optional)
#   7. unified_control_gui         — QML GUI (HDMI)
#
# Usage: bash start_all.sh [--web]
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
export ROS_DOMAIN_ID=22

[ -f /opt/ros/jazzy/setup.bash ] && source /opt/ros/jazzy/setup.bash || echo "⚠️  /opt/ros/jazzy/setup.bash not found"
[ -f "$WS/install/setup.bash" ]  && source "$WS/install/setup.bash"  || echo "⚠️  $WS/install/setup.bash not found — run: colcon build"
set -u

# Kiểm tra binary có sẵn không
if [ ! -f "$WS/install/unified_control_gui/lib/unified_control_gui/unified_control_gui" ]; then
    echo "❌ unified_control_gui binary not found. Chạy: cd ~/ros2_ws && colcon build --packages-select unified_control_gui"
    exit 1
fi

LOG_DIR="$WS/logs"
mkdir -p "$LOG_DIR"

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "🚀 Full System — Cartridge + Robot Launcher"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

# ─── Check for duplicate processes ───
EXISTING=$(ps aux | grep -E "(cartridge_providesystem_py|robot_logic_node|dobot_bringup|gripper_festo|dual_csi_camera|yolo_ros_hailort|unified_control_gui)" | grep -v -E "(grep|tail|start_all)" | wc -l || true)
if [ "${EXISTING:-0}" -gt 0 ]; then
    echo "⚠️  WARNING: Found $EXISTING existing process(es)! Tự động dọn dẹp..."
    pkill -9 -f "cartridge_providesystem_py" 2>/dev/null || true
    pkill -9 -f "cartridge_gui.py" 2>/dev/null || true
    pkill -9 -f "unified_control_gui/unified_control_gui" 2>/dev/null || true
    pkill -9 -f "robot_logic_node" 2>/dev/null || true
    pkill -9 -f "motion_executor" 2>/dev/null || true
    pkill -9 -f "dobot_bringup" 2>/dev/null || true
    pkill -9 -f "gripper_festo_node" 2>/dev/null || true
    pkill -9 -f "dual_csi_camera" 2>/dev/null || true
    pkill -9 -f "yolo_ros_hailort" 2>/dev/null || true
    pkill -9 -f "component_container" 2>/dev/null || true
    sleep 2
fi

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

# Fallback pkill — also kill any scripts holding Dobot ports
# Kill any Python scripts holding Dobot dashboard/motion ports
pkill -9 -f "dobot_api" 2>/dev/null || true
fuser -k 29999/tcp 2>/dev/null || true
pkill -9 -f "cartridge_providesystem_py" 2>/dev/null || true
pkill -9 -f "vfd_logic_node.py" 2>/dev/null || true
pkill -9 -f "cartridge_gui.py" 2>/dev/null || true
pkill -9 -f "unified_control_gui/unified_control_gui" 2>/dev/null || true
pkill -9 -f "robot_logic_node" 2>/dev/null || true
pkill -9 -f "motion_executor" 2>/dev/null || true
pkill -9 -f "dobot_bringup" 2>/dev/null || true
pkill -9 -f "gripper_festo_node" 2>/dev/null || true
pkill -9 -f "dual_csi_camera" 2>/dev/null || true
pkill -9 -f "yolo_ros_hailort" 2>/dev/null || true
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
PID_DOBOT=""
PID_ROBOT=""
PID_GRIPPER=""
PID_CAMERA=""
PID_WEB_GUI=""
PID_QML_GUI=""

_CLEANUP_DONE=0
cleanup() {
    [ "$_CLEANUP_DONE" -eq 1 ] && return
    _CLEANUP_DONE=1
    echo ""
    echo "🛑 Shutting down..."
    rm -f "$PIDFILE"
    for pid in "${PID_QML_GUI:-}" "${PID_WEB_GUI:-}" "${PID_CAMERA:-}" "${PID_GRIPPER:-}" "${PID_ROBOT:-}" "${PID_DOBOT:-}" "${PID_PROVIDE:-}"; do
        [ -n "$pid" ] && kill "$pid" 2>/dev/null || true
    done
    sleep 2
    for pid in "${PID_QML_GUI:-}" "${PID_WEB_GUI:-}" "${PID_CAMERA:-}" "${PID_GRIPPER:-}" "${PID_ROBOT:-}" "${PID_MOTION:-}" "${PID_DOBOT:-}" "${PID_PROVIDE:-}"; do
        [ -n "$pid" ] && kill -9 "$pid" 2>/dev/null || true
    done
    # Fallback kill all known process names
    pkill -9 -f "cartridge_providesystem_py" 2>/dev/null || true
    pkill -9 -f "cartridge_gui.py" 2>/dev/null || true
    pkill -9 -f "unified_control_gui/unified_control_gui" 2>/dev/null || true
    pkill -9 -f "robot_logic_node" 2>/dev/null || true
    pkill -9 -f "motion_executor" 2>/dev/null || true
    pkill -9 -f "dobot_bringup" 2>/dev/null || true
    pkill -9 -f "gripper_festo_node" 2>/dev/null || true
    pkill -9 -f "dual_csi_camera" 2>/dev/null || true
    pkill -9 -f "yolo_ros_hailort" 2>/dev/null || true
    pkill -9 -f "component_container" 2>/dev/null || true
    # Kill remote RevPi A nodes
    ssh -o ConnectTimeout=2 -o BatchMode=yes "pi@192.168.27.193" "pkill -9 -f rs485_bus_node 2>/dev/null || true" &>/dev/null || true
    echo "✅ All processes stopped."
}
trap cleanup EXIT INT TERM HUP QUIT

# ══════════════════════════════════════════
# CARTRIDGE FEEDER SYSTEM
# ══════════════════════════════════════════

# ── [1/7] Cartridge Provide System Node (servo control) ──
LOG_PROVIDE="$LOG_DIR/cartridge_node.log"
CARTRIDGE_BIN="$WS/install/system_feed_cartridge/lib/system_feed_cartridge/cartridge_providesystem_py"
echo "  [1/7] 🔧 Cartridge Provide System Node..."
"$CARTRIDGE_BIN" > "$LOG_PROVIDE" 2>&1 &
PID_PROVIDE=$!
echo "        PID=$PID_PROVIDE  Log: $LOG_PROVIDE"
echo "$PID_PROVIDE" > "$PIDFILE"

# ── [1.2/7] VFD Logic Node (Independent VFD logic) ──
LOG_VFD="$LOG_DIR/vfd_logic.log"
echo "  [1.2/7] ⚙️  VFD Independent Logic Node..."
nohup python3 "$WS/src/unified_control_gui/scripts/vfd_logic_node.py" > "$LOG_VFD" 2>&1 &
PID_VFD=$!
echo "        PID=$PID_VFD  Log: $LOG_VFD"
echo "$PID_VFD" >> "$PIDFILE"

sleep 3

# ══════════════════════════════════════════
# REVPI A — LOADCELL NODE & FILL MACHINE (Remote via SSH)
# ══════════════════════════════════════════

REVPI_A_IP="192.168.27.193"
REVPI_A_USER="pi"
LOG_REVPI="$LOG_DIR/revpi_a_nodes.log"

echo "  [1.5/7] ⚖️  RevPi A — Loadcell & Fill Machine (SSH → $REVPI_A_IP)..."

if ssh -o ConnectTimeout=3 -o BatchMode=yes "${REVPI_A_USER}@${REVPI_A_IP}" "echo ok" &>/dev/null; then
    ssh -o BatchMode=yes "${REVPI_A_USER}@${REVPI_A_IP}" "pkill -9 -f rs485_bus_node 2>/dev/null || true; pkill -9 -f cpx_festo_node 2>/dev/null || true" &>/dev/null

    ssh -o BatchMode=yes "${REVPI_A_USER}@${REVPI_A_IP}" "
        source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null
        source ~/ros_ws/install/setup.bash 2>/dev/null || true
        export ROS_DOMAIN_ID=22
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
        nohup ros2 run com_rs485 rs485_bus_node --ros-args \
            -p slave_id:=2 \
            -p loadcell_slave_id:=3 \
            -p port:=/dev/ttyRS485 \
            -p baudrate:=9600 \
            -p stopbits:=1 \
            -p loadcell_period_ms:=500 \
            > ~/loadcell_node.log 2>&1 &
        
        nohup ros2 run main_pkg cpx_festo_node.py --ros-args \
            -p slave_id:=254 \
            > ~/cpx_festo_node.log 2>&1 &
            
        echo \$!
    " > /tmp/revpi_a_pid.txt 2>"$LOG_REVPI" &
    PID_REVPI_SSH=$!
    echo "$PID_REVPI_SSH" >> "$PIDFILE"
    sleep 2
else
    echo "        ❌ Cannot SSH to RevPi A ($REVPI_A_IP) — nodes NOT started"
fi

# ══════════════════════════════════════════
# ROBOT SYSTEM
# ══════════════════════════════════════════

# ── [2/7] Dobot Bringup (Nova5 driver) ──
LOG_DOBOT="$LOG_DIR/dobot_bringup.log"
echo "  [2/7] 🤖 Dobot Bringup (Nova5)..."
ros2 launch dobot_bringup_v3 nova5.launch.py > "$LOG_DOBOT" 2>&1 &
PID_DOBOT=$!
echo "        PID=$PID_DOBOT  Log: $LOG_DOBOT"
echo "$PID_DOBOT" >> "$PIDFILE"
sleep 2

# ── [3/8] Robot Logic Node (pick-and-place) ──
LOG_ROBOT="$LOG_DIR/robot_logic_node.log"
echo "  [3/8] 🧠 Robot Logic Node..."
ros2 run robot_control_main robot_logic_node --ros-args --params-file "$WS/src/robot_control_main/config/joint_pose_params.yaml" > "$LOG_ROBOT" 2>&1 &
PID_ROBOT=$!
echo "        PID=$PID_ROBOT  Log: $LOG_ROBOT"
echo "$PID_ROBOT" >> "$PIDFILE"

# ── [3.5/8] Motion Executor (Action Server) ──
LOG_MOTION="$LOG_DIR/motion_executor.log"
echo "  [3.5/8] ⚙️  Motion Executor Node..."
ros2 run robot_control_main motion_executor --ros-args --params-file "$WS/src/robot_control_main/config/joint_pose_params.yaml" > "$LOG_MOTION" 2>&1 &
PID_MOTION=$!
echo "        PID=$PID_MOTION  Log: $LOG_MOTION"
echo "$PID_MOTION" >> "$PIDFILE"
sleep 1

# ── [4/7] Gripper Node (Festo CPX, venv) ──
LOG_GRIPPER="$LOG_DIR/gripper_festo_node.log"
echo "  [4/7] 🦾 Gripper Festo Node..."
"$WS/run_gripper_node.sh" > "$LOG_GRIPPER" 2>&1 &
PID_GRIPPER=$!
echo "        PID=$PID_GRIPPER  Log: $LOG_GRIPPER"
echo "$PID_GRIPPER" >> "$PIDFILE"
sleep 1

# ── [5/7] Dual Camera System (CSI + YOLO) ──
LOG_CAMERA="$LOG_DIR/dual_camera_system.log"
echo "  [5/7] 📷 Dual Camera System (CSI + YOLO)..."
ros2 launch csi_camera dual_camera_system.launch.py > "$LOG_CAMERA" 2>&1 &
PID_CAMERA=$!
echo "        PID=$PID_CAMERA  Log: $LOG_CAMERA"
echo "$PID_CAMERA" >> "$PIDFILE"
sleep 1

# ══════════════════════════════════════════
# GUI
# ══════════════════════════════════════════

# ── [6/7] Web GUI (cartridge_gui.py — port 8080) — OPTIONAL ──
LOG_WEB="$LOG_DIR/cartridge_web_gui.log"
WEB_GUI="$WS/src/system_feed_cartridge/scripts/cartridge_gui.py"
WEB_GUI_ENABLED=false
for arg in "$@"; do [ "$arg" = "--web" ] && WEB_GUI_ENABLED=true; done

if $WEB_GUI_ENABLED && [ -f "$WEB_GUI" ]; then
    echo "  [6/7] 🌐 Web GUI (port 8080)..."
    python3 "$WEB_GUI" > "$LOG_WEB" 2>&1 &
    PID_WEB_GUI=$!
    echo "        PID=$PID_WEB_GUI  Log: $LOG_WEB  Access: http://$(hostname -I | awk '{print $1}'):8080"
    echo "$PID_WEB_GUI" >> "$PIDFILE"
    sleep 1
else
    echo "  [6/7] ⏭️  Web GUI skipped (thêm --web để bật)"
fi

# ── [7/7] QML GUI (native, HDMI) ──
LOG_QML="$LOG_DIR/unified_gui.log"
QML_BIN="$WS/install/unified_control_gui/lib/unified_control_gui/unified_control_gui"
if [ -n "${DISPLAY:-}" ]; then
    echo "  [7/7] 🖥️  QML GUI (DISPLAY=$DISPLAY)..."
    "$QML_BIN" > "$LOG_QML" 2>&1 &
    PID_QML_GUI=$!
    echo "        PID=$PID_QML_GUI  Log: $LOG_QML"
    echo "$PID_QML_GUI" >> "$PIDFILE"
else
    echo "  [7/7] ⚠️  DISPLAY not set — skipping QML GUI"
fi

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "✅ All processes started! (7 components)"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "📋 Logs:"
echo "  tail -f $LOG_PROVIDE        # Cartridge feeder"
echo "  tail -f $LOG_DOBOT          # Dobot driver"
echo "  tail -f $LOG_ROBOT          # Robot logic"
echo "  tail -f $LOG_GRIPPER        # Gripper"
echo "  tail -f $LOG_CAMERA         # Camera + YOLO"
[ -n "${PID_QML_GUI:-}" ] && echo "  tail -f $LOG_QML             # QML GUI"
echo ""
echo "🌐 Web GUI: bash start_all.sh --web"
echo ""
echo "Press Ctrl+C to stop all"
echo ""

# Monitor — exit nếu critical process chết
while true; do
    sleep 3
    if [ -n "${PID_QML_GUI:-}" ] && ! kill -0 "$PID_QML_GUI" 2>/dev/null; then
        echo "[GUI] Closed — dừng hệ thống"
        break
    fi
done
