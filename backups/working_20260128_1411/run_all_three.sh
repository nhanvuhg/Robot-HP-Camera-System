#!/usr/bin/env bash
set -euo pipefail

# Start all robot system processes:
# - Dobot bringup
# - Robot logic node
# - Gripper node
# - CSI camera node
# - YOLO detection launch
# Usage: ./run_all_three.sh

WS="$HOME/ros2_ws"

# Set DISPLAY to show GUI on Pi's HDMI screen (even when SSH)
export DISPLAY=${DISPLAY:-:0}

# Source ROS 2 and workspace overlays if present
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
  # shellcheck source=/dev/null
  set +u
  
  # Disable SHM transport to avoid RTPS_TRANSPORT_SHM errors
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export FASTRTPS_DEFAULT_PROFILES_FILE="$WS/fastdds_no_shm.xml"
  
  source /opt/ros/jazzy/setup.bash || true
  set -u
fi
if [ -f "$WS/install/setup.bash" ]; then
  # shellcheck source=/dev/null
  set +u
  source "$WS/install/setup.bash" || true
  set -u
fi

LOG_DIR="$WS/logs"
mkdir -p "$LOG_DIR"

# ✅ Check for existing processes before starting
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "🔍 Checking for duplicate processes..."

# Use || true to prevent grep exit code from triggering set -e
EXISTING=$(ps aux | grep -E "(robot_logic_node|dobot_bringup|gripper_festo|dual_csi_camera|yolo_ros_hailort|ros2_qml_gui1)" | grep -v -E "(grep|tail)" | wc -l || true)

if [ "${EXISTING:-0}" -gt 0 ]; then
    echo "⚠️  WARNING: Found $EXISTING existing process(es)!"
    echo ""
    ps aux | grep -E "(robot_logic_node|dobot_bringup|gripper_festo|dual_csi_camera|yolo_ros_hailort|ros2_qml_gui1)" | grep -v -E "(grep|tail)" || true
    echo ""
    echo "Please stop them first with:"
    echo "  ./stop_all.sh"
    echo ""
    exit 1
fi
echo "✅ No duplicates found. Starting processes..."
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

DOBOT_LOG="$LOG_DIR/dobot_bringup.log"
ROBOT_LOG="$LOG_DIR/robot_logic_node.log"
GRIPPER_LOG="$LOG_DIR/gripper_festo_node.log"
CAMERA_LOG="$LOG_DIR/dual_camera_system.log"
GUI_LOG="$LOG_DIR/qml_gui.log"

echo "Starting dobot_bringup_v3 nova5.launch.py (background). Log: $DOBOT_LOG"
ros2 launch dobot_bringup_v3 nova5.launch.py > "$DOBOT_LOG" 2>&1 &
PID_DOBOT=$!
echo "dobot bringup PID=$PID_DOBOT"

echo "Starting robot_logic_node with params file (background). Log: $ROBOT_LOG"
ros2 run robot_control_main robot_logic_node --ros-args --params-file "$WS/src/robot_control_main/config/joint_pose_params.yaml" > "$ROBOT_LOG" 2>&1 &
PID_ROBOT=$!
echo "robot_logic_node PID=$PID_ROBOT"

echo "Starting gripper node (background via venv wrapper). Log: $GRIPPER_LOG"
"$WS/run_gripper_node.sh" > "$GRIPPER_LOG" 2>&1 &
PID_GRIPPER=$!
echo "gripper_festo_node PID=$PID_GRIPPER"

echo "Starting Dual CSI Camera System (camera + YOLO for both). Log: $CAMERA_LOG"
ros2 launch csi_camera dual_camera_system.launch.py > "$CAMERA_LOG" 2>&1 &
PID_CAMERA=$!
echo "dual_camera_system PID=$PID_CAMERA"

# Only start GUI if DISPLAY is available
if [ -n "${DISPLAY:-}" ]; then
    echo "Starting QML GUI (background). Log: $GUI_LOG"
    ros2 run ros2_qml_gui1 ros2_qml_gui1 > "$GUI_LOG" 2>&1 &
    PID_GUI=$!
    echo "qml_gui PID=$PID_GUI"
else
    echo "⚠️  DISPLAY not set - skipping GUI (use VNC/X11 to enable)"
    PID_GUI=""
fi

cleanup() {
  echo "Cleaning up..."
  for pid in "$PID_GUI" "$PID_CAMERA" "$PID_GRIPPER" "$PID_ROBOT" "$PID_DOBOT"; do
    if [ -n "$pid" ] && ps -p "$pid" > /dev/null 2>&1; then
      echo "Killing PID $pid"
      kill "$pid" || true
      wait "$pid" 2>/dev/null || true
    fi
  done
}

trap cleanup EXIT INT TERM

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "✅ All processes started successfully!"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "Tail logs with:"
echo "  tail -f $ROBOT_LOG $GRIPPER_LOG $DOBOT_LOG"
echo "  tail -f $CAMERA_LOG  # Contains camera + YOLO output"
echo "  tail -f $GUI_LOG     # QML GUI output"
echo ""

# Wait indefinitely (cleanup will run on Ctrl-C)
while true; do sleep 3600; done
