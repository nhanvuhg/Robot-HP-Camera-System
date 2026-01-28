#!/usr/bin/env bash
set -euo pipefail

# ============================================================================
# RUN ALL THREE V2 - Uses libcamera_dual_node (experimental high-FPS version)
# ============================================================================
# Difference from run_all_three.sh:
#   - Uses libcamera_dual_node (libcamera C++ API, ~30+ FPS)
#   - Instead of dual_camera_system.launch.py (rpicam-still, ~3 FPS)
# ============================================================================

WS="$HOME/ros2_ws"

# Set DISPLAY to show GUI on Pi's HDMI screen (even when SSH)
export DISPLAY=${DISPLAY:-:0}

# Source ROS 2 and workspace overlays if present
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
  set +u
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export FASTRTPS_DEFAULT_PROFILES_FILE="$WS/fastdds_no_shm.xml"
  source /opt/ros/jazzy/setup.bash || true
  set -u
fi
if [ -f "$WS/install/setup.bash" ]; then
  set +u
  source "$WS/install/setup.bash" || true
  set -u
fi

LOG_DIR="$WS/logs"
mkdir -p "$LOG_DIR"

# ✅ Check for existing processes before starting
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "🔍 V2: Checking for duplicate processes..."

EXISTING=$(ps aux | grep -E "(robot_logic_node|dobot_bringup|gripper_festo|libcamera_dual|dual_csi_camera|yolo_ros_hailort|ros2_qml_gui1)" | grep -v -E "(grep|tail)" | wc -l || true)

if [ "${EXISTING:-0}" -gt 0 ]; then
    echo "⚠️  WARNING: Found $EXISTING existing process(es)!"
    ps aux | grep -E "(robot_logic_node|dobot_bringup|gripper_festo|libcamera_dual|dual_csi_camera|yolo_ros_hailort|ros2_qml_gui1)" | grep -v -E "(grep|tail)" || true
    echo ""
    echo "Please stop them first with: ./stop_all.sh"
    exit 1
fi
echo "✅ No duplicates found. Starting V2 processes..."
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

DOBOT_LOG="$LOG_DIR/dobot_bringup.log"
ROBOT_LOG="$LOG_DIR/robot_logic_node.log"
GRIPPER_LOG="$LOG_DIR/gripper_festo_node.log"
CAMERA_LOG="$LOG_DIR/libcamera_dual_node.log"
YOLO_LOG="$LOG_DIR/yolo_system.log"
OVERLAY_LOG="$LOG_DIR/bbox_drawer.log"
GUI_LOG="$LOG_DIR/qml_gui.log"

# 1. Dobot bringup
echo "Starting dobot_bringup_v3 nova5.launch.py..."
ros2 launch dobot_bringup_v3 nova5.launch.py > "$DOBOT_LOG" 2>&1 &
PID_DOBOT=$!
echo "dobot bringup PID=$PID_DOBOT"

# 2. Robot logic node
echo "Starting robot_logic_node..."
ros2 run robot_control_main robot_logic_node --ros-args --params-file "$WS/src/robot_control_main/config/joint_pose_params.yaml" > "$ROBOT_LOG" 2>&1 &
PID_ROBOT=$!
echo "robot_logic_node PID=$PID_ROBOT"

# 3. Gripper node
echo "Starting gripper node..."
"$WS/run_gripper_node.sh" > "$GRIPPER_LOG" 2>&1 &
PID_GRIPPER=$!
echo "gripper_festo_node PID=$PID_GRIPPER"

# 4. NEW: libcamera_dual_node (instead of dual_camera_system.launch.py)
echo "🎥 Starting libcamera_dual_node (HIGH FPS VERSION). Log: $CAMERA_LOG"
ros2 run csi_camera libcamera_dual_node --ros-args -p fps:=30 -p width:=640 -p height:=480 > "$CAMERA_LOG" 2>&1 &
PID_CAMERA=$!
echo "libcamera_dual_node PID=$PID_CAMERA"

# Wait a bit for camera to initialize
sleep 2

# 5. YOLO nodes (separate launch)
echo "🤖 Starting YOLO detection nodes. Log: $YOLO_LOG"
ros2 launch yolo_ros_hailort_cpp system_csi_dual_model.launch.py > "$YOLO_LOG" 2>&1 &
PID_YOLO=$!
echo "yolo_system PID=$PID_YOLO"

sleep 1

# 6. BBox Drawer Node (for image_overlay topics)
echo "🖼️ Starting bbox_drawer_node for overlay. Log: $OVERLAY_LOG"
ros2 run bbox_drawer_cpp overlay_bboxes_node --ros-args \
  -p cam0.image_topic:=/cam0HP/image_raw \
  -p cam0.boxes_topic:=/cam0HP/yolo/bounding_boxes \
  -p cam0.output_topic:=/cam0HP/image_overlay \
  -p cam1.image_topic:=/cam1HP/image_raw \
  -p cam1.boxes_topic:=/cam1HP/yolo/bounding_boxes \
  -p cam1.output_topic:=/cam1HP/image_overlay \
  > "$OVERLAY_LOG" 2>&1 &
PID_OVERLAY=$!
echo "bbox_drawer_node PID=$PID_OVERLAY"

# 7. GUI
if [ -n "${DISPLAY:-}" ]; then
    echo "Starting QML GUI..."
    ros2 run ros2_qml_gui1 ros2_qml_gui1 > "$GUI_LOG" 2>&1 &
    PID_GUI=$!
    echo "qml_gui PID=$PID_GUI"
else
    echo "⚠️  DISPLAY not set - skipping GUI"
    PID_GUI=""
fi

cleanup() {
  echo ""
  echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
  echo "🛑 Cleaning up all processes..."
  
  # Kill tracked PIDs first
  for pid in "$PID_GUI" "$PID_OVERLAY" "$PID_YOLO" "$PID_CAMERA" "$PID_GRIPPER" "$PID_ROBOT" "$PID_DOBOT"; do
    if [ -n "$pid" ] && ps -p "$pid" > /dev/null 2>&1; then
      kill -9 "$pid" 2>/dev/null || true
    fi
  done
  
  # Force kill ALL camera-related processes to release hardware
  pkill -9 libcamera_dual 2>/dev/null || true
  pkill -9 overlay_bboxes 2>/dev/null || true
  pkill -9 -f component_container 2>/dev/null || true
  pkill -9 -f yolo_ros 2>/dev/null || true
  pkill -9 rpicam 2>/dev/null || true
  killall -9 libcamera_dual_node 2>/dev/null || true
  
  sleep 1
  echo "✅ All processes stopped"
  echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
}

trap cleanup EXIT INT TERM

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "✅ V2: All processes started!"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "📊 Using libcamera_dual_node (~30+ FPS) instead of rpicam-still (~3 FPS)"
echo ""
echo "Tail logs with:"
echo "  tail -f $CAMERA_LOG  # Camera output"
echo "  tail -f $YOLO_LOG    # YOLO detection"
echo "  tail -f $ROBOT_LOG $GRIPPER_LOG $DOBOT_LOG"
echo ""

# Wait indefinitely (cleanup will run on Ctrl-C)
while true; do sleep 3600; done
