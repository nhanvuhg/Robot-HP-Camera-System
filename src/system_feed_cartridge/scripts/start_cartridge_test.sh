#!/bin/bash
# ============================================================
# Start Cartridge System + GUI for Testing
# Usage: bash start_cartridge_test.sh
# Stop:  Ctrl+C (kills both processes)
# ============================================================

# Source ROS2
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

echo "========================================"
echo "  🔧 Cartridge System Test Launcher"
echo "========================================"
echo ""

# Kill any old GUI process occupying port 8080
fuser -k 8080/tcp 2>/dev/null
pkill -f "cartridge_gui.py" 2>/dev/null
pkill -f "cartridge_providesystem" 2>/dev/null
sleep 1

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

PID_NODE=""
PID_GUI=""

# Cleanup on exit — kill immediately
cleanup() {
    echo ""
    echo "  Shutting down..."
    
    # Send SIGTERM + SIGKILL immediately to all related processes
    [ -n "$PID_GUI" ] && kill $PID_GUI 2>/dev/null
    [ -n "$PID_NODE" ] && kill $PID_NODE 2>/dev/null
    
    # Brief wait then force kill everything
    sleep 0.5
    
    [ -n "$PID_GUI" ] && kill -9 $PID_GUI 2>/dev/null
    [ -n "$PID_NODE" ] && kill -9 $PID_NODE 2>/dev/null
    pkill -9 -f "cartridge_gui.py" 2>/dev/null
    pkill -9 -f "cartridge_providesystem_py" 2>/dev/null
    
    wait $PID_NODE 2>/dev/null
    wait $PID_GUI 2>/dev/null
    
    echo "  ✅ All processes stopped."
}
trap cleanup EXIT INT TERM

# Start cartridge node
echo "  [1/2] Starting Cartridge Node..."
ros2 run system_feed_cartridge cartridge_providesystem_py &
PID_NODE=$!
sleep 1

# Start GUI
echo "  [2/2] Starting GUI Server..."
python3 "$SCRIPT_DIR/cartridge_gui.py" &
PID_GUI=$!
sleep 1

echo ""
echo "  ✅ Both running!"
echo "  📊 GUI: http://localhost:8080"
echo "  Press Ctrl+C to stop all"
echo ""

# Wait for any process to exit
wait -n $PID_NODE $PID_GUI 2>/dev/null || true
