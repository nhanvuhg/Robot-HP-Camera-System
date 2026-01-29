#!/bin/bash
# GUI auto-restart wrapper
# Automatically restarts GUI if it crashes

export DISPLAY=${DISPLAY:-:0}
LOG_FILE="/home/pi/ros2_ws/logs/qml_gui.log"

source /home/pi/ros2_ws/install/setup.bash

while true; do
    echo "[$(date)] Starting GUI..." >> "$LOG_FILE"
    ros2 run ros2_qml_gui1 ros2_qml_gui1 >> "$LOG_FILE" 2>&1
    EXIT_CODE=$?
    
    if [ $EXIT_CODE -ne 0 ]; then
        echo "[$(date)] GUI crashed with exit code $EXIT_CODE. Restarting in 3 seconds..." >> "$LOG_FILE"
        sleep 3
    else
        echo "[$(date)] GUI exited normally." >> "$LOG_FILE"
        break
    fi
done
