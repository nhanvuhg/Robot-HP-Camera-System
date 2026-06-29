#!/usr/bin/env bash
# Deploy com_rs485 package lên RevPi A — chỉ cần chạy 1 lần
# Usage: bash ~/deploy_revpi.sh [revpi-host] [revpi-user]
set -euo pipefail

# Source ros2_env.sh để có $REVPI_A_HOST (single source of truth cho IP RevPi A)
# shellcheck source=/dev/null
[ -f "$HOME/ros2_ws/ros2_env.sh" ] && source "$HOME/ros2_ws/ros2_env.sh"
REVPI_HOST="${1:-${REVPI_HOST:-${REVPI_A_HOST:-192.168.27.197}}}"
REVPI_USER="${2:-${REVPI_USER:-pi}}"
REVPI_WS="${REVPI_WS:-/home/${REVPI_USER}/ros2_jazzy}"
PI5_WS="$HOME/ros2_ws"

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  Deploy com_rs485 → ${REVPI_USER}@${REVPI_HOST}"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# 1. Kiểm tra RevPi A có ping được không
if ! ping -c 1 -W 2 "$REVPI_HOST" >/dev/null 2>&1; then
    echo "❌ Không ping được $REVPI_HOST — kiểm tra LAN/hostname"
    exit 1
fi
echo "✅ RevPi A ($REVPI_HOST) reachable"

# 2. Copy source package
echo ""
echo "[1/4] Copying com_rs485 source..."
ssh "${REVPI_USER}@${REVPI_HOST}" "mkdir -p ${REVPI_WS}/src"
scp -r "${PI5_WS}/src/com_rs485" "${REVPI_USER}@${REVPI_HOST}:${REVPI_WS}/src/"
echo "      ✅ Source copied"

# 3. Install libmodbus trên RevPi A (nếu chưa có)
echo ""
echo "[2/4] Installing libmodbus-dev on RevPi A..."
ssh "${REVPI_USER}@${REVPI_HOST}" \
    "dpkg -l libmodbus-dev >/dev/null 2>&1 || sudo apt-get install -y libmodbus-dev" \
    && echo "      ✅ libmodbus-dev ready"

# 4. Build trên RevPi A
echo ""
echo "[3/4] Building com_rs485 on RevPi A..."
ssh "${REVPI_USER}@${REVPI_HOST}" "
    source /opt/ros/jazzy/setup.bash
    cd ${REVPI_WS}
    colcon build --base-paths src --packages-select com_rs485 --cmake-args -DCMAKE_BUILD_TYPE=Release
" && echo "      ✅ Build OK"

# 5. Copy FastDDS config — ros2_env.sh trỏ tới /home/pi/fastdds_peers.xml
#    (unicast peers cross-host). Cũng giữ legacy fastdds_no_shm.xml nếu có.
echo ""
echo "[4/5] Copying FastDDS configs..."
if [ -f "${PI5_WS}/fastdds_peers.xml" ]; then
    scp "${PI5_WS}/fastdds_peers.xml" "${REVPI_USER}@${REVPI_HOST}:/home/${REVPI_USER}/fastdds_peers.xml"
    echo "      ✅ fastdds_peers.xml → /home/${REVPI_USER}/ (cross-host unicast)"
else
    echo "      ⚠️  fastdds_peers.xml không tìm thấy — bỏ qua"
fi
if [ -f "${PI5_WS}/fastdds_no_shm.xml" ]; then
    scp "${PI5_WS}/fastdds_no_shm.xml" "${REVPI_USER}@${REVPI_HOST}:/home/${REVPI_USER}/fastdds_no_shm.xml"
    echo "      ✅ fastdds_no_shm.xml → /home/${REVPI_USER}/ (legacy)"
fi

# 6. Copy start script — RevPi A đọc từ /home/pi/start_rs485.sh
echo ""
echo "[5/6] Copying start_rs485.sh..."
if [ -f "${PI5_WS}/start_rs485_revpi.sh" ]; then
    scp "${PI5_WS}/start_rs485_revpi.sh" "${REVPI_USER}@${REVPI_HOST}:/home/${REVPI_USER}/start_rs485.sh"
    ssh "${REVPI_USER}@${REVPI_HOST}" "chmod +x /home/${REVPI_USER}/start_rs485.sh"
    echo "      ✅ start_rs485.sh → /home/${REVPI_USER}/ (mode +x)"
else
    echo "      ⚠️  start_rs485_revpi.sh không tìm thấy — bỏ qua"
fi

# 7. Copy ros2_env.sh + cài đặt vào ~/.bashrc RevPi A (idempotent)
echo ""
echo "[6/6] Installing ros2_env.sh + ~/.bashrc hook..."
if [ -f "${PI5_WS}/ros2_env.sh" ]; then
    scp "${PI5_WS}/ros2_env.sh" "${REVPI_USER}@${REVPI_HOST}:/home/${REVPI_USER}/ros2_env.sh"
    ssh "${REVPI_USER}@${REVPI_HOST}" "
        for f in ~/.bashrc ~/.profile; do
            [ -f \"\$f\" ] || touch \"\$f\"
            grep -qxF 'source /home/${REVPI_USER}/ros2_env.sh' \"\$f\" \
              || echo 'source /home/${REVPI_USER}/ros2_env.sh' >> \"\$f\"
        done
    "
    echo "      ✅ ros2_env.sh → /home/${REVPI_USER}/ + ~/.bashrc + ~/.profile hook"
else
    echo "      ⚠️  ros2_env.sh không tìm thấy — bỏ qua"
fi

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "✅ Deploy xong! Giờ chỉ cần chạy:"
echo "   bash ~/start_all.sh"
echo ""
echo "   RevPi A sẽ tự được start rs485_bus_node."
echo ""
echo "   Nếu muốn start thủ công trên RevPi A:"
echo "   ssh ${REVPI_USER}@${REVPI_HOST} bash /home/${REVPI_USER}/start_rs485.sh"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
