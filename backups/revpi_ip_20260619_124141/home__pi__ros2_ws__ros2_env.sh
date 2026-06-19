# ROS 2 + FastDDS env — single source of truth.
# Source từ ~/.bashrc để mọi shell/process tự động có đúng env, không phụ
# thuộc vào start_all.sh hay start_rs485.sh.
#
# Pi 5 path:    source /home/pi/ros2_ws/ros2_env.sh
# RevPi A path: source /home/pi/ros2_env.sh   (deploy_revpi.sh copy sang)
#
# Cài đặt 1 lần (idempotent):
#   grep -qxF 'source /home/pi/ros2_ws/ros2_env.sh' ~/.bashrc \
#     || echo 'source /home/pi/ros2_ws/ros2_env.sh' >> ~/.bashrc

export ROS_DOMAIN_ID=22
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

# Host của RevPi A — single source of truth cho IP RevPi A.
# Dùng cho SSH start rs485_bus_node + loadcell_node, và iframe web GUI loadcell.
# Đổi IP bằng /home/pi/Desktop/update_revpi_ip.sh để tự đồng bộ file này,
# fastdds_peers.xml, Start All và config trên RevPi A.
# Lưu ý: fastdds_peers.xml là XML static, FastDDS không support env-var trong
# <address>, nên updater sẽ sửa các locator tương ứng. Sanity check:
#   bash scripts/check_revpi_ip_sync.sh
export REVPI_A_HOST="192.168.27.197"

# Cross-host discovery: unicast peers (xem fastdds_peers.xml). Cùng file
# này được dùng trên cả Pi 5 (path tuyệt đối) và RevPi A (path tuyệt đối
# /home/pi/fastdds_peers.xml sau khi deploy_revpi.sh copy sang).
if [ -f /home/pi/ros2_ws/fastdds_peers.xml ]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE=/home/pi/ros2_ws/fastdds_peers.xml
elif [ -f /home/pi/fastdds_peers.xml ]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE=/home/pi/fastdds_peers.xml
fi
