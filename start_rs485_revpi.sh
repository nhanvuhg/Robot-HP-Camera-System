#!/bin/bash
# RS485 Bus Node Startup — RevPi A
# Rule: start script CHỈ chạy node. Tất cả params (port, baud, slave_id,
# ref_hz) đã set sẵn trong rs485_bus_node.cpp default values. Không truyền
# --ros-args ở đây để tránh drift giữa script và source code.
export ROS_DOMAIN_ID=22
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# Switch không forward multicast → dùng unicast initial peers list để
# Pi 5 ↔ RevPi A thấy được nhau (xem fastdds_peers.xml).
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/pi/fastdds_peers.xml
source /home/pi/ros2_jazzy/install/setup.bash 2>/dev/null

killall -9 rs485_bus_node 2>/dev/null
sleep 1

exec /home/pi/ros2_jazzy/install/com_rs485/lib/com_rs485/rs485_bus_node
