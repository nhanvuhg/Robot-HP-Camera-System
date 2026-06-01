#!/usr/bin/env python3
"""
camera_only.launch.py

Single-camera test launch: starts ONLY the CSI camera node (CAM0 / Input Tray).
No YOLO, no bbox drawer, no vision_decision — for verifying camera publishes
/cam0HP/image_raw and that the GUI can subscribe to it.

Use this BEFORE wiring YOLO. For the full pipeline use dual_camera_system.launch.py.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cam_node = Node(
        package='csi_camera',
        executable='dual_csi_camera_node',
        name='dual_csi_camera_node',
        output='screen',
        parameters=[{
            'width': 1280,
            'height': 720,
            'fps': 30,
            'publish_fps': 15,
            'cam0_enable': True,
            'cam1_enable': False,            # CAM1 not physically connected
            'cam0_topic': '/cam0HP/image_raw',
            'cam1_topic': '/cam1HP/image_raw',
        }],
        respawn=True,
        respawn_delay=5.0,
    )

    return LaunchDescription([cam_node])
