#!/usr/bin/env python3
"""
camera_with_overlay.launch.py

Single-camera (CAM0/Input Tray) pipeline with YOLO + bbox overlay.
Wires:
  CSI cam0 ─► /cam0HP/image_raw
                  │
                  ▼
              yolo_cam0  ─► /cam0HP/yolo/bounding_boxes
                  │
                  ▼
          overlay_dual_cam ─► /cam0HP/image_overlay   ← subscribe trên GUI

Mirrors the cam0 channel of dual_camera_system.launch.py but without cam1 +
without vision_decision_node — for testing detection on a single physical
camera. Cam1 path inside overlay node just stays idle (no input).

Default hef:
  /home/pi/input_1_yolov8s.hef   (input tray model)
Override with launch arg: model_path:=...
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='/home/pi/input_1_yolov8s.hef',
        description='Path to YOLO .hef model for cam0 (input tray)',
    )
    model_path = LaunchConfiguration('model_path')

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
            'cam1_enable': False,
            'cam0_topic': '/cam0HP/image_raw',
            'cam1_topic': '/cam1HP/image_raw',
        }],
        respawn=True,
        respawn_delay=5.0,
    )

    yolo_cam0 = Node(
        package='yolo_ros_hailort_cpp',
        executable='yolo_ros_hailort_cpp_node',
        name='yolo_cam0',
        output='screen',
        parameters=[{
            'model_path': model_path,
            'src_image_topic_name': '/cam0HP/image_raw',
            'publish_boundingbox_topic_name': '/cam0HP/yolo/bounding_boxes',
            'publish_image_topic_name': '/cam0HP/yolo/image_raw',
            'conf': 0.35,
            'publish_resized_image': False,
        }],
        respawn=True,
        respawn_delay=3.0,
    )

    overlay = Node(
        package='bbox_drawer_cpp',
        executable='overlay_bboxes_node',
        name='overlay_dual_cam',
        output='screen',
        parameters=[{
            'cam0.image_topic':   '/cam0HP/image_raw',
            'cam0.boxes_topic':   '/cam0HP/yolo/bounding_boxes',
            'cam0.output_topic':  '/cam0HP/image_overlay',
            # Match camera publish res (1280x720, 16:9) → bypass resize,
            # preserve aspect ratio bằng với image_raw trên GUI.
            'cam0.output_width':  1280,
            'cam0.output_height': 720,
            # cam1.* dùng default — không có nguồn → kênh cam1 idle, vô hại
        }],
        respawn=True,
        respawn_delay=2.0,
    )

    return LaunchDescription([
        model_path_arg,
        cam_node,
        yolo_cam0,
        overlay,
    ])
