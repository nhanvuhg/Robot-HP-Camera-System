#!/usr/bin/env python3
"""
dual_camera_system.launch.py

Complete dual-camera system for 2x GMSL2 boards on Raspberry Pi 5
- Camera 0 on CSI0 (I2C bus 6) → Input Tray
- Camera 1 on CSI1 (I2C bus 4) → Output Tray

Both cameras run in parallel, no switching required.
"""

from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    
    # ================================================================
    # 1. DUAL CSI CAMERA NODE (Serialized Processing)
    # ================================================================
    # Uses Picamera2 with SERIALIZED PROCESSING PATTERN:
    # - Single timer alternates between cameras
    # - Prevents ISP contention freeze on Pi 5 kernel 6.12
    # Publishes:
    #   - /cam0HP/image_raw (Camera 0 - Input Tray)
    #   - /cam1HP/image_raw (Camera 1 - Output Tray)
    
    dual_camera_node = Node(
        package='csi_camera',
        executable='dual_csi_camera_node',
        name='dual_csi_camera_node',
        output='screen',
        parameters=[{
            'width': 640,
            'height': 480,
            'fps': 10,  # Set to required FPS based on processing requirements
            # cam1 chưa lắp hardware → disable để tránh 15s wait_for_first_frame
            # + spam reconnect log. Đổi True khi cam1 (Output Tray) lắp xong.
            'cam0_enable': True,
            'cam1_enable': False,
            'cam0_topic': '/cam0HP/image_raw',
            'cam1_topic': '/cam1HP/image_raw',
        }],
        respawn=True,
        respawn_delay=5.0,
    )
    
    # ================================================================
    # 2. YOLO CONTAINER WITH 2 MODELS
    # ================================================================
    # Both YOLO nodes run simultaneously, processing their respective camera feeds
    
    yolo_container = ComposableNodeContainer(
        name='yolo_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',  # multi-threaded: allows cam0/cam1 inference overlap
        composable_node_descriptions=[
            # YOLO for Camera 0 (Input Tray Detection)
            ComposableNode(
                package='yolo_ros_hailort_cpp',
                plugin='yolo_ros_hailort_cpp::YoloNode',
                name='yolo_cam0',
                parameters=[{
                    'model_path': '/home/pi/input_1_yolov8s.hef',
                    'src_image_topic_name': '/cam0HP/image_raw',
                    'publish_boundingbox_topic_name': '/cam0HP/yolo/bounding_boxes',
                    'publish_image_topic_name': '/cam0HP/yolo/image_raw',
                    'conf': 0.35,
                    'publish_resized_image': False,
                }]
            ),

            # YOLO for Camera 1 (Output Tray Detection)
            # DISABLED — cam1 hardware chưa lắp + /home/pi/yolov8s.hef chưa có.
            # Uncomment khi cam1 + model sẵn sàng. Cũng nhớ set cam1_enable=True
            # ở dual_camera_node phía trên.
            # ComposableNode(
            #     package='yolo_ros_hailort_cpp',
            #     plugin='yolo_ros_hailort_cpp::YoloNode',
            #     name='yolo_cam1',
            #     parameters=[{
            #         'model_path': '/home/pi/yolov8s.hef',
            #         'src_image_topic_name': '/cam1HP/image_raw',
            #         'publish_boundingbox_topic_name': '/cam1HP/yolo/bounding_boxes',
            #         'publish_image_topic_name': '/cam1HP/yolo/image_raw',
            #         'conf': 0.35,
            #         'publish_resized_image': False,
            #     }]
            # ),
        ],
        output='screen',
        respawn=True,
        respawn_delay=3.0,
    )
    
    # ================================================================
    # 3. BBOX DRAWER (Visualization)
    # ================================================================
    # Overlays bounding boxes on both camera feeds for debugging
    
    bbox_drawer_node = Node(
        package='bbox_drawer_cpp',
        executable='overlay_bboxes_node',
        name='overlay_dual_cam',
        output='screen',
        parameters=[{
            # Camera 0 (Input Tray)
            'cam0.image_topic': '/cam0HP/image_raw',
            'cam0.boxes_topic': '/cam0HP/yolo/bounding_boxes',
            'cam0.output_topic': '/cam0HP/image_overlay',
            'cam0.output_width': 640,
            'cam0.output_height': 480,
            
            # Camera 1 (Output Tray)
            'cam1.image_topic': '/cam1HP/image_raw',
            'cam1.boxes_topic': '/cam1HP/yolo/bounding_boxes',
            'cam1.output_topic': '/cam1HP/image_overlay',
            'cam1.output_width': 640,
            'cam1.output_height': 480,
        }],
        respawn=True,
        respawn_delay=2.0,
    )
    
    # ================================================================
    # 4. VISION DECISION NODE (Row/Slot Selection from YOLO)
    # ================================================================
    # Converts YOLO bounding boxes → robot control decisions
    # Publishes:
    #   - /vision/input_tray/selected_row  (Int32)
    #   - /vision/input_tray/row_status    (Int32MultiArray)
    #   - /vision/input_tray/empty         (Bool)
    #   - /vision/output_tray/selected_slot (Int32)
    #   - /vision/output_tray/slot_status  (Int32MultiArray)
    #   - /vision/output_tray/full         (Bool)
    #   - /vision/heartbeat                (Header)

    vision_decision_node = Node(
        package='robot_control_main',
        executable='vision_decision_node',
        name='vision_decision_node',
        output='screen',
        respawn=True,
        respawn_delay=3.0,
    )

    return LaunchDescription([
        dual_camera_node,
        yolo_container,
        bbox_drawer_node,
        vision_decision_node,
    ])

"""
================================================================================
🚀 DUAL GMSL2 CAMERA SYSTEM
================================================================================

Hardware:
  ┌──────────────────────────────────────────────┐
  │  Raspberry Pi 5                               │
  │                                               │
  │  CSI0 (I2C 6) → GMSL2 Board 1 → Camera 0     │
  │  CSI1 (I2C 4) → GMSL2 Board 2 → Camera 1     │
  └──────────────────────────────────────────────┘

Topic Flow:
  Camera 0 Thread → /cam0HP/image_raw → YOLO cam0 → /cam0HP/yolo/bounding_boxes
                                                   ↓
                                              bbox_drawer → /cam0HP/image_overlay
  
  Camera 1 Thread → /cam1HP/image_raw → YOLO cam1 → /cam1HP/yolo/bounding_boxes
                                                   ↓
                                              bbox_drawer → /cam1HP/image_overlay

Launch:
  ros2 launch csi_camera dual_camera_system.launch.py

Verify:
  # Check both cameras publishing (expect ~6-8 Hz with alternating pattern)
  ros2 topic hz /cam0HP/image_raw
  ros2 topic hz /cam1HP/image_raw
  
  # Check YOLO detections
  ros2 topic echo /cam0HP/yolo/bounding_boxes
  ros2 topic echo /cam1HP/yolo/bounding_boxes
  
  # View live feeds
  ros2 run rqt_image_view rqt_image_view
    → Select /cam0HP/image_overlay
    → Select /cam1HP/image_overlay

Performance:
  ✅ Serialized processing - both cameras at 6-8 FPS (alternating capture)
  ✅ Prevents ISP freeze on Pi 5 Kernel 6.12
  ✅ Independent YOLO inference on each stream (23-24ms with Hailo-8L)
  ✅ Stable operation without CSI timeout errors

================================================================================
"""
