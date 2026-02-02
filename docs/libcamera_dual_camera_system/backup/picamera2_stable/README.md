# Picamera2 Stable Dual Camera - Backup

**Date**: 2026-02-02  
**Status**: Production Ready ✅

## Architecture

**Approach**: Timer-based Alternating Capture with Picamera2 Python API

### Why This Works

❌ **Previous (libcamera C++)**:
- Dual concurrent streams → ISP resource contention
- Kernel 6.12 + libcamera v0.5.2 = unstable
- Crashed after 30-60 seconds
- "Camera frontend timed out" errors

✅ **Current (Picamera2 Python)**:
- Single process, alternating timer
- Prevents ISP conflicts
- Official Raspberry Pi Foundation approach
- Stable 60+ minutes (tested)

## Performance

| Metric | Value |
|--------|-------|
| **Camera FPS** | 6-7 per camera |
| **YOLO Inference** | 23-26ms (Hailo-8L) |
| **CPU Usage** | ~30-35% total |
| **Memory** | ~270MB |
| **Temperature** | 45-50°C |
| **Errors** | 0 |

## Files in This Backup

1. **picamera2_dual_node.py** - Main camera node
   - Timer period: 80-150ms (adaptive)
   - Diagnostics enabled
   - Auto-recovery on errors

2. **dual_camera_system.launch.py** - Launch configuration
   - Starts camera + YOLO + bbox overlay
   - Respawn enabled
   - Updated documentation

3. **CMakeLists.txt** - Build configuration
   - Python script installation
   - Package dependencies

## How to Use

```bash
# Launch full system
cd ~/ros2_ws
source install/setup.bash
ros2 launch csi_camera dual_camera_system.launch.py

# Check FPS
ros2 topic hz /cam0HP/image_raw
ros2 topic hz /cam1HP/image_raw

# View diagnostics in log output
```

## Restore from Backup

```bash
cd ~/ros2_ws
cp docs/libcamera_dual_camera_system/backup/picamera2_stable/* src/csi_camera/scripts/
cp docs/libcamera_dual_camera_system/backup/picamera2_stable/dual_camera_system.launch.py src/csi_camera/launch/
cp docs/libcamera_dual_camera_system/backup/picamera2_stable/CMakeLists.txt src/csi_camera/
colcon build --packages-select csi_camera
```

## Key Optimizations Applied

1. **Camera Init Delay**: 0.5s → 1.5s (CSI stability)
2. **Timer Period**: Adaptive 80-150ms (prevents queue buildup)
3. **Buffer Count**: 4 buffers per camera (optimal for Pi 5)
4. **Error Recovery**: Automatic camera restart on failures

## Tested Scenarios

✅ 60+ minute continuous run  
✅ People entering frame (YOLO stress test)  
✅ Both cameras simultaneously  
✅ With Hailo-8L acceleration  
✅ System reboot and restart  

## Known Limitations

- FPS: 6-7 per camera (not 30 FPS due to alternating pattern)
- Single process (Python GIL may limit parallelism)
- Requires Picamera2 library

## Production Deployment

This configuration is **PRODUCTION READY** for robot vision system.

**Recommendation**: Use this as the stable baseline for dual camera operations.
