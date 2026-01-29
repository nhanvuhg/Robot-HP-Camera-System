# Dual Camera Production Stabilization - Walkthrough

## Problem
Camera system was experiencing intermittent timeouts on both cameras, causing recovery loops every few seconds. This occurred despite hardware checks passing (temp, power, throttle all OK).

## Root Cause Analysis
**libcamera Dual Camera Resource Conflict** - When running 2 cameras simultaneously via libcamera API:
- Both cameras' callbacks compete for processing resources
- ISP pipelines share internal resources on Pi 5
- Buffer queue starvation when one camera blocks

## Solution Implemented

### Changes to [libcamera_dual_node.cpp](file:///home/pi/ros2_ws/src/csi_camera/src/libcamera_dual_node.cpp)

| Change | Before | After |
|--------|--------|-------|
| Default FPS | 30 | **15** (reduces system load) |
| Buffer count | 4 | **6** (more headroom) |
| Processing model | Parallel | **Serialized** (mutex-protected) |
| Recovery | Single camera restart | **Full restart after 3 failures** |
| Failure tracking | None | **Consecutive failure counter** |

### Key Code Changes

1. **Serialized Processing** - Only one camera processes at a time via `capture_mutex_`
   ```cpp
   std::lock_guard<std::mutex> capture_lock(capture_mutex_);
   ```

2. **Enhanced Recovery** - Full libcamera restart after 3 consecutive failures
   ```cpp
   if (consecutive_failures_[cam_id] >= 3) {
       fullRestart();
   }
   ```

3. **FPS in launch script**
   - Updated `run_all_three_v2.sh` to use `-p fps:=15`

---

## Verification Results

### Before Fix
```
⚠️ Camera 0 stuck! No frames for 7 seconds. Attempting recovery...
⚠️ Camera 1 stuck! No frames for 6 seconds. Attempting recovery...
(Repeating every 6-8 seconds)
```

### After Fix
```
📊 Cam0: 1391 frames, Cam1: 1255 frames
(Running stable for 2+ minutes without any recovery)
```

### Performance
| Metric | Value |
|--------|-------|
| Cam0 FPS | ~11.6 fps |
| Cam1 FPS | ~10.5 fps |
| Total throughput | ~22 fps combined |
| Recovery attempts | 1 (initial only) |
| Stability | ✅ 2+ minutes without issues |

---

## Files Changed

1. [libcamera_dual_node.cpp](file:///home/pi/ros2_ws/src/csi_camera/src/libcamera_dual_node.cpp) - Core stability improvements
2. [run_all_three_v2.sh](file:///home/pi/ros2_ws/run_all_three_v2.sh) - Updated FPS parameter

---

## Recommendations for Production

1. **Monitor frame counts** - Use `tail -f /home/pi/ros2_ws/logs/libcamera_dual_node.log` to verify cameras are running
2. **Check for recovery messages** - Occasional recovery is OK, continuous recovery indicates hardware issue
3. **Temperature monitoring** - Keep system below 70°C for best stability
4. **Power supply** - Use official Pi 5 27W USB-C power supply

> [!TIP]
> If still experiencing issues, consider using the `dual_csi_camera_node.cpp` (rpicam-still backend) which is slower (~3 fps) but extremely stable.
