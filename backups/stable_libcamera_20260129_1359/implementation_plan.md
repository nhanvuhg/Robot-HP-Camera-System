# Stabilize Dual Camera System for Production

## Problem Analysis

### Root Cause
The camera timeout issue is caused by **libcamera dual camera resource contention**, NOT hardware problems:

1. **ISP Resource Sharing**: Pi 5's dual ISPs share internal resources
2. **Callback Thread Conflicts**: Both cameras' callbacks compete for processing time
3. **Buffer Queue Starvation**: When one camera blocks, it affects the other
4. **libcamera v0.5.2 limitations**: Known issues with concurrent dual camera streaming

### Evidence
- Hardware checks pass (temp, power, throttle all OK)
- Individual camera tests work perfectly
- Timeouts occur alternately between cameras (/dev/video4 and /dev/video12)

---

## Proposed Solution: Sequential Frame Capture with Time Multiplexing

Instead of true parallel streaming (which causes conflicts), implement **time-multiplexed capture**:

```
Timeline:  |--Cam0--|--Cam1--|--Cam0--|--Cam1--|
           Frame 0   Frame 1   Frame 2   Frame 3
```

### Key Changes to `libcamera_dual_node.cpp`

#### 1. Add Frame Interleaving Logic
- Process cameras alternately, not in parallel
- Each camera captures one frame, then yields to the other
- Prevents ISP resource conflicts

#### 2. Increase Buffer Count
- Change from 4 to 6 buffers per camera
- Provides more headroom for processing delays

#### 3. Add Capture Coordination
- Use mutex to ensure only one camera captures at a time
- Add small delay (5-10ms) between camera switches

#### 4. Reduce Target FPS
- Current: 30 fps per camera (60 fps total load)
- Proposed: 15 fps per camera (30 fps total load)
- Still sufficient for YOLO detection at ~20ms inference

#### 5. Enhanced Error Recovery
- Full camera restart on 3 consecutive timeouts (not just soft restart)
- Add libcamera CameraManager restart as last resort

---

## Implementation Changes

### [MODIFY] [libcamera_dual_node.cpp](file:///home/pi/ros2_ws/src/csi_camera/src/libcamera_dual_node.cpp)

1. **Add time-multiplexing mutex**
   ```cpp
   std::mutex capture_mutex_;  // Ensure only one camera captures at a time
   std::atomic<int> active_camera_{0};  // Which camera should capture next
   ```

2. **Modify processRequest() to be sequential**
   ```cpp
   // Only process if it's this camera's turn
   if (active_camera_.load() != cam_id) {
       request->reuse(Request::ReuseBuffers);
       camera->queueRequest(request);
       return;
   }
   // Process frame...
   // Then switch to other camera
   active_camera_.store((cam_id + 1) % 2);
   ```

3. **Reduce default FPS to 15**
   ```cpp
   this->declare_parameter("fps", 15);  // Changed from 30
   ```

4. **Increase buffer count**
   ```cpp
   streamConfig.bufferCount = 6;  // Changed from 4
   ```

5. **Add consecutive failure counter for full restart**
   ```cpp
   std::atomic<int> consecutive_failures_[2] = {0, 0};
   // In watchdogCallback:
   if (consecutive_failures_[cam_id] >= 3) {
       fullRestart();  // Restart entire libcamera
   }
   ```

---

## Verification Plan

### Automated Tests
1. Run system for 30 minutes continuously
2. Monitor frame counts - should increase steadily
3. Check for any timeout errors in log
4. Verify YOLO inference continues working

### Acceptance Criteria
- No camera timeouts for 30+ minutes
- Frame rate stable at 15 fps per camera
- YOLO detection latency < 50ms
- No recovery attempts needed

---

## Alternative Solutions (if above doesn't work)

### Option B: Separate Process per Camera
- Run 2 separate ROS nodes, one per camera
- Complete process isolation
- Higher memory usage but guaranteed stability

### Option C: Use rpicam-still fallback
- Keep current `dual_csi_camera_node.cpp` (uses rpicam-still)
- Less efficient but proven stable
- Already implemented and working

> [!IMPORTANT]
> The current `dual_csi_camera_node.cpp` using `rpicam-still` subprocess is more stable but slower (~2-3 fps). If libcamera fixes don't work, recommend using this as production fallback.
