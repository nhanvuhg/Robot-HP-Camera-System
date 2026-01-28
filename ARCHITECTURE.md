# Dual CSI Camera System với libcamera C++ API

## Tổng quan kiến trúc

Hệ thống camera kép sử dụng **libcamera C++ API** trực tiếp trên Raspberry Pi 5, đạt **30+ FPS** thay vì phương pháp `rpicam-still` cũ chỉ đạt 2-3 FPS.

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         RASPBERRY PI 5                                  │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  ┌──────────────┐    ┌──────────────┐                                   │
│  │   CSI Port 0  │    │   CSI Port 1  │   ← Hardware (2 CSI ports)      │
│  │   (Camera 0)  │    │   (Camera 1)  │                                 │
│  └──────┬───────┘    └──────┬───────┘                                   │
│         │                    │                                          │
│         ▼                    ▼                                          │
│  ┌──────────────────────────────────────┐                               │
│  │     libcamera_dual_node.cpp          │  ← libcamera C++ API          │
│  │  - CameraManager (enumerate cameras) │                               │
│  │  - Streaming callbacks               │                               │
│  │  - mmap zero-copy buffers            │                               │
│  │  - RGB→BGR color conversion          │                               │
│  │  - FPS limiter (configurable)        │                               │
│  └──────────────┬───────────────────────┘                               │
│                 │                                                       │
│    ┌────────────┴────────────┐                                          │
│    ▼                         ▼                                          │
│  /cam0HP/image_raw     /cam1HP/image_raw   ← ROS2 Topics (sensor_msgs)  │
│    │                         │                                          │
│    ▼                         ▼                                          │
│  ┌─────────────────────────────────────┐                                │
│  │   YOLO Nodes (Hailo-8L Accelerator) │  ← AI Inference (~20ms)        │
│  │   - yolo_cam0: Process cam0 frames  │                                │
│  │   - yolo_cam1: Process cam1 frames  │                                │
│  └──────────────┬──────────────────────┘                                │
│                 │                                                       │
│    ┌────────────┴────────────┐                                          │
│    ▼                         ▼                                          │
│  /cam0HP/yolo/bounding_boxes  /cam1HP/yolo/bounding_boxes               │
│    │                         │                                          │
│    ▼                         ▼                                          │
│  ┌─────────────────────────────────────┐                                │
│  │   bbox_drawer_node (Overlay)        │  ← Draw bounding boxes         │
│  └──────────────┬──────────────────────┘                                │
│                 │                                                       │
│    ┌────────────┴────────────┐                                          │
│    ▼                         ▼                                          │
│  /cam0HP/image_overlay  /cam1HP/image_overlay  ← Final output           │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

## Files quan trọng

| File | Mô tả |
|------|-------|
| `src/csi_camera/src/libcamera_dual_node.cpp` | **Camera node chính** - libcamera C++ API |
| `src/csi_camera/src/dual_csi_camera_node.cpp` | Node cũ (rpicam-still) - backup |
| `src/csi_camera/CMakeLists.txt` | Build configuration |
| `run_all_three_v2.sh` | Script khởi động toàn bộ hệ thống |
| `stop_all.sh` | Script dừng tất cả processes |

## Cách hoạt động chi tiết

### 1. libcamera C++ API Flow

```cpp
// 1. Khởi tạo CameraManager
CameraManager cm;
cm.start();

// 2. Lấy danh sách cameras
auto cameras = cm.cameras();  // Phát hiện 2 cameras

// 3. Acquire camera (chiếm quyền sử dụng)
camera0 = cm.get(cameras[0]->id());
camera0->acquire();

// 4. Cấu hình stream
config = camera->generateConfiguration({StreamRole::VideoRecording});
config->at(0).size = {640, 480};
config->at(0).pixelFormat = formats::BGR888;
camera->configure(config.get());

// 5. Allocate buffers
FrameBufferAllocator allocator(camera);
allocator.allocate(stream);

// 6. Kết nối callback cho frame completed
camera->requestCompleted.connect(this, &processRequest);

// 7. Start streaming
camera->start();
for (auto& request : requests) {
    camera->queueRequest(request.get());
}

// 8. Trong callback processRequest():
void processRequest(Request* request, int cam_id) {
    // mmap buffer (zero-copy access)
    void* data = mmap(nullptr, plane.length, PROT_READ, MAP_SHARED,
                      plane.fd.get(), plane.offset);
    
    // Convert RGB→BGR (libcamera BGR888 thực ra là RGB)
    cv::Mat frame(height, width, CV_8UC3, data);
    cv::cvtColor(frame, frame_safe, cv::COLOR_RGB2BGR);
    
    munmap(data, plane.length);
    
    // Publish ROS message
    sensor_msgs::msg::Image msg;
    msg.data.assign(frame_safe.data, frame_safe.data + total_bytes);
    publisher->publish(msg);
    
    // Requeue request cho frame tiếp theo
    request->reuse(Request::ReuseBuffers);
    camera->queueRequest(request);
}
```

### 2. Các vấn đề đã giải quyết

| Vấn đề | Giải pháp |
|--------|-----------|
| `rpicam-vid` bị stall | Dùng libcamera C++ API trực tiếp |
| FPS thấp (2-3 FPS) | Streaming callbacks → 30+ FPS |
| Color sai (xanh→vàng) | `cv::cvtColor(COLOR_RGB2BGR)` |
| Crash khi exit | Disconnect signals trước cleanup |
| Race condition mmap | Mutex lock trong processRequest |
| "Pipeline in use" | Force kill processes với `pkill -9` |

### 3. Performance đạt được

| Metric | Old (rpicam-still) | New (libcamera API) |
|--------|-------------------|---------------------|
| FPS | ~2-3 | ~30+ |
| Latency | ~300ms | ~16ms |
| CPU Usage | ~25% | ~8% |
| Method | Subprocess spawn | Direct API streaming |

## Cách sử dụng

### Khởi động hệ thống
```bash
cd ~/ros2_ws
./run_all_three_v2.sh
```

### Dừng hệ thống
```bash
./stop_all.sh
# hoặc nhấn Ctrl+C trong terminal đang chạy run_all_three_v2.sh
```

### Kiểm tra FPS
```bash
ros2 topic hz /cam0HP/image_raw
ros2 topic hz /cam1HP/image_raw
```

### Xem camera status
```bash
ros2 topic echo /camera/status
```

## Troubleshooting

### Lỗi "Pipeline handler in use by another process"
```bash
pkill -9 libcamera_dual
pkill -9 rpicam
~/ros2_ws/stop_all.sh
# Sau đó chạy lại
```

### Lỗi "Camera frontend has timed out"
- Đây là lỗi **HARDWARE** (cáp lỏng)
- Tắt Pi, kiểm tra cáp ribbon
- Thử cáp khác

### Build lại sau khi sửa code
```bash
cd ~/ros2_ws
colcon build --packages-select csi_camera --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

---

## Auto-Recovery Feature (NEW)

### Tính năng tự động phục hồi

Khi camera timeout (không có frame trong N giây), hệ thống sẽ tự động:
1. Phát hiện camera bị stuck
2. Dừng và restart camera đó
3. Tiếp tục streaming bình thường

### Cách hoạt động

```cpp
// Watchdog timer chạy mỗi 2 giây
void watchdogCallback() {
    auto elapsed = now - last_successful_frame_[cam_id];
    if (elapsed > timeout) {
        restartCamera(cam_id);  // Tự động restart
    }
}

// Restart camera
void restartCamera(int cam_id) {
    camera->requestCompleted.disconnect(...);
    camera->stop();
    std::this_thread::sleep_for(500ms);
    camera->requestCompleted.connect(...);
    camera->start();
    // ✅ Camera recovered!
}
```

### Parameters

| Parameter | Default | Mô tả |
|-----------|---------|-------|
| `watchdog_timeout_sec` | 5 | Thời gian chờ trước khi restart |

### Ví dụ log khi recovery

```
⚠️ Camera 0 stuck! No frames for 6 seconds. Attempting recovery...
🔄 Restarting camera 0...
✅ Camera 0 recovered!
📊 Cam0: 20755 frames, Cam1: 21000 frames  ← Tiếp tục chạy
```

### Lợi ích
- **24/7 Operation**: Chạy liên tục không cần can thiệp
- **Tự phục hồi**: Xử lý timeout tự động
- **Không mất dữ liệu**: Camera kia vẫn tiếp tục hoạt động
