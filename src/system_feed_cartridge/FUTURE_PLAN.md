# Kế Hoạch Phát Triển Hệ Thống Cấp Khay (Future Plan)
> File này lưu các định hướng và kế hoạch cần thực hiện trong tương lai.
> Khi muốn thực hiện: kêu AI đọc file này và implement.

---

## 1. Bỏ Hoàn Toàn Sensor Simulation

**Mục tiêu:** Xóa toàn bộ `_sim_sensors` và logic simulation, cả 2 mode đều chỉ đọc sensor thực.

**Việc cần làm:**
- Xóa `self._sim_sensors: dict = {}` khỏi `__init__`
- Xóa method `sensor()` hiện tại, đổi tên `_sensor_raw()` thành `sensor()`
- Xóa `sensor_real()` (không cần nữa vì sensor() = real value)
- Xóa callback `_cb_sim_sensor()` trong node
- Xóa subscriber `sub_sim_sensor` và publisher tương ứng
- Xóa toàn bộ `_sim_sensors.clear()` trong `_cb_mode_change()` và `_cb_start()`
- Xóa UI sim sensor trong GUI (CartridgePage.qml hoặc cartridge_gui.py):
  - Nút "SIM" hoặc ô nhập sim sensor
  - Topic publisher `/revpi/sim_sensor`
- Xóa block `if self.operation_mode != 'manual':` guard trong `_cb_sim_sensor`

**Files cần sửa:**
- `scripts/cartridge_providesystem_py_node.py`
- `scripts/cartridge_gui.py`
- `qml/CartridgePage.qml` (nếu có nút sim)

---

## 2. Đồng Nhất Workflow Cả 2 Mode

**Mục tiêu:** Manual và Auto có cùng cơ chế state machine, chỉ khác ở điểm trigger thay khay.

**Điểm khác biệt duy nhất:**
- **Manual mode**: Operator có thể gửi topic `change_tray_input` để trigger thay khay thủ công
- **Auto mode**: Robot tự gửi `robot_done` khi hoàn thành row 5

**Việc cần làm:**
- Đảm bảo `_cb_start()` trong cả 2 mode đều init state machine giống nhau
- State 1 entry: bỏ mọi check mode-specific, chỉ check điều kiện hardware (S12, homing)
- State 2 entry: chỉ check S13 ON (không phân biệt manual/auto)
- Manual: thêm nút "Thay khay" trên GUI → publish `change_tray_input` → set `_robot_done = True`

**Files cần sửa:**
- `scripts/cartridge_providesystem_py_node.py`
- `qml/CartridgePage.qml` (thêm nút "Thay khay thủ công")

---

## 3. Ignore Tín Hiệu Pos1 Khi Chưa Chọn State 1

**Mục tiêu:** Nếu sensor S1/S2/S3/S4/S5 thỏa điều kiện nhưng chưa nhấn State 1 → hệ thống bỏ qua, không tự vào State 1.

**Logic cần implement:**
```python
# Trong _do_idle() AUTO mode:
# Chỉ tự vào State 1 khi _state1_enabled = True
# _state1_enabled được set khi operator nhấn STATE 1 từ GUI

# Thêm flag:
self._state1_enabled = False  # Phải nhấn STATE 1 từ GUI để enable

# Trong _cb_goto_state() khi nhận 'STATE1':
self._state1_enabled = True
self._enter(SystemState.S1_CONFIRM_SAFE)

# Trong _do_idle() AUTO:
if has_tray and s12_on and homed and self._state1_enabled:
    self._enter(SystemState.S1_CONFIRM_SAFE)
    
# Reset sau khi State 1 complete:
# self._state1_enabled = False  (hoặc giữ True để loop liên tục trong auto)
```

**Behavior:**
- **Auto mode**: Sau khi nhấn START → `_state1_enabled = True`, loop tự chạy
- **Manual mode**: Phải nhấn STATE 1 mỗi lần muốn cấp khay (hoặc dùng auto loop nếu muốn)

---

## 4. Tăng move_timeout (Nhanh, Low Risk)

**Mục tiêu:** Giảm bớt lỗi timeout di chuyển servo.

```yaml
# cartridge_config.yaml
move_timeout: 60.0   # tăng từ 25.0 → 60.0
```

**Áp dụng ngay không cần code change.**

---

## 5. Chuyển Đổi Remaining _error() → Retry (Middle Risk)

Còn ~20 điểm timeout di chuyển ở State 1 và State 2 vẫn dùng `_error()`.
Xem file: `/home/pi/.gemini/antigravity/brain/09f4caf1-0ff0-4a9a-ab16-959d0a3a47e1/error_review.md`

**Danh sách cần sửa (State 1):**
- S1 Step8: INY → 50mm timeout (dòng ~1443)
- S1 Step10: INY → 200mm timeout (dòng ~1466, 1473)
- S1 Step12: INY → 50mm timeout (dòng ~1507, 1513)
- S1 Step13: INX → 20mm timeout (dòng ~1527, 1533)

**Danh sách cần sửa (State 2):**
- S2A A2: INX timeout
- S2A A3: INY timeout
- S2A A5: INY timeout
- S2A A6: INX timeout
- S2A A8: INY timeout
- S2A A10: INY timeout
- S2A A11: INX timeout

**Pattern sửa (giống đã làm với các điểm rủi ro cao):**
```python
# Thay:
if time.time() > self._step_timeout:
    self._error("... timeout")
# Bằng:
if time.time() > self._step_timeout:
    self.get_logger().warn("⚠️ ... timeout → retry move")
    self._cmd_sent = False  # reset để retry
```
