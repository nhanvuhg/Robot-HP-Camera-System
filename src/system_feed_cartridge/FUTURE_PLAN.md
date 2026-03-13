# Kế Hoạch Phát Triển Hệ Thống Cấp Khay (Future Plan)
> Khi muốn thực hiện: kêu AI đọc file này và implement.

---

## Plan 1: Bỏ Hoàn Toàn Sensor Simulation *(chờ thực hiện)*

**Mục tiêu:** Xóa toàn bộ `_sim_sensors` và logic simulation. Cả 2 mode chỉ đọc sensor thực.

**Việc cần làm:**
- Xóa `self._sim_sensors: dict = {}` khỏi `__init__`
- Xóa `_sensor_raw()`, `sensor_real()`, đổi `sensor()` đọc thẳng hardware
- Xóa callback `_cb_sim()` và subscriber `/providesystem/sim_sensor`
- Xóa `_sim_sensors.clear()` trong `_cb_stop()` và `_cb_mode_change()`
- Xóa UI sim sensor trong GUI (`cartridge_gui.py`, `CartridgePage.qml`)
