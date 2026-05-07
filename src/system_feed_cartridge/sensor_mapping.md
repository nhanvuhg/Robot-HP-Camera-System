# Feeder Sensor Mapping

Hệ thống sử dụng **22 digital sensor** kết nối qua 2 module Festo CPX-AP IO.
- **CPX 253** (`io_ip = 192.168.27.253`): Module 2 (I1.0–I1.7) + Module 3 (I2.0–I2.7) — 16 kênh DI
- **CPX 254** (`io_ip_2 = 192.168.27.254`): Module 2 (I3.0–I3.7) — 8 kênh DI

Tất cả cấu hình **PNP, Normally Open (NO)**. Cập nhật: 2026-05-04.

Bảng mapping phản ánh thứ tự đọc thực tế trong code: `cache1[sid-1]` cho S1–S16, `cache2[sid-17]` cho S17–S22.

---

## CPX 253 — Module 2 (I1.0–I1.7): S1–S8

| Logic ID | Tên (Code) / Chức năng | Vị trí | Index cache |
|----------|------------------------|---------|-------------|
| **S1** | S1_BELT_START — phát hiện khay ở đầu băng tải | Pos 1 | cache1[0] |
| **S2** | S2_BELT_MID — phát hiện khay đang di chuyển | Pos 1 | cache1[1] |
| **S3** | S3_BELT_END — xác nhận khay đến cuối băng tải | Pos 1 | cache1[2] |
| **S4** | S4_SCAN_STACK_P1 — quét khay xếp chồng (InY xuống) | Pos 1 | cache1[3] |
| **S5** | S5_OUTPUT_DETECT — phát hiện khay ở khu vực đặt đầu ra | Pos 1 | cache1[4] |
| **S6** | S6_CHECK_TRAY_OUTP1 — INY chạm khay khi tìm kiếm xuống | Pos 1 | cache1[5] |
| **S7** | S7_TRAY_AT_ROBOT — xác nhận khay đã lên vị trí robot | Pos 1 | cache1[6] |
| **S8** | S8_RESERVED — dự phòng | — | cache1[7] |

---

## CPX 253 — Module 3 (I2.0–I2.7): S9–S16

| Logic ID | Tên (Code) / Chức năng | Vị trí | Index cache |
|----------|------------------------|---------|-------------|
| **S9**  | S9_CYL1_RETRACTED — Cylinder 1 đã co lại hoàn toàn | Pos 1 | cache1[8] |
| **S10** | S10_CYL1_EXTENDED — Cylinder 1 đã đẩy ra hoàn toàn | Pos 1 | cache1[9] |
| **S11** | S11_ATV_RUN — trạng thái ATV Run (VFD monitor) | VFD | cache1[10] |
| **S12** | S12_ATV_FAULT — trạng thái ATV Fault (VFD monitor) | VFD | cache1[11] |
| **S13** | S13_RESERVED — dự phòng | — | cache1[12] |
| **S14** | S14_RESERVED — dự phòng | — | cache1[13] |
| **S15** | S15_RESERVED — dự phòng | — | cache1[14] |
| **S16** | S16_RESERVED — dự phòng | — | cache1[15] |

---

## CPX 254 — Module 2 (I3.0–I3.7): S17–S22

| Logic ID | Tên (Code) / Chức năng | Vị trí | Index cache |
|----------|------------------------|---------|-------------|
| **S17** | S17_PLATFORM — khay đã lên Platform Servo 3 | Pos 2 | cache2[0] |
| **S18** | S18_FEED_OK — xác nhận cấp khay thành công | Pos 2 | cache2[1] |
| **S19** | S19_CHECK_TRAY_P2 — kiểm tra có khay tại Output P2 | Pos 2 | cache2[2] |
| **S20** | S20_SCAN_STACK_P2 — OUTY quét phát hiện cạnh khay | Pos 2 | cache2[3] |
| **S21** | S21_CYL2_RETRACTED — Cylinder 2 đã co lại | Pos 2 | cache2[4] |
| **S22** | S22_CYL2_EXTENDED — Cylinder 2 đã đẩy ra | Pos 2 | cache2[5] |

---

## Kênh Output Cylinder (Digital Output — trên CPX 253)

| Cylinder | Extend DO | Retract DO | Vị trí | Config key |
|----------|-----------|------------|--------|------------|
| Cylinder 1 | **5** | **4** | Pos 1 — Input Pickup | `cylinder1_extend_channel` |
| Cylinder 2 | **9** | **8** | Pos 2 — Output Pickup | `cylinder2_extend_channel` |

> ⚠️ Lưu ý: Các giá trị kênh DO được load từ `cartridge_config.yaml` (không phải `sensors.yaml`).
> `sensors.yaml` chỉ là tài liệu tham khảo, **không được node load trực tiếp**.

---

## Sử dụng sensor trong State Machine

| Sensor | Dùng trong | Vai trò |
|--------|-----------|---------|
| S1, S2, S3 | `_can_start_s1`, IDLE trigger | Phát hiện khay trên băng tải |
| S4 | S1_INY_SCAN | Phát hiện cạnh khay khi InY quét xuống |
| S5 | S1_CHECK_S5 | Xác nhận khay vào đúng vị trí nạp |
| S6 | S2A_CHECK_INTERLOCK | Snapshot có khay trước khi STATE 2A |
| S7 | `_can_start_s2a`, S1_WAIT_S7 | Khay đã lên vị trí robot |
| S9 | `_can_start_s1` (`cyl1_ret_ok`) | CYL1 phải retracted trước khi S1 |
| S10 | `_can_start_s1` (`cyl1_ext_ok`) | CYL1 phải không extended trước khi S1 |
| S17 | S3_CHECK_S17, S3_WAIT_S17, S4 | Khay có trên Platform Servo 3 |
| S18 | `_can_start_s3`, `_can_start_s4`, S3_SERVO3_FEED | Feed OK / tín hiệu sẵn sàng P2 |
| S19 | S4_CHECK_S11 | Có khay tại stack P2 (bỏ scan nếu OFF) |
| S20 | S4_OUTY_SCAN_S12 | Phát hiện cạnh khay khi OUTY quét |
| S21 | S4_CYL2_EXTEND (wait retracted) | CYL2 phải retracted trước khi extend |
| S22 | S4_CYL2_EXTEND (confirm extended) | CYL2 đã extend xong |
