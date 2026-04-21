# Feeder Sensor Mapping

Hệ thống sử dụng **16 digital sensor** kết nối qua Festo CPX-AP IO module (Gồm 2 Module, mỗi module 8 kênh). Tất cả cấu hình **PNP, Normally Open (NO)**. Cập nhật: 2026-04-15.

Bảng mapping dưới đây mô tả chính xác cả ID logic (S1-S16) được sử dụng trên GUI / Backend, hoàn toàn trùng khớp với thứ tự kết nối vật lý trên các trạm Module I/O.

---

## Module 1 (8 Kênh Đầu) - Sensor Quang & Logic

| Logic ID | Tên (Code) / Chức năng | Vị trí | Cổng Vật Lý |
|----|------------|---------|-----------|
| **S1** | S1_CONVEYOR_START (Phát hiện khay ở điểm bắt đầu) | Pos 1 | Module 1 - Index 0 |
| **S2** | S2_CONVEYOR_MID (Phát hiện khay đang di chuyển) | Pos 1 | Module 1 - Index 1 |
| **S3** | S3_CONVEYOR_END (Xác nhận khay đến đúng vị trí nhận) | Pos 1 | Module 1 - Index 2 |
| **S4** | S4_SCAN_STACK_P1 (Quét khay xếp chồng — InY đi xuống tìm khay) | Pos 1 | Module 1 - Index 3 |
| **S5** | S5_OUTPUT_DETECT (Phát hiện khay ở khu vực đặt đầu ra) | Pos 1 | Module 1 - Index 4 |
| **S6** | S6_CHECK_TRAY_OUTP1 (INY chạm khay khi tìm kiếm xuống) | Pos 1 | Module 1 - Index 5 |
| **S7** | S7_TRAY_AT_ROBOT (Cảm biến có khay ở vị trí robot thực thi, thay thế S11 cũ) | Pos 1/2 | Module 1 - Index 6 |
| **S8** | S8_RESERVED (Dự phòng) | — | Module 1 - Index 7 |

---

## Module 2 (8 Kênh Sau) - Sensor Platform & Cylinder
> Lưu ý: Các cảm biến khí nén từ S13-S16 là cứng (Hardware Sensor), các khóa an toàn Interlock trong code ngăn chuyển động sai thứ tự gây hư hỏng cơ khí.

| Logic ID | Tên (Code) / Chức năng | Vị trí | Cổng Vật Lý |
|----|------------|---------|-----------|
| **S9** | S9_PLATFORM_TRAY (Khay trên Platform Servo 3, thay thế S7 cũ) | Pos 2 | Module 2 - Index 0 |
| **S10** | S10_FEED_SUCCESS (Cấp khay thành công vào đúng vị trí, thay thế S8 cũ) | Pos 2 | Module 2 - Index 1 |
| **S11** | S11_CHECK_TRAY_OUTP2 (Kiểm tra khay tại Output P2, quét OUTY, thay S9 cũ) | Pos 2 | Module 2 - Index 2 |
| **S12** | S12_SCAN_STACK_P2 (Quét khay xếp chồng — OUTY đi xuống, thay S10 cũ) | Pos 2 | Module 2 - Index 3 |
| **S13** | S13_CYL1_RETRACTED (Cyl 1 co lại, thay S15 cũ - Chặn INX nếu OFF) | Pos 1 | Module 2 - Index 4 |
| **S14** | S14_CYL1_EXTENDED (Cyl 1 đẩy ra, thay S16 cũ - Chặn INY xuống nếu ON) | Pos 1 | Module 2 - Index 5 |
| **S15** | S15_CYL2_RETRACTED (Cyl 2 co lại, thay S19 cũ) | Pos 2 | Module 2 - Index 6 |
| **S16** | S16_CYL2_EXTENDED (Cyl 2 đẩy ra, thay S20 cũ) | Pos 2 | Module 2 - Index 7 |

---

## Kênh Output Cylinder (Digital Output)

| Cylinder | Extend DO | Retract DO | Vị trí |
|----------|-----------|------------|--------|
| Cylinder 1 | 5 | 4 | Pos 1 — Input Pickup |
| Cylinder 2 | 7 | 6 | Pos 2 — Output Pickup |
