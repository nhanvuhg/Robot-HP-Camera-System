# Feeder Sensor Mapping

Hệ thống sử dụng **15 digital sensor** kết nối qua Festo CPX-AP IO module. Tất cả cấu hình **PNP, Normally Open (NO)**. Cập nhật: 2026-03-17.

---

## Sensor Quang & Logic (S1–S11)

| ID | Tên (Code) | Vị trí | Chức năng |
|----|------------|---------|-----------|
| **S1** | S1_CONVEYOR_START | Pos 1 | Đầu băng tải đầu vào — phát hiện khay ở điểm bắt đầu |
| **S2** | S2_CONVEYOR_MID | Pos 1 | Giữa băng tải đầu vào — phát hiện khay đang di chuyển |
| **S3** | S3_CONVEYOR_END | Pos 1 | Cuối băng tải đầu vào — xác nhận khay đến đúng vị trí nhận |
| **S4** | S4_SCAN_STACK_P1 | Pos 1 | Quét khay xếp chồng — InY đi xuống tìm khay đầu tiên |
| **S5** | S5_OUTPUT_DETECT | Pos 1 | Phát hiện khay ở khu vực đặt đầu ra — robot đặt khay xong |
| **S6** | S6_CHECK_TRAY_OUTP1 | Pos 1 | Kiểm tra khay tại khay chứa (Output P1) — INY chạm khay khi tìm kiếm xuống |
| **S7** | S7_PLATFORM_TRAY | Pos 2 | Phát hiện có khay trên Platform (Servo 3) — khay đang ở vị trí nạp |
| **S8** | S8_FEED_SUCCESS | Pos 2 | Cấp khay thành công — xác nhận khay đã được đưa vào đúng vị trí |
| **S9** | S9_CHECK_TRAY_OUTP2 | Pos 2 | Kiểm tra khay tại khay chứa (Output P2) — OUTY quét khay khi tìm kiếm xuống |
| **S10** | S10_SCAN_STACK_P2 | Pos 2 | Quét khay xếp chồng — OUTY đi xuống tìm khay đầu tiên tương tự S4 |
| **S11** | S11_TRAY_AT_ROBOT | Pos 1/2 | Cảm biến có khay ở vị trí robot thực thi (thay thế chức năng của S18 cũ) |

---

## S12–S14 ← **DỰ PHÒNG** (chưa đấu nối)

---

## Sensor Cylinder / Limit Switch (S15–S20)

> Tất cả đều là **hardware sensor**. Logic interlock trong code chỉ để bảo vệ cơ khí (ngăn chuyển động sai thứ tự gây hư hỏng).

| ID | Tên (Code) | Vị trí | Chức năng | Interlock |
|----|------------|---------|-----------|-----------|
| **S15** | S15_CYL1_RETRACTED | Pos 1 | Cylinder 1 đã co lại hoàn toàn | Chặn INX nếu OFF (State 2) |
| **S16** | S16_CYL1_EXTENDED | Pos 1 | Cylinder 1 đã đẩy ra hoàn toàn | Chặn INY đi xuống nếu ON |
| **S19** | S19_CYL2_RETRACTED | Pos 2 | Cylinder 2 (đầu ra) đã co lại hoàn toàn | — |
| **S20** | S20_CYL2_EXTENDED | Pos 2 | Cylinder 2 (đầu ra) đã đẩy ra hoàn toàn | — |

---

## Kênh Output Cylinder (Digital Output)

| Cylinder | Extend DO | Retract DO | Vị trí |
|----------|-----------|------------|--------|
| Cylinder 1 | 5 | 4 | Pos 1 — Input Pickup |
| Cylinder 2 | 7 | 6 | Pos 2 — Output Pickup |
