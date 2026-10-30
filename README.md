# Robot Dò Line ESP32 - Bám Line PID & Điều Khiển BLE

Dự án xe/robot dò line tốc độ cao sử dụng vi điều khiển **ESP32**, thuật toán **bám line PD**, xử lý góc vuông 90°, vạch ngắt quãng (nét đứt) và tích hợp điều khiển / giám sát từ xa qua **Bluetooth Low Energy (BLE)**.

---

## 1. Cấu hình phần cứng

* **Vi điều khiển:** ESP32 DevKit (DOIT ESP32 DEVKIT V1)
* **Mạch điều khiển động cơ:** TB6612FNG (2 kênh PWM)
  * Động cơ Trái: `PWM: 25`, `IN1: 26`, `IN2: 27`
  * Động cơ Phải: `PWM: 14`, `IN1: 12`, `IN2: 13`
* **Cảm biến dò line:** Thanh 8 mắt QTR qua IC dồn kênh CD74HC4067 (12-bit ADC)
  * 3 chân chọn kênh: `S0: 18`, `S1: 19`, `S2: 23`
  * Chân tín hiệu ADC: `SIG: 32`

---

## 2. Tính năng chính

* **Tự động cân chỉnh (Calibration) & Lưu EEPROM:** Tự quét trắng/đen trong 5s và lưu vào Flash của ESP32.
* **Bám line vi sai PD:** Tính toán vị trí trọng số (Weighted Average) từ 0 – 7000 (tâm 3500) kết hợp cơ chế BOOST khi góc cua gắt.
* **Xử lý sa hình nâng cao:**
  * Cua góc vuông 90° bằng tốc độ chuyên biệt `TURN_OUT` và `TURN_IN`.
  * Vượt vạch ngắt quãng (nét đứt) mà không bị mất lái.
  * Tự tìm lại line khi văng ra ngoài.
  * Tự động phanh điện tử hãm cứng (Short Brake) khi gặp vạch kết thúc (toàn đen).
* **Kết nối BLE Telemetry & Điều khiển:** Tên thiết bị quảng bá: `ROBOT_BLE`. Gửi dữ liệu trạng thái dạng JSON mỗi 150ms.

---

## 3. Tập lệnh điều khiển qua BLE

| Lệnh BLE | Chức năng |
| :--- | :--- |
| `CAL` | Bắt đầu cân chỉnh cảm biến trong 5s và lưu Flash |
| `AUTO` | Bắt đầu chạy tự động dò line theo PID + PROF |
| `MANUAL` | Chuyển sang chế độ lái tay |
| `FWD`, `BWD`, `LFT`, `RGT`, `STP` | Tiến / Lùi / Trái / Phải / Dừng (Manual) |
| `SPD:<value>` | Cài đặt tốc độ lái tay (30 – 255) |
| `PID:<kp>,<kd>,<ki>,<base>` | Cập nhật PID và tốc độ tối đa (Mặc định: `0.04,0.4,0.00005,150`) |
| `PROF:<min>,<e_slow>` | Tốc độ theo đường: giảm tốc khi vào cua (Mặc định: `110,1800`) |
| `TURN:<out>,<in>,<base>` | Cập nhật cua vuông 90° (Mặc định: `60,60,120`) |
| `GETALL` / `GETPID` | Đọc tất cả thông số PID, PROF, TURN từ robot |

---

## 4. Biên dịch và nạp code (PlatformIO)

```bash
# Biên dịch
pio run

# Nạp code vào ESP32 qua cáp USB
pio run -t upload

# Mở Serial Monitor xem log (115200 baud)
pio device monitor
```
