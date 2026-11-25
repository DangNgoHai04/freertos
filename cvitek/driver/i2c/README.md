# Driver Soft-I2C — Tầng 2 (RTOS Soft-I²C Bus Driver API)

Lớp bọc FreeRTOS an toàn đa luồng cho [HAL I2C5 Bitbang T0–T1](../../hal/cv181x/i2c/README.md), cung cấp API thanh ghi 16-bit Big-Endian cho các driver ngoại vi (như [Tiny1C](../../driver/sensor/tiny1C/README.md)).

---

## 1. Cấu trúc mã nguồn

| Tệp | Trách nhiệm |
| :--- | :--- |
| [include/soft_i2c.h](include/soft_i2c.h) | Khai báo `soft_i2c_bus_t` (chứa `bb_master_t`, `StaticSemaphore_t`, thống kê `transfers`/`errors`) và 5 hàm Public API `soft_i2c_*` tách biệt hoàn toàn với ABI `i2c.h` legacy. |
| [src/soft_i2c.c](src/soft_i2c.c) | Triển khai quản lý Static Mutex, ngân sách thời gian thực thống nhất `timeout_ms` và các hàm helper đọc/ghi thanh ghi 16-bit. |
| `src/i2c.c`, `include/i2c.h`, `CMakeLists.txt` | Driver I2C DW phần cứng legacy (không dùng trên nhánh Tiny1C `riscv64`). |

---

## 2. Cơ chế hoạt động & Hợp đồng API

1. **Khởi tạo & Mutex tĩnh (`soft_i2c_init`)**:
   - Caller cấp phát tĩnh `soft_i2c_bus_t` (zero-init) giữ sống suốt vòng đời firmware.
   - Tạo mutex bằng `xSemaphoreCreateMutexStatic(&b->lock_storage)` (không dùng Heap) và khởi tạo `bb_master_t` ở Tầng 1. Chỉ gọi ở ngữ cảnh Task (không gọi trong ISR).
2. **Ngân sách thời gian thực (`run`)**:
   - Nhận `timeout_ms` hợp lệ trong khoảng `1 .. 60000 ms`.
   - Quy đổi số tick chờ mutex làm tròn lên `((ms * configTICK_RATE_HZ + 999) / 1000)`; sau khi chiếm được `b->lock`, tính `elapsed` thực tế qua `now_us()`:
     - Nếu `elapsed >= budget`, trả ngay `BB_TIMEOUT` mà không chạm dây.
     - Nếu còn ngân sách, truyền `budget - elapsed` xuống `bb_transfer()` hoặc `bb_recover()`.
   - Cập nhật bộ đếm `b->transfers++` (và `b->errors++` nếu có lỗi) trước khi trả mutex.
3. **Helper Thanh ghi 16-bit (CCI Tiny1C)**:
   - `soft_i2c_read_reg16(bus, addr, reg, data, len, ms)`: Gửi 2 byte địa chỉ thanh ghi MSB-first (`reg >> 8`, `reg & 0xFF`), phát `Repeated START` và đọc `len` byte dữ liệu trong **cùng một lần giữ mutex**.
   - `soft_i2c_write_reg16(bus, addr, reg, data, len, ms)`: Hỗ trợ `1 <= len <= 256` byte, ghép `reg_hi`, `reg_lo` và dữ liệu vào mảng stack `buffer[258]` để phát trong một frame I²C duy nhất.
   - `soft_i2c_transfer()` & `soft_i2c_recover()`: Thực thi mảng `bb_msg_t` tùy biến (ví dụ probe địa chỉ `len = 0`) hoặc cứu hộ bus 9 xung SCL dưới mutex.
4. **Phối hợp khóa với Tầng 3**:
   - T2 chỉ giữ `b->lock` trong phạm vi **từng giao dịch I²C đơn lẻ**. Một lệnh VDCMD ở Tầng 3 gồm chuỗi *ghi lệnh (`0x9D00`/`0x1D00`) $\rightarrow$ poll `0x0200` $\rightarrow$ đọc kết quả*, do đó Tầng 3 ([vdcmd.c](../sensor/tiny1C/src/vdcmd.c)) giữ mutex thiết bị riêng bao trọn cả chuỗi để tránh xen lệnh giữa hai task.

---

## 3. Tích hợp Build & Kiểm thử

- Trong firmware C906L (`riscv64`), [soft_i2c.c](src/soft_i2c.c) được biên dịch trực tiếp vào thư viện `thermal_tiny1c` qua [tiny1C/CMakeLists.txt](../sensor/tiny1C/CMakeLists.txt) khi bật `-DTHERMAL_TINY1C=ON`.
- Chạy host test (T1 + T2) và cross-compile RISC-V với header FreeRTOS thật:
  ```bash
  bash cvitek/hal/cv181x/i2c/tests/run.sh
  bash cvitek/hal/cv181x/i2c/tests/compile-riscv.sh
  ```

Liên kết: [T0 & T1 — HAL I2C5 Bitbang](../../hal/cv181x/i2c/README.md) · [T3 — Driver Tiny1C](../sensor/tiny1C/README.md)
