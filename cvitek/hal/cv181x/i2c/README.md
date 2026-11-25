# HAL I2C5 Bitbang — Tầng 0 (GPIO Platform) & Tầng 1 (Bitbang Controller HAL)

Tài liệu kỹ thuật cho hai tầng phần cứng của nhánh **Soft-I²C (Bus logic ID 5)** trên **SOPHGO SG2002 / CV181x** (nhân RISC-V C906L chạy FreeRTOS, bo mạch LicheeRV-Nano).

---

## 1. Cấu trúc mã nguồn

| Tầng | Tệp | Trách nhiệm chính |
| :--- | :--- | :--- |
| **T0 — GPIO Platform** | [hal_i2c_gpio_platform.c](src/hal_i2c_gpio_platform.c), [hal_i2c_gpio_platform.h](include/hal_i2c_gpio_platform.h) | Cấu hình `GPIOA15` (`SCL`) & `GPIOA27` (`SDA`), Open-Drain qua `GPIOA_DDR`, kéo lên nội IOBLK (`PAD_PU`), kiểm tra trạng thái idle HIGH sau `50 µs` và cung cấp timebase `rdtime`. |
| **T1 — Bitbang HAL** | [hal_i2c_bitbang.c](src/hal_i2c_bitbang.c), [hal_i2c_bitbang.h](include/hal_i2c_bitbang.h) | Máy trạng thái giao thức I²C Master thuần phần mềm (`1–100 kHz`): `START`, `Repeated START`, `STOP`, MSB-first byte, ACK/NACK, Clock Stretching, phát hiện Arbitration Loss, cứu hộ 9 xung và `nack_phase`. |
| **Legacy DW I2C** | `src/hal_dw_i2c.c`, `include/hal_dw_i2c.h` | Driver DW I2C cứng cũ (hiện không biên dịch trong [hal/cv181x/CMakeLists.txt](../CMakeLists.txt) cho `riscv64`). |

---

## 2. Tầng 0 (GPIO Platform Layer — `GPIOA15` / `GPIOA27`)

- **Chân vật lý trên LicheeRV-Nano**:
  - `SCL`: `GPIOA15` (`PAD_SPK_EN`, FMUX `0x0300103C` = Func `3`, IOBLK `0x03001908`).
  - `SDA`: `GPIOA27` (`PAD_EMMC_DAT3`, FMUX `0x03001058` = Func `3`, IOBLK `0x03001924`).
- **Trình tự khởi tạo an toàn `bb_gpio_platform_init(io, timer_hz, ownership_confirmed)`**:
  1. Kiểm tra tham số (`io`, `timer_hz > 0`), quyền sở hữu `ownership_confirmed == true` (cam kết phối hợp toàn bank `GPIOA` giữa Linux và FreeRTOS) và cờ singleton `claimed`.
  2. Trong `taskENTER_CRITICAL()`: mở clock `clk_apb_gpio` (`0x03002000` bit `29`), nhả `GPIOA_DDR` (`0x03020004`) về `0` (Hi-Z) và đặt `GPIOA_DR` (`0x03020000`) bằng `0` trước khi đổi mux (glitch-free), bật pull-up pad `PAD_PU` (`1U << 2`, xóa `PAD_PD`) trên `SPK_EN_IOBLK` & `EMMC_DAT3_IOBLK`, và đặt FMUX cả hai chân sang Func `3`.
  3. Đọc kiểm chứng (readback) FMUX, DDR và IOBLK; chờ `50 µs` rồi kiểm tra mức thực tế qua `GPIOA_EXT` (`0x03020050`). Nếu một trong hai dây vẫn LOW, in `[T0-I2C] Idle line LOW: SCL=... SDA=...` và trả `BB_NOT_READY`.
- **Giả lập Open-Drain & Timing**:
  - `drive(ctx, scl, low)`: thao tác RMW trên `GPIOA_DDR` được bọc `taskENTER_CRITICAL()`/`taskEXIT_CRITICAL()`; `DR` luôn giữ `0` để không bao giờ phát xung Push-Pull HIGH.
  - `sample(ctx, scl)`: đọc mức thực tế từ `GPIOA_EXT`.
  - `time_us()` / `delay_us()`: đọc trực tiếp thanh ghi RISC-V `rdtime` (mặc định `25 MHz` trên SG2002).

---

## 3. Tầng 1 (Bitbang Controller HAL)

Tầng 1 **không** phụ thuộc FreeRTOS hay địa chỉ MMIO của SG2002; toàn bộ giao tiếp đi qua `bb_io_t`:

- **Tần số & Timing**: `bb_init()` nhận `hz` trong dải `1,000 .. 100,000 Hz`, tính `half_us = (500000U + hz - 1) / hz` (làm tròn lên để không vượt tốc độ yêu cầu; ví dụ `100 kHz` $\rightarrow$ `5 µs`, `20 kHz` $\rightarrow$ `25 µs`).
- **Giao thức & Bắt lỗi (`bb_transfer`)**:
  - Hỗ trợ chuỗi `bb_msg_t` (tạo `Repeated START` giữa các message, `STOP` chỉ ở cuối mảng), chấp nhận địa chỉ 7-bit (`0x00..0x7F`; với Tiny1C là `0x3C`) và ghi `len == 0` để probe/scan địa chỉ.
  - Hỗ trợ **Clock Stretching** qua `high()` với đồng thời hai giới hạn `stretch_us` và `deadline`.
  - Phát hiện **Bus Busy** (`BB_BUSY`), **Arbitration Loss** (`BB_ARBITRATION`, nhả dây ngay không phát `STOP`), **Address/Data NACK** (`BB_NACK`, phát `STOP` và ghi `cleanup` riêng).
  - Cập nhật `master.nack_phase` để chẩn đoán vị trí lỗi (`0x00` = byte địa chỉ msg0, `1..` = byte dữ liệu msg0, bit `7` (`0x80`) = thuộc msg1, `0xFF` = không NACK).
  - **Không tự retry** trong T1 vì lệnh điều khiển camera (như kích hoạt `0x1D00` hoặc `FFC`) có thể đã được Slave thực thi trước khi mất ACK.
- **Cứu hộ bus (`bb_recover`)**: Phát tối đa `9` xung `SCL` khi `SDA` còn thấp rồi phát `STOP`.

---

## 4. Tích hợp Build & Kiểm thử

- Trong kiến trúc FreeRTOS hiện tại, [hal/cv181x/CMakeLists.txt](../CMakeLists.txt) comment nhánh `# i2c` cũ (`hal_dw_i2c.c`) và các tệp Bitbang T0–T1 được biên dịch trực tiếp vào `libthermal_tiny1c.a` trong [tiny1C/CMakeLists.txt](../../../driver/sensor/tiny1C/CMakeLists.txt) khi bật `-DTHERMAL_TINY1C=ON`.
- Chạy unit test giả lập trên Host (T1 + T2) và kiểm tra biên dịch chéo RISC-V với header FreeRTOS thật (`-Wall -Wextra -Werror`):
  ```bash
  bash cvitek/hal/cv181x/i2c/tests/run.sh
  bash cvitek/hal/cv181x/i2c/tests/compile-riscv.sh
  ```

Liên kết: [T2 — RTOS Soft-I2C Driver](../../../driver/i2c/README.md) · [T3 — Driver Tiny1C](../../../driver/sensor/tiny1C/README.md)
