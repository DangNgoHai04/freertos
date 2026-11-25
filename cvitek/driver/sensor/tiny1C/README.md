# Driver T3 & RTOS Service Camera Nhiệt Tiny1C (ASIC-2121W)

Tầng 3 điều khiển module camera nhiệt **InfiRay Tiny1C** bằng giao thức VDCMD qua [Soft-I²C5 (T2)](../../i2c/README.md) và thu khung ảnh/nhiệt VoSPI qua [SPI0 (T2)](../../spi/README.md). Dịch vụ RTOS ([thermal_service.c](rtos/thermal_service.c)) gắn vào hàng đợi `E_QUEUE_CAMERA` (`IP_CAMERA = 7`) trong [comm_main.c](../../../task/comm/src/riscv64/comm_main.c) để giao tiếp với nhân Linux qua Mailbox và vùng nhớ dùng chung DDR.

---

## 1. Cấu trúc mã nguồn trong `freertos`

| Thành phần | Tệp | Trách nhiệm chính |
| :--- | :--- | :--- |
| **Core & Lifecycle** | [src/core.c](src/core.c), [include/core.h](include/core.h) | Khởi tạo (`core_init`), probe thông tin thiết bị (`core_probe_device_info`), bật/tắt luồng (`core_start`, `core_stop`), `trigger_shutter` (FFC), truy vấn nhiệt độ TPD, cấu hình ISP và hiệu chuẩn. |
| **Falcon ASIC Commands** | [src/falcon_cmd.c](src/falcon_cmd.c), [include/falcon_cmd.h](include/falcon_cmd.h) | Đóng gói tập lệnh ASIC-2121W (`get_device_info`, `preview_start`, `y16_preview_start`, `ooc_b_update`, TPD, ISP, Flash). |
| **VDCMD Engine** | [src/vdcmd.c](src/vdcmd.c), [include/vdcmd.h](include/vdcmd.h) | Đóng gói header Short (4B), Standard (8B), Long (16B); chia nạp giữ (`0x9D00`) / kích hoạt (`0x1D00`) và giữ Static Mutex thiết bị trọn chuỗi submit $\rightarrow$ poll $\rightarrow$ response. |
| **I2C5 Transport Adapter** | [src/data_rw.c](src/data_rw.c), [include/data_rw.h](include/data_rw.h) | Adapter gọi `bb_gpio_platform_init()` + `soft_i2c_*()`, chuẩn hóa địa chỉ `0x78` $\rightarrow$ 7-bit `0x3C`, poll thanh ghi `0x0200` bằng `rdtime` `25 MHz`, thu thập chẩn đoán I2C/VDCMD và `data_rw_scan_i2c()`. |
| **SPI0 Transport Adapter** | [src/hal_spi.c](src/hal_spi.c), [include/hal_spi.h](include/hal_spi.h) | Khởi tạo SPI0 qua `spi_bus_init()` / `spi_device_init()` (Mode 3, 8-bit, MSB-first, `rx_sample_dly=1`, CS `GPIOC17`), đổi tần số SCK và chuyển gói qua `spi_transfer_one()`. |
| **VoSPI Frame Acquisition** | [src/vospi.c](src/vospi.c), [include/vospi.h](include/vospi.h) | Khởi tạo luồng VoSPI (`spi_stream_init`), thu gói Header `512B` (`0xAA`/`0xCC`), quét động offset header `64..480`, kiểm tra `frame_valid` và lỗi all-`0xFF`, thu Payload `98,304B` (`MODE_TEMP`/`MODE_IMAGE`) hoặc `196,608B` (`MODE_DUAL`). |
| **Cấu hình & ABI** | [include/all_config.h](include/all_config.h), [include/thermal_protocol.h](include/thermal_protocol.h) | Cấu hình tần số I2C/SPI, timeout, mã lỗi `ir_error_t`, và định nghĩa ABI 64-byte `thermal_header_t` (`THM1`) dùng chung với Linux. |
| **RTOS Service & Mailbox** | [rtos/thermal_service.c](rtos/thermal_service.c), [comm_main.c](../../../task/comm/src/riscv64/comm_main.c) | Tác vụ `thermal_service_task` nhận lệnh `THERMAL_START` (`0x40`), `CAPTURE` (`0x41`), `STOP` (`0x42`), `FFC` (`0x43`), `INFO` (`0x44`), ghi vào `reply` căn lề 64B, `flush_dcache_range` + `fence rw, rw` và trả qua `E_QUEUE_CMDQU`. |

---

## 2. Điều khiển qua Soft-I²C5

- Task xác nhận quyền sở hữu GPIOA (`THERMAL_GPIO_OWNED`) trước khi gọi `core_init(5, I2C_SLAVE_ID)`. Driver chuẩn hóa `0x78` thành địa chỉ 7-bit `0x3C`.
- `core_probe_device_info()` đọc `DEV_INFO_CHIP_ID`, `PROJECT`, `FW_DATE`, `FW_VERSION`, `PN`, `SN` và ghi nhận `probe_ok_mask` (`0xF3` khi đủ toàn bộ). Lỗi transport ở `PROJECT`/`PN`/`SN` trả `IR_NOT_GET_DEVICE_INFO`; riêng `IR_CHECK_DONE_FAIL` vẫn ghi log và cho phép tiếp tục để hỗ trợ chẩn đoán trạng thái ASIC.
- Ghi nạp giữ dùng vùng `0x9D00` (cho phép retry tối đa 3 lần khi lỗi đường dây); ghi kích hoạt dùng vùng `0x1D00` (không retry mù tránh lặp lệnh) rồi poll thanh ghi `0x0200` tối đa `20 s` (`I2C_TRANSFER_WAIT_TIME_MS`).
- Cấu hình mặc định trong [all_config.h](include/all_config.h): `100 kHz`, `THERMAL_I2C_STRETCH_US = 20000`, `THERMAL_I2C_TIMEOUT_MS = 1000`, `THERMAL_VDCMD_MUTEX_TIMEOUT_MS = 25000`. Khi build firmware thực tế cho board có thể override qua `-DTHERMAL_I2C_FREQ_HZ=20000`.

---

## 3. Thu ảnh qua SPI0 (VoSPI)

- `spi_stream_init()` cấu hình SPI0 Mode 3, 8-bit, MSB-first, CS `GPIOC17`, `rx_sample_dly = 1`, backend mặc định **`SPI_BACKEND_POLL`** (chỉ chuyển sang `SPI_BACKEND_AUTO` khi caller xác nhận `THERMAL_DMA_OWNED`).
- `core_start()` đặt SCK về `11,718,750 Hz` (`187.5 MHz / 16` theo `THERMAL_SPI_DEFAULT_SCK_HZ` / `THERMAL_SPI_DUAL_SCK_HZ`) cho cả chế độ đơn và kép, bật `preview_start()` (và `y16_preview_start(..., Y16_MODE_TEMPERATURE)` đối với `MODE_TEMP`).
- Mỗi mặt phẳng thu qua 2 phiên CS riêng:
  1. Gửi `512` byte (`s_tx_dummy[0] = 0xAA` hoặc `0xCC`) vào `s_rx_dummy`; thử header 32 byte tại offset `64`, nếu `(s_rx_dummy[64] & 0x01) == 0` thì quét từ `64` đến `480` tìm byte khác `0` đầu tiên (tương thích module Tiny1C xuất 480B dummy `0x00` + 32B header). Từ chối header toàn `0xFF` (`-2`) hoặc `frame_valid == 0` (`-4`).
  2. Chờ `delay_ms(1)` rồi đọc `98,304` byte (`dummy_fill = 0x55`) thẳng vào buffer đích. Ở `MODE_DUAL`, thu tuần tự `0xAA` (96KB YUYV) và `0xCC` (96KB Y16) và gắn cờ nếu `index_image != index_temp`.

---

## 4. Tích hợp CMake & Kiểm thử độc lập trong `freertos`

- [task/CMakeLists.txt](../../../task/CMakeLists.txt) thêm thư viện `thermal_tiny1c` (và [task/main/CMakeLists.txt](../../../task/main/CMakeLists.txt) liên kết vào `cvirtos.elf`) khi truyền `-DTHERMAL_TINY1C=ON` (yêu cầu `CHIP=cv181x`, `FAST_IMAGE_TYPE=0`). Khi tắt tùy chọn (`-DTHERMAL_TINY1C=OFF`), firmware build như cấu hình mặc định.
- Chạy host test (`test_driver`, `test_i2c_failure`, `test_vospi`) và kiểm tra cross-compile RISC-V từ thư mục `freertos/`:
  ```bash
  make -C cvitek/driver/sensor/tiny1C test riscv-check
  ```
  *Lưu ý bảo vệ mục tiêu:* trên RISC-V (`__riscv`), [all_config.h](include/all_config.h) cấm định nghĩa `THERMAL_HOST_TEST` và yêu cầu đầy đủ header FreeRTOS, Soft-I²C và SPI T2 thật.

Liên kết các tầng dưới: [T0 & T1 I2C5](../../../hal/cv181x/i2c/README.md) · [T2 Soft-I2C](../../i2c/README.md) · [T0 & T1 SPI0](../../../hal/cv181x/spi/README.md) · [T2 SPI0](../../spi/README.md)
