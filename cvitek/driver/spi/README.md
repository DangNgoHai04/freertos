# Driver SPI0 — Tầng 2 (Public RTOS Bus & Device Driver API)

Lớp điều phối Bus và Thiết bị SPI an toàn đa luồng trên FreeRTOS dành cho các driver cảm biến ngoại vi (như [Tiny1C](../../driver/sensor/tiny1C/README.md)).

---

## 1. Cấu trúc mã nguồn

| Tệp | Trách nhiệm |
| :--- | :--- |
| [include/spi.h](include/spi.h) | Khai báo Public API Tầng 2 (`spi_*`), mã trạng thái `spi_status_t`, backend `spi_backend_t` (`POLL`, `IRQ`, `DMA`, `AUTO`), cấu trúc `spi_bus_config_t`, `spi_device_config_t`, `spi_segment_t`, `spi_bus_t`, `spi_device_t`. |
| [src/spi.c](src/spi.c) | Triển khai khóa `StaticSemaphore_t` Mutex, giao dịch nguyên tử đa phân đoạn, giữ/nhả CS, Lazy Reconfiguration, điều phối backend `AUTO`, kiểm tra quyền trước CS và phục hồi lỗi. |
| [CMakeLists.txt](CMakeLists.txt) | Biên dịch object `spi` và cài đặt `include/driver/spi/spi.h`. |

---

## 2. Cơ chế hoạt động cốt lõi

1. **An toàn đa nhiệm không dùng Heap**:
   - `spi_bus_init()` tạo mutex tĩnh `xSemaphoreCreateMutexStatic(&bus->mutex_storage)` ngay trong vùng nhớ do caller cấp. Mọi API truy cập bus (`spi_transfer`, `spi_bus_deinit`, `spi_bus_recover`, `spi_device_set_backend`, `spi_device_set_freq`) đều giữ mutex trong suốt thao tác.
2. **Giao dịch đa phân đoạn & Chip-Select nguyên tử**:
   - `spi_transfer()` kiểm tra toàn bộ mảng `spi_segment_t`, giữ chân CS `GPIOC17` ở mức LOW (`hal_spi_platform_cs_set(true)`) xuyên suốt mọi phân đoạn, hỗ trợ `delay_us` sau phân đoạn và xung nhả CS `2 µs` khi `seg->cs_change == true` trước khi nhả CS về HIGH ở cuối phiên.
3. **Chính sách Backend & Khởi tạo trễ trước CS (Pre-CS Validation)**:
   - Mặc định `spi_bus_config_t` chỉ cho phép **`SPI_BACKEND_POLL`**. IRQ và DMA chỉ được phép khi bus bật `enable_irq` / `enable_dma` (và `dma_resources_reserved = true` đối với DMA).
   - Ở chế độ `SPI_BACKEND_AUTO`: chọn `DMA` nếu được phép và `frames >= dma_threshold_frames` (`64`), `IRQ` nếu được phép và `frames >= irq_threshold_frames` (`32`), ngược lại dùng `POLL`.
   - Trước khi kéo CS xuống LOW, T2 gọi `hal_spi_prepare_irq()` / `hal_spi_prepare_dma()` cho các backend cần dùng trong message; nếu lỗi, trả mã lỗi ngay mà không làm nháy chân CS.
4. **Lazy Reconfiguration & Fault Recovery**:
   - T2 lưu `bus->active_dev`; nếu cấu hình phần cứng không đổi giữa hai lần truyền liên tiếp, bước ghi lại thanh ghi T1 được bỏ qua.
   - Khi một phân đoạn gặp `SPI_ERR_FIFO` hoặc `SPI_ERR_TIMEOUT`, T2 tự động chụp `hal_spi_snapshot(&bus->hal, &bus->last_snapshot)`, gọi `hal_spi_abort()`, nhả CS lên HIGH, xóa `active_dev`, tăng `error_count` và mở khóa mutex.

---

## 3. Danh mục Public API

- **Quản trị Bus & Thiết bị**: `spi_bus_init()`, `spi_bus_deinit()`, `spi_bus_recover()`, `spi_device_init()`, `spi_device_set_backend()`, `spi_device_set_freq()`.
- **Truyền nhận dữ liệu**: `spi_transfer()`, `spi_transfer_one()`, `spi_write_then_read()`, `spi_write()`, `spi_read()`.

### Cách Tiny1C (T3) sử dụng SPI0 T2
- Khởi tạo bus `0x04180000` (polling mặc định, hoặc `AUTO` nếu board xác nhận DMA), gắn thiết bị ở **SPI Mode 3**, `8-bit`, MSB-first, `rx_sample_dly = 1`, `cs_id = SPI_CS0_GPIO`, tần số `11.71875 MHz` (`187.5 MHz / 16`).
- Mỗi mặt phẳng VoSPI thực hiện hai lần gọi `spi_transfer_one()`: một phiên CS gửi/nhận Header `512` byte (`0xAA`/`0xCC`) và một phiên CS nhận Payload `98,304` byte (`dummy_fill = 0x55`).

---

## 4. Tích hợp Build & Kiểm thử

- Target `spi` được thêm tự động cho `CHIP == "cv181x"` trên `riscv64` trong [driver/CMakeLists.txt](../CMakeLists.txt) và đóng gói vào `libdriver.a`.
- Chạy unit test cho cả T0–T2:
  ```bash
  bash cvitek/hal/cv181x/spi/tests/run.sh
  ```

Liên kết: [T0 & T1 — HAL SPI0](../../hal/cv181x/spi/README.md) · [T3 — Driver Tiny1C](../sensor/tiny1C/README.md)
