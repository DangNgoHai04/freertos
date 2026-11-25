# HAL SPI0 — Tầng 0 (Platform SoC) & Tầng 1 (DW APB SSI Controller HAL)

Tài liệu kỹ thuật cho hai tầng phần cứng SPI0 trên **SOPHGO SG2002 / CV181x** (nhân RISC-V C906L chạy FreeRTOS, cổng FPC2 của LicheeRV-Nano).

---

## 1. Cấu trúc mã nguồn

| Tầng | Tệp | Trách nhiệm chính |
| :--- | :--- | :--- |
| **T0 — Platform Limits** | [hal_spi_platform_limits.h](src/hal_spi_platform_limits.h) | Giới hạn phần cứng SoC: `HAL_SPI_REF_CLOCK_HZ` (`187.5 MHz`), trần `HAL_SPI_MAX_SCK_HZ` (`46.875 MHz`), `HAL_SPI_FIFO_DEPTH` (`8`), `BAUDR` chẵn `2..65534`, `RX_SAMPLE_DLY` `0..8`, `HAL_SPI_DMA_MAX_BLOCK_FRAMES` (`4096`). |
| **T0 — Platform I/O** | [hal_spi_platform_io.c](src/hal_spi_platform_io.c) | Đọc clock tree `clk_spi`, bật clock gate `CLK_EN_0/1/3`, Soft-Reset `SPI0`, mở khóa MIPI DSI D-PHY, cấu hình `PADCTRL` + `FMUX` cho FPC2 và điều khiển Software CS `GPIOC17`. |
| **T0 — IRQ & DMA** | [hal_spi_platform.c](src/hal_spi_platform.c) | Khởi tạo trễ (lazy init) ngắt PLIC `SPI_0_SSI_INTR` (IRQ 37) + `StaticSemaphore_t`, System DMA (`DW_axi_dmac`) Kênh 0/1 và đồng bộ D-Cache 64-byte của C906L. |
| **T1 — Controller HAL** | [hal_dw_spi.c](src/hal_dw_spi.c), [dw_spi_regs.h](src/dw_spi_regs.h) | Bộ điều khiển Synopsys DW APB SSI (`0x04180000`): cấu hình `CTRLR0`/`BAUDR`/`RX_SAMPLE_DLY`, đảo bit LSB-first, 3 engine `POLL`/`IRQ`/`DMA`, `hal_spi_snapshot()` và `hal_spi_abort()`. |
| **T0/T1 Interface** | [hal_dw_spi.h](include/hal_dw_spi.h) | Khai báo 12 Platform Hooks của T0, API vòng đời của T1, cấu trúc `hal_spi_t`, `hal_spi_config_t`, `hal_spi_snapshot_t`. |

---

## 2. Tầng 0 (Platform SoC Layer)

Tầng 0 trừu tượng hóa tài nguyên SoC nằm ngoài khối IP SPI0 (`0x04180000`):

1. **Clock Tree & Soft-Reset (`hal_spi_platform_io.c`)**:
   - `hal_spi_platform_get_ref_clock_hz()` đọc `CLK_BYP_0` (`0x03002030` bit 30) và `DIV_CLK_SPI` (`0x03002100`) để lấy tần số `clk_spi` thực (`25 MHz` XTAL khi bypass hoặc `1500 MHz / div`, mặc định chia 8 = **`187.5 MHz`**).
   - `hal_spi_platform_clock_enable()` mở `clk_apb_gpio` (`0x03002000` bit 29), `clk_apb_spi0` (`0x03002004` bit 9), `clk_spi` (`0x0300200C` bit 6) kèm readback xác nhận.
   - `hal_spi_platform_reset()` phát xung Reset cứng mức thấp tại `SOFT_RSTN_1` (`0x03003004` bit 8) và đọc kiểm chứng.
2. **MIPI DSI D-PHY Bypass, PADCTRL, Pinmux FPC2 & Glitch-free CS**:
   - `hal_spi_platform_cs_init()` nạp trước mức `DR = 1` (Inactive HIGH) và hướng Output (`DDR = 1`) cho `GPIOC17` (`0x03022000`/`0x03022004`) **trước khi** chuyển mux để tránh xung CS giả lúc khởi tạo.
   - `hal_spi_platform_pinmux_init()` mở khóa 5 lane khối MIPI DSI D-PHY (`0x0A0D1000`, `GPO`/`GPI` = `0x00001F1F`), cấu hình `PADCTRL` (`0x03001C70..0x03001C7C = 0x44`) và ánh xạ `FMUX` cho cổng FPC2 kèm readback:
     - `FPC2.15` (`PAD_MIPI_TXM2`, `0x030011A4`): Func `6` = `SPI0_SCK`
     - `FPC2.14` (`PAD_MIPI_TXP2`, `0x030011A8`): Func `3` = `XGPIOC_17` (Software CS)
     - `FPC2.18` (`PAD_MIPI_TXM1`, `0x030011AC`): Func `6` = `SPI0_SDO` (MOSI)
     - `FPC2.17` (`PAD_MIPI_TXP1`, `0x030011B0`): Func `6` = `SPI0_SDI` (MISO)
   - Bổ sung thanh ghi và cấu hình tham chiếu trong [top_reg.h](../config/top_reg.h) và [hal_pinmux.c](../pinmux/src/hal_pinmux.c).
3. **Ngắt PLIC & System DMA (`hal_spi_platform.c` — Lazy Init)**:
   - **PLIC IRQ**: Chỉ đăng ký `SPI_0_SSI_INTR` (IRQ `37`) và tạo `StaticSemaphore_t` khi T1 gọi `hal_spi_prepare_irq()`; xóa token cũ và đánh thức task từ `hal_spi_platform_irq_notify()`.
   - **System DMA (`DW_axi_dmac` tại `0x04330000`)**: `hal_spi_platform_dma_init()` kiểm tra `resources_reserved == true` trước khi chạm MMIO; bật clock `clk_sdma_axi` (`0x03002004` bit 1), nhả reset `SDMA` (`0x03003000` bit 18) và kiểm tra mọi kênh trong `DW_DMAC_CHEN_L` đang rảnh (`(chen & 0xFF) == 0`) trước khi remap Kênh 0 RX (`Req 16`) / Kênh 1 TX (`Req 17`) qua `0x03000154` và `0x03000298`.
   - `hal_spi_platform_dma_transfer()` dùng 2 dòng cache tĩnh tách biệt 64-byte (`g_dma_tx_dummy_line`, `g_dma_rx_dummy_line`) tránh false-sharing, thực hiện Clean TX / Flush RX trước truyền và Invalidate RX sau truyền theo cache line `64B` của C906L, tự chia block tối đa `4096` khung.

---

## 3. Tầng 1 (Synopsys DW APB SSI Controller HAL)

Tầng 1 điều khiển trực tiếp thanh ghi DW APB SSI (`0x04180000`) và **không** quản lý Mutex hay giữ/nhả chân CS (thuộc [Tầng 2](../../../driver/spi/README.md)):

- **Cấu hình (`hal_spi_configure`)**: Hỗ trợ Motorola SPI Mode `0..3`, độ rộng khung `4..16 bit`, `RX_SAMPLE_DLY` `0..8`, tính bộ chia chẵn `BAUDR` (`2..65534`) với trần an toàn `HAL_SPI_MAX_SCK_HZ = 46.875 MHz`, và chế độ đảo bit phần mềm `lsb_first` (dùng bounce buffer 128B trên Stack khi chạy DMA).
- **3 Engine thực thi**:
  - `hal_spi_poll()`: Bơm/rút FIFO giữ bất biến `in_flight = tx_done - rx_done < 8` chống tràn RX FIFO (`RXOI`).
  - `hal_spi_transfer_irq()` + `hal_spi_isr()`: Hạ động ngưỡng `RXFTLR` cho phần đuôi gói tin (`rem_rx <= 4`) để không bị treo ngắt cuối gói.
  - `hal_spi_transfer_dma()`: Kích hoạt `DMACR` (`RDMAE | TDMAE`) với `DMATDLR = 4`, `DMARDLR = 0`.
- **Vòng đời & Thu hồi**: `hal_spi_init()` $\rightarrow$ (`hal_spi_prepare_irq()` / `hal_spi_prepare_dma()`) $\rightarrow$ `hal_spi_configure()` $\rightarrow$ `hal_spi_begin()` $\rightarrow$ engine $\rightarrow$ `hal_spi_finish()`; khi lỗi gọi `hal_spi_snapshot()` và `hal_spi_abort()`.

---

## 4. Tích hợp Build & Kiểm thử

- Target CMake `halspi` trong [CMakeLists.txt](CMakeLists.txt) được đóng gói vào `libhal.a` qua [hal/cv181x/CMakeLists.txt](../CMakeLists.txt).
- Chạy kiểm thử đơn vị giả lập trên Host cho cả T0–T2:
  ```bash
  bash cvitek/hal/cv181x/spi/tests/run.sh
  ```
  *(Bao phủ POLL, IRQ, DMA, AUTO, LSB-first, Helpers, Fault Recovery trong [test_spi.c](tests/test_spi.c).)*

Liên kết tầng trên: [Tầng 2 — RTOS SPI Driver](../../../driver/spi/README.md) · [Tầng 3 — Driver Tiny1C](../../../driver/sensor/tiny1C/README.md)
