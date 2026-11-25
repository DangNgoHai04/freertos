/*
 * Tầng 3 (T3 - Tiny1C SPI Transport Adapter): Cầu nối giao tiếp SPI0 tốc độ cao
 * ủy quyền hoàn toàn xuống API Tầng 2 (cvitek/driver/spi/include/spi.h).
 *
 * Nguyên tắc phân tầng:
 * - Tầng 3 KHÔNG ghi trực tiếp thanh ghi MMIO của SPI0, D-PHY, Pinmux hay DMA.
 * - Mọi thao tác khởi tạo, đổi tốc độ SCK, điều khiển CS0 (GPIOC17), truyền/nhận
 *   DMA + Polling và phục hồi bus đều đi qua spi_bus_t & spi_device_t của Tầng 2.
 */

#ifndef TINY1C_HAL_SPI_H
#define TINY1C_HAL_SPI_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "all_config.h"

#if defined(__has_include) && __has_include("spi.h") && __has_include("hal_dw_spi.h") && __has_include("FreeRTOS.h")
#undef spi_read
#undef spi_write
#include "spi.h"
#define TINY1C_HAS_T2_SPI 1
#endif
#if defined(__riscv) && !defined(TINY1C_HAS_T2_SPI)
#error "Target builds require the real T2 SPI headers"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Các hằng số phần cứng giữ lại để tương thích ngược với cấu trúc RTOS_V1 */
#ifndef SPI0_BASE
#define SPI0_BASE 0x04180000
#endif
#define SPI0_DSI_DEFAULT_CS_PIN     0x0C11      /* Port C (0x0C), Pin 17 (XGPIOC[17]) */
#define SPI0_DSI_CS_PIN_DEFAULT     SPI0_DSI_DEFAULT_CS_PIN
#define SPI0_DSI_CS_GPIOC_BIT       17U
#define SPI0_VOSPI_DEFAULT_BAUD_DIV 8U          /* 187.5 MHz / 8 = 23.4375 MHz */
#define SPI0_BAUDR_DIV_DEFAULT      SPI0_VOSPI_DEFAULT_BAUD_DIV

#if defined(TINY1C_HAS_T2_SPI)
/* Liên kết trực tiếp một bus SPI0 đã được khởi tạo ở Tầng 2 (nếu chia sẻ bus) */
int hal_spi_bind_bus(spi_bus_t *bus);

/* Lấy con trỏ tới đối tượng spi_device_t Tầng 2 của Tiny1C */
spi_device_t *hal_spi_get_device(void);
#endif

/*
 * Xác nhận từ tầng tích hợp Board rằng kênh DMA 0/1 (remap 20/21) đã được dành riêng
 * cho SPI0 trên RTOS trước khi khởi tạo spi_bus_init().
 */
void hal_spi_set_dma_reserved(bool reserved);

/*
 * Đổi trực tiếp tần số xung nhịp SCK của thiết bị SPI0 Tiny1C thông qua spi_device_set_freq()
 * ở Tầng 2 (ví dụ: 23.4375 MHz cho luồng đơn 96KB, 46.875 MHz cho luồng kép 192KB).
 */
int hal_spi_set_freq_hz(uint32_t sck_hz);
uint32_t hal_spi_get_freq_hz(void);

/*
 * Truyền/nhận nguyên tử một khối dữ liệu qua Tầng 2 (spi_transfer_one):
 * - Nếu tx_buf == NULL, Tầng 2/Tầng 1 tự động đẩy dummy_fill (ví dụ 0x0055) mà không tốn RAM.
 * - Mặc định dùng polling. Chỉ khi đã xác nhận tài nguyên DMA trước khi khởi tạo bus,
 *   Tầng 2 mới được phép chọn DMA. CS GPIOC17 giữ LOW xuyên suốt giao dịch.
 */
int hal_spi_transfer_frame(const void *tx_buf, void *rx_buf, size_t len,
                           uint16_t dummy_fill, uint32_t timeout_ms);

/* Khôi phục cứng Bus SPI0 thông qua spi_bus_recover() của Tầng 2 khi mất đồng bộ VoSPI */
int hal_spi_recover_bus(void);

/* =========================================================================
 * Các hàm API tương thích hoàn toàn với RTOS_V1 (bên trong gọi xuống Tầng 2)
 * ========================================================================= */

/* Khởi tạo Bus SPI0 và Thiết bị Tiny1C ở Mode 3, 8-bit, CS0 GPIOC17 qua Tầng 2 */
void spi0_dsi_hw_init(void);

/* Thay đổi tốc độ xung nhịp SCK của thiết bị Tiny1C qua spi_device_set_freq() (Tầng 2) */
void spi0_dsi_configure_vospi(uint16_t baud_div, bool use_25m_osc);

/* Điều khiển chân CS (được Tầng 2/Tầng 1 quản lý tự động trong mỗi spi_transfer) */
void spi0_dsi_cs_set(int cs_gpio_pin, bool level);

/* Truyền nhận khối dữ liệu qua spi_transfer_one() của Tầng 2 */
int spi0_dsi_transfer_burst(const uint8_t *tx_buf, uint8_t *rx_buf, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* TINY1C_HAL_SPI_H */
