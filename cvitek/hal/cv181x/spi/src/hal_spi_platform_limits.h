/*
 * Các thông số giới hạn phần cứng của IP SPI0 và System DMA trên SoC SG2002 (CV181x).
 *
 * File này chứa các hằng số vật lý cố định của SoC SG2002 dùng chung giữa
 * Tầng 0 (hal_spi_platform.c) và Tầng 1 (hal_dw_spi.c).
 */

#ifndef HAL_SPI_PLATFORM_LIMITS_H
#define HAL_SPI_PLATFORM_LIMITS_H

/*
 * Profile danh định FPLL/8 = 187.5 MHz; clock thực phải đọc từ T0.
 * Giới hạn SCK của tích hợp SG2002 là 46.875 MHz (TRM mục SPI).
 */
#define HAL_SPI_MAX_SCK_HZ              46875000U
#define HAL_SPI_REF_CLOCK_HZ            187500000U

/*
 * Độ sâu phần cứng của mỗi bộ đệm TX FIFO và RX FIFO trên SG2002 SPI0 là 8 khung.
 *
 * Lưu ý quan trọng:
 * Bộ điều khiển DW APB SSI không tự động dừng phát xung SCK khi RX FIFO đầy.
 * Do đó, khi truyền bằng CPU Polling hoặc Interrupt, số lượng khung "đang bay"
 * (in-flight = số khung đã nạp vào TX nhưng chưa rút ra khỏi RX) tuyệt đối không
 * được vượt quá 8 khung để tránh lỗi tràn bộ đệm nhận (RXOI).
 */
#define HAL_SPI_FIFO_DEPTH              8U

/*
 * Mức ngưỡng mặc định cho TXFTLR và DMATDLR (bằng một nửa độ sâu FIFO = 4 khung).
 * Khi số khung trong TX FIFO giảm xuống <= 4, phần cứng sẽ phát ngắt TXEI hoặc
 * phát yêu cầu DMA (dma_tx_req) để nạp đợt dữ liệu kế tiếp.
 */
#define HAL_SPI_DEFAULT_TX_THRESHOLD    (HAL_SPI_FIFO_DEPTH / 2U)

/*
 * Giá trị tối đa của thanh ghi RX_SAMPLE_DLY trên SG2002 (tính theo chu kỳ ssi_clk).
 * Mỗi đơn vị tương đương ~5.33 ns (1 / 187.5 MHz), dùng để bù trễ truyền dẫn trên
 * đường MISO khi chạy tốc độ cao (23.4 MHz .. 46.875 MHz) hoặc qua mạch dịch mức áp.
 */
#define HAL_SPI_MAX_RX_SAMPLE_DLY       8U

/*
 * Giới hạn bộ chia tần số chẵn SCKDV trong thanh ghi BAUDR:
 * - SCKDV nhỏ nhất = 2     (SCK tối đa lý thuyết = 93.75 MHz)
 * - SCKDV lớn nhất = 65534 (SCK tối thiểu        ~ 2.86 kHz)
 */
#define HAL_SPI_MIN_DIVIDER             2U
#define HAL_SPI_MAX_DIVIDER             65534U

/*
 * Số lượng khung tối đa trong mỗi lần kích hoạt một Block đơn của bộ điều khiển
 * DW AXI DMAC. Với các gói dữ liệu lớn hơn 4096 khung (ví dụ khung ảnh nhiệt
 * 96 KiB = 98,304 byte), Tầng 0 tự động chia thành nhiều block liên tiếp trong
 * khi vẫn giữ nguyên chân CS ở mức LOW.
 */
#define HAL_SPI_DMA_MAX_BLOCK_FRAMES    4096U

#endif /* HAL_SPI_PLATFORM_LIMITS_H */
