/*
 * Định nghĩa thanh ghi phần cứng Synopsys DesignWare APB SSI (SPI Master) trên SG2002 (CV181x).
 *
 * Nguyên tắc phân tầng:
 * - File này thuộc Tầng 1 (T1 - Controller HAL), chỉ được sử dụng nội bộ bởi hal_dw_spi.c,
 *   hal_spi_platform.c và bộ kiểm thử test_spi.c.
 *
 * Sơ đồ thanh ghi SPI0 trên SG2002 (Base Address = 0x04180000):
 * +--------+------------------+--------------------------------------------------------------+
 * | Offset | Tên thanh ghi    | Vai trò vận hành                                             |
 * +--------+------------------+--------------------------------------------------------------+
 * | 0x000  | CTRLR0           | Cấu hình khung truyền: DFS (4..16 bit), FRF, SCPH, SCPOL, TMOD|
 * | 0x004  | CTRLR1           | Số khung nhận (NDF - 1) khi chạy chế độ Receive-Only/EEPROM. |
 * | 0x008  | SSIENR           | Bật/tắt bộ điều khiển SSI (0 = Disable/Reset FIFO, 1 = Enable)|
 * | 0x00C  | MWCR             | Điều khiển giao thức Microwire.                              |
 * | 0x010  | SER              | Slave Enable: cho phép bộ dịch bit bắt đầu phát xung SCK.    |
 * | 0x014  | BAUDR            | Hệ số chia xung nhịp SCK: Fsck = Fssi_clk / SCKDV (số chẵn). |
 * | 0x018  | TXFTLR           | Ngưỡng FIFO phát (phát ngắt TXEI khi số khung <= TXFTLR).    |
 * | 0x01C  | RXFTLR           | Ngưỡng FIFO nhận (phát ngắt RXFI khi số khung >= RXFTLR + 1).|
 * | 0x020  | TXFLR            | Số khung hiện đang nằm chờ trong TX FIFO (0..8).             |
 * | 0x024  | RXFLR            | Số khung hiện đang nằm chờ trong RX FIFO (0..8).             |
 * | 0x028  | SR               | Trạng thái hoạt động: BUSY, TFNF, TFE, RFNE, RFF, DCOL.      |
 * | 0x02C  | IMR              | Mặt nạ ngắt (1 = Cho phép ngắt, 0 = Che ngắt).               |
 * | 0x030  | ISR              | Trạng thái ngắt sau khi đi qua mặt nạ IMR.                   |
 * | 0x034  | RISR             | Trạng thái ngắt thô (Raw status - không phụ thuộc IMR).      |
 * | 0x038  | TXOICR           | Đọc thanh ghi này để xóa cờ lỗi tràn TX FIFO (TXOI).         |
 * | 0x03C  | RXOICR           | Đọc thanh ghi này để xóa cờ lỗi tràn RX FIFO (RXOI).         |
 * | 0x040  | RXUICR           | Đọc thanh ghi này để xóa cờ lỗi đọc rỗng RX FIFO (RXUI).     |
 * | 0x044  | MSTICR           | Đọc thanh ghi này để xóa cờ lỗi tranh chấp bus (MSTI).       |
 * | 0x048  | ICR              | Đọc thanh ghi này để xóa toàn bộ các cờ lỗi cùng lúc.        |
 * | 0x04C  | DMACR            | Điều khiển bắt tay DMA: Bit 0 = RDMAE, Bit 1 = TDMAE.        |
 * | 0x050  | DMATDLR          | Ngưỡng TX FIFO phát yêu cầu dma_tx_req (FIFO <= DMATDLR).    |
 * | 0x054  | DMARDLR          | Ngưỡng RX FIFO phát yêu cầu dma_rx_req (FIFO >= DMARDLR + 1).|
 * | 0x058  | IDR              | Mã định danh phần cứng của IP.                               |
 * | 0x05C  | SSI_VERSION_ID   | Phiên bản phát hành của IP Synopsys.                         |
 * | 0x060  | DR               | Thanh ghi dữ liệu (Data Register) nối trực tiếp vào TX/RX FIFO|
 * | 0x0F0  | RX_SAMPLE_DLY    | Độ trễ lấy mẫu chân MISO (rxd) tính theo chu kỳ ssi_clk.     |
 * +--------+------------------+--------------------------------------------------------------+
 */

#ifndef DW_SPI_REGS_H
#define DW_SPI_REGS_H

#include <stdint.h>

/* --------------------------------------------------------------------------
 * 1. Địa chỉ offset các thanh ghi (Register Offsets)
 * -------------------------------------------------------------------------- */

#define DW_SPI_CTRLR0           0x000U /* Chỉ được phép ghi khi SSIENR = 0 */
#define DW_SPI_CTRLR1           0x004U /* Chỉ được phép ghi khi SSIENR = 0 */
#define DW_SPI_SSIENR           0x008U /* Bật/tắt bộ điều khiển (Bit 0) */
#define DW_SPI_MWCR             0x00CU /* Cấu hình Microwire */
#define DW_SPI_SER              0x010U /* Kích hoạt đường chọn Slave nội bộ */
#define DW_SPI_BAUDR            0x014U /* Bộ chia tần số SCK (chỉ ghi khi SSIENR = 0) */
#define DW_SPI_TXFTLR           0x018U /* Ngưỡng kích hoạt ngắt TX FIFO Empty */
#define DW_SPI_RXFTLR           0x01CU /* Ngưỡng kích hoạt ngắt RX FIFO Full */
#define DW_SPI_TXFLR            0x020U /* Số phần tử hiện có trong TX FIFO */
#define DW_SPI_RXFLR            0x024U /* Số phần tử hiện có trong RX FIFO */
#define DW_SPI_SR               0x028U /* Thanh ghi trạng thái tổng hợp (Chỉ đọc) */
#define DW_SPI_IMR              0x02CU /* Thanh ghi mặt nạ ngắt */
#define DW_SPI_ISR              0x030U /* Trạng thái ngắt đã lọc qua IMR (Chỉ đọc) */
#define DW_SPI_RISR             0x034U /* Trạng thái ngắt gốc từ phần cứng (Chỉ đọc) */
#define DW_SPI_TXOICR           0x038U /* Đọc để xóa lỗi tràn TX FIFO */
#define DW_SPI_RXOICR           0x03CU /* Đọc để xóa lỗi tràn RX FIFO */
#define DW_SPI_RXUICR           0x040U /* Đọc để xóa lỗi đọc rỗng RX FIFO */
#define DW_SPI_MSTICR           0x044U /* Đọc để xóa lỗi Multi-Master */
#define DW_SPI_ICR              0x048U /* Đọc để xóa đồng thời mọi cờ lỗi FIFO */
#define DW_SPI_DMACR            0x04CU /* Bật/tắt tín hiệu yêu cầu DMA cho TX và RX */
#define DW_SPI_DMATDLR          0x050U /* Mức ngưỡng TX FIFO gọi DMA */
#define DW_SPI_DMARDLR          0x054U /* Mức ngưỡng RX FIFO gọi DMA */
#define DW_SPI_IDR              0x058U /* Thanh ghi ID phần cứng */
#define DW_SPI_VERSION          0x05CU /* Thanh ghi phiên bản IP */
#define DW_SPI_DR               0x060U /* Cửa sổ ghi TX FIFO và đọc RX FIFO */
#define DW_SPI_RX_SAMPLE_DLY    0x0F0U /* Độ trễ lấy mẫu MISO (chỉ ghi khi SSIENR = 0) */

/* --------------------------------------------------------------------------
 * 2. Các trường bit của thanh ghi điều khiển CTRLR0 (Offset 0x000)
 * -------------------------------------------------------------------------- */

/*
 * DFS (Data Frame Size - bits [3:0]):
 * Quy định số bit trên mỗi khung truyền (hỗ trợ từ 4 đến 16 bit).
 * Giá trị nạp vào thanh ghi bằng (bits_per_word - 1).
 * Ví dụ: khung 8-bit -> ghi 7 (0x7); khung 16-bit -> ghi 15 (0xF).
 */
#define DW_SPI_CTRLR0_DFS_MASK      0x000FU

/*
 * FRF (Frame Format - bits [5:4]):
 * Chọn chuẩn giao thức vật lý: 0x0 = Motorola SPI, 0x1 = TI SSP, 0x2 = Microwire.
 */
#define DW_SPI_CTRLR0_FRF_MASK      (0x3U << 4)
#define DW_SPI_CTRLR0_FRF_SPI       (0x0U << 4)

/*
 * SCPH (Clock Phase / CPHA - bit 6) và SCPOL (Clock Polarity / CPOL - bit 7):
 * - Mode 0: CPOL = 0, CPHA = 0 (Xung nhàn rỗi mức THẤP, lấy mẫu ở cạnh đầu tiên)
 * - Mode 1: CPOL = 0, CPHA = 1 (Xung nhàn rỗi mức THẤP, lấy mẫu ở cạnh thứ hai)
 * - Mode 2: CPOL = 1, CPHA = 0 (Xung nhàn rỗi mức CAO,  lấy mẫu ở cạnh đầu tiên)
 * - Mode 3: CPOL = 1, CPHA = 1 (Xung nhàn rỗi mức CAO,  lấy mẫu ở cạnh thứ hai)
 */
#define DW_SPI_CTRLR0_SCPH          (1U << 6)
#define DW_SPI_CTRLR0_SCPOL         (1U << 7)

/*
 * TMOD (Transfer Mode - bits [9:8]):
 * - TMOD_TR (0x0): Song công toàn phần (Transmit & Receive).
 *   Mỗi khung nạp vào TX FIFO phát ra đúng 1 khung xung SCK và thu về 1 khung vào RX FIFO.
 *   Driver sử dụng chế độ TMOD_TR làm nền tảng thống nhất cho cả Full-Duplex, TX-only
 *   và RX-only để kiểm soát chính xác tuyệt đối số lượng khung dữ liệu trên đường truyền.
 */
#define DW_SPI_CTRLR0_TMOD_MASK     (0x3U << 8)
#define DW_SPI_CTRLR0_TMOD_TR       (0x0U << 8)
#define DW_SPI_CTRLR0_TMOD_TO       (0x1U << 8)
#define DW_SPI_CTRLR0_TMOD_RO       (0x2U << 8)
#define DW_SPI_CTRLR0_TMOD_EPROM    (0x3U << 8)

/* SRL (Shift Register Loop - bit 11): Nối vòng TX sang RX nội bộ để tự kiểm tra */
#define DW_SPI_CTRLR0_SRL           (1U << 11)

/* --------------------------------------------------------------------------
 * 3. Các cờ trong thanh ghi trạng thái SR (Offset 0x028)
 * -------------------------------------------------------------------------- */

#define DW_SPI_SR_BUSY              (1U << 0) /* 1 = Đang dịch bit trên đường dây vật lý */
#define DW_SPI_SR_TFNF              (1U << 1) /* 1 = TX FIFO chưa đầy (còn chỗ để ghi) */
#define DW_SPI_SR_TFE               (1U << 2) /* 1 = TX FIFO hoàn toàn trống */
#define DW_SPI_SR_RFNE              (1U << 3) /* 1 = RX FIFO có ít nhất 1 khung chờ đọc */
#define DW_SPI_SR_RFF               (1U << 4) /* 1 = RX FIFO đã đầy */
#define DW_SPI_SR_DCOL              (1U << 6) /* 1 = Xung đột dữ liệu (Data Collision) */

/* --------------------------------------------------------------------------
 * 4. Các cờ ngắt trong IMR / ISR / RISR (Offsets 0x02C .. 0x034)
 * -------------------------------------------------------------------------- */

#define DW_SPI_INT_TXEI             (1U << 0) /* TX FIFO xuống dưới hoặc bằng ngưỡng TXFTLR */
#define DW_SPI_INT_TXOI             (1U << 1) /* Lỗi tràn TX FIFO (ghi khi FIFO đã đầy) */
#define DW_SPI_INT_RXUI             (1U << 2) /* Lỗi đọc rỗng RX FIFO (đọc khi FIFO trống) */
#define DW_SPI_INT_RXOI             (1U << 3) /* Lỗi tràn RX FIFO (nhận khung mới khi RX đã đầy) */
#define DW_SPI_INT_RXFI             (1U << 4) /* RX FIFO đạt từ (RXFTLR + 1) khung trở lên */
#define DW_SPI_INT_MSTI             (1U << 5) /* Tranh chấp Multi-Master */

/* Nhóm cờ báo lỗi phần cứng FIFO cần hủy phiên và thu hồi ngay */
#define DW_SPI_FIFO_ERR_MASK        (DW_SPI_INT_TXOI | DW_SPI_INT_RXUI | DW_SPI_INT_RXOI)

/* Nhóm cờ ngắt sử dụng trong chế độ truyền bằng ngắt (IRQ mode) */
#define DW_SPI_IRQ_XFER_MASK        (DW_SPI_INT_TXEI | DW_SPI_INT_RXFI | DW_SPI_FIFO_ERR_MASK)

/* --------------------------------------------------------------------------
 * 5. Các trường của thanh ghi DMA (DMACR) và lấy mẫu trễ (RX_SAMPLE_DLY)
 * -------------------------------------------------------------------------- */

#define DW_SPI_DMACR_RDMAE          (1U << 0) /* Cho phép phát yêu cầu DMA nhận (dma_rx_req) */
#define DW_SPI_DMACR_TDMAE          (1U << 1) /* Cho phép phát yêu cầu DMA phát (dma_tx_req) */

#define DW_SPI_RX_SAMPLE_DLY_MASK   0x00FFU   /* Mặt nạ số chu kỳ trễ lấy mẫu MISO (0..8) */

#endif /* DW_SPI_REGS_H */
