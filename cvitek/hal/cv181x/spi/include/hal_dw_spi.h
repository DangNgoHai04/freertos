/*
 * Giao diện lập trình Tầng 0 (Platform SoC Hooks) và Tầng 1 (Controller HAL).
 *
 * Phân tách trách nhiệm:
 * - Tầng 0 (hal_spi_platform.c):
 *   Chịu trách nhiệm với các tài nguyên cấp SoC SG2002 nằm bên ngoài khối IP SPI:
 *   + Cấp xung nhịp (Clock Gate) và phát xung Reset mềm (Soft-Reset) cho SPI0 và SDMA.
 *   + Mở khóa bộ đệm số TTL của cụm MIPI D-PHY (FPC 26 chân) và cấu hình Pinmux / IOBLK.
 *   + Điều khiển chân Chip-Select (GPIOC17) bằng phần mềm để giữ CS ổn định suốt giao dịch.
 *   + Đăng ký ngắt PLIC (SPI_0_SSI_INTR = 37) và đồng bộ tiến trình qua Semaphore.
 *   + Lập trình bộ điều khiển System DMA (DW_axi_dmac tại 0x04330000), ánh xạ kênh phần cứng
 *     và đồng bộ bộ nhớ đệm D-Cache (Clean / Invalidate) trên nhân RISC-V C906L.
 *
 * - Tầng 1 (hal_dw_spi.c):
 *   Quản lý trực tiếp máy trạng thái và hệ thống thanh ghi của bộ điều khiển DW APB SSI:
 *   + Tính toán bộ chia tần số chẵn BAUDR, cấu hình CTRLR0 (Mode 0..3, 4..16 bit/khung),
 *     độ trễ lấy mẫu RX_SAMPLE_DLY và hỗ trợ đảo bit phần mềm cho chế độ LSB-first.
 *   + Cung cấp 3 cơ chế truyền nhận cho từng phân đoạn (segment):
 *     1. hal_spi_poll()         : Bơm FIFO trực tiếp bằng CPU (Polling).
 *     2. hal_spi_transfer_irq() : Truyền nhận qua ngắt FIFO (TXEI / RXFI) cùng hal_spi_isr().
 *     3. hal_spi_transfer_dma() : Truyền nhận tốc độ cao qua bắt tay phần cứng DMA.
 *   + Cung cấp cơ chế hủy và khôi phục khẩn cấp hal_spi_abort() khi phát hiện lỗi.
 */

#ifndef HAL_DW_SPI_H
#define HAL_DW_SPI_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Mã kết quả trả về của Tầng 1 */
typedef enum {
    HAL_SPI_OK = 0,               /* Hoàn tất thành công */
    HAL_SPI_PENDING = 1,          /* Phân đoạn đang thực thi dở dang, cần gọi tiếp */
    HAL_SPI_ERR_INVALID = -1,     /* Tham số đầu vào hoặc cấu hình không hợp lệ */
    HAL_SPI_ERR_BUSY = -2,        /* Bộ điều khiển đang bận, không thể cấu hình lại */
    HAL_SPI_ERR_FIFO = -3,        /* Lỗi phần cứng tràn hoặc đọc rỗng FIFO */
    HAL_SPI_ERR_TIMEOUT = -4,     /* Quá thời gian chờ hoàn tất truyền nhận */
    HAL_SPI_ERR_UNSUPPORTED = -5, /* Tính năng không được hỗ trợ */
} hal_spi_status_t;

/*
 * Cấu hình phần cứng của một thiết bị SPI ở Tầng 1.
 * Hai trường cuối (rx_sample_dly, lsb_first) nếu không khai báo sẽ mặc định bằng 0 / false.
 */
typedef struct {
    uint32_t sck_hz;        /* Tần số xung nhịp SCK mong muốn (Hz) */
    uint8_t mode;           /* Chế độ SPI chuẩn Motorola (0..3: bit 0 = CPHA, bit 1 = CPOL) */
    uint8_t bits_per_word;  /* Số bit trên mỗi khung dữ liệu (hỗ trợ từ 4 đến 16 bit) */
    uint8_t rx_sample_dly;  /* Độ trễ lấy mẫu MISO tính theo chu kỳ ssi_clk (0..8) */
    bool lsb_first;         /* false = chuẩn MSB-first, true = đảo bit sang LSB-first */
} hal_spi_config_t;

/*
 * Ảnh chụp tức thời (Snapshot) các thanh ghi phần cứng phục vụ chẩn đoán khi có lỗi.
 */
typedef struct {
    uint32_t ctrlr0;        /* Thanh ghi điều khiển khung truyền CTRLR0 (0x000) */
    uint32_t baudr;         /* Thanh ghi hệ số chia xung nhịp BAUDR (0x014) */
    uint32_t sr;            /* Thanh ghi trạng thái SR (0x028) */
    uint32_t risr;          /* Thanh ghi trạng thái ngắt thô RISR (0x034) */
    uint32_t txflr;         /* Số khung còn trong TX FIFO (0x020) */
    uint32_t rxflr;         /* Số khung còn trong RX FIFO (0x024) */
    uint32_t imr;           /* Mặt nạ ngắt IMR (0x02C) */
    uint32_t dmacr;         /* Thanh ghi điều khiển DMA DMACR (0x04C) */
    uint32_t rx_sample_dly; /* Thanh ghi trễ lấy mẫu RX_SAMPLE_DLY (0x0F0) */
} hal_spi_snapshot_t;

/* Kiểu con trỏ hàm xử lý ngắt đăng ký từ Tầng 1 xuống Tầng 0 */
typedef void (*hal_spi_irq_handler_t)(void *ctx);

/*
 * Cấu trúc trạng thái vận hành của bộ điều khiển Tầng 1.
 * Được cấp phát tĩnh bên trong đối tượng bus ở Tầng 2, không sử dụng bộ nhớ Heap.
 */
typedef struct {
    uintptr_t reg_base;          /* Địa chỉ cơ sở MMIO của bộ điều khiển SPI (0x04180000) */
    uint32_t ref_clock_hz;       /* Xung nhịp tham chiếu đầu vào ssi_clk (187.5 MHz) */
    uint32_t actual_sck_hz;      /* Tần số SCK thực tế sau khi áp dụng bộ chia chẵn */
    const void *tx;              /* Bộ đệm phát của phân đoạn hiện tại (NULL nếu chỉ nhận) */
    void *rx;                    /* Bộ đệm nhận của phân đoạn hiện tại (NULL nếu chỉ phát) */
    size_t frames;               /* Tổng số khung cần trao đổi trong phân đoạn hiện tại */
    size_t tx_done;              /* Số khung đã đẩy vào TX FIFO */
    size_t rx_done;              /* Số khung đã rút ra khỏi RX FIFO */
    uint16_t fill_word;          /* Giá trị khung nhồi phát ra trên MOSI khi tx == NULL */
    uint8_t bits_per_word;       /* Độ rộng khung dữ liệu hiện tại (4..16 bit) */
    uint8_t frame_bytes;         /* Số byte mỗi khung trong RAM (1 byte cho <=8b, 2 byte cho 9..16b) */
    uint8_t mode;                /* SPI Mode hiện tại (0..3) */
    uint8_t rx_sample_dly;       /* Độ trễ lấy mẫu MISO hiện tại (0..8) */
    bool lsb_first;              /* Cờ bật chế độ đảo bit LSB-first */
    bool configured;             /* Đánh dấu bộ điều khiển đã được cấu hình hợp lệ */
    bool active;                 /* Đánh dấu đang có một phân đoạn đang chạy */
    bool irq_ready;              /* Chỉ true sau prepare_irq thành công */
    bool dma_ready;              /* Chỉ true sau prepare_dma thành công */
    volatile int32_t irq_status; /* Mã kết quả truyền nhận được cập nhật từ bên trong ISR */
} hal_spi_t;

/* --------------------------------------------------------------------------
 * Nhóm hàm Tầng 0 (Platform Hooks - triển khai trong hal_spi_platform.c)
 * -------------------------------------------------------------------------- */

/* Bật xung nhịp phần cứng cho bộ điều khiển SPI */
hal_spi_status_t hal_spi_platform_clock_enable(uintptr_t reg_base);

/* Phát xung Reset mềm để đưa khối IP SPI về trạng thái mặc định */
hal_spi_status_t hal_spi_platform_reset(uintptr_t reg_base);

/* Mở khóa MIPI D-PHY TTL và cấu hình Pinmux / IOBLK cho các chân SPI0 */
hal_spi_status_t hal_spi_platform_pinmux_init(void);

/* Khởi tạo chân GPIO Chip-Select (GPIOC17) ở trạng thái nhả (mức HIGH) */
void hal_spi_platform_cs_init(void);

/* Điều khiển chân Chip-Select: asserted = true -> kéo xuống LOW, false -> kéo lên HIGH */
void hal_spi_platform_cs_set(bool asserted);

/* Clock tree readback; FPLL của board phải là 1.5 GHz. Trả 0 nếu divider reset/invalid. */
uint32_t hal_spi_platform_get_ref_clock_hz(void);

/* Đăng ký hàm xử lý ngắt của Tầng 1 với bộ điều khiển ngắt PLIC trên C906L */
hal_spi_status_t hal_spi_platform_irq_init(uintptr_t reg_base,
                                           hal_spi_irq_handler_t handler,
                                           void *ctx);

/* Đưa Task hiện tại vào trạng thái ngủ chờ đến khi ISR thông báo hoàn tất hoặc quá hạn */
hal_spi_status_t hal_spi_platform_irq_wait(uintptr_t reg_base, uint32_t timeout_ms);

/* Đánh thức Task đang chờ từ bên trong ngữ cảnh ngắt (ISR) */
void hal_spi_platform_irq_notify(uintptr_t reg_base);

/* Khởi tạo bộ điều khiển System DMA (DW_axi_dmac) và bảng ánh xạ kênh cho SPI0 */
/* resources_reserved là cam kết của board: đã dành channel 0/1 và quyền
 * cấu hình DMAC/remap/clock cho SPI. Không tự khóa Linux hoặc sửa DTS. */
hal_spi_status_t hal_spi_platform_dma_init(uintptr_t reg_base, bool resources_reserved);

/* Thực thi truyền nhận song công qua phần cứng System DMA kèm đồng bộ D-Cache */
hal_spi_status_t hal_spi_platform_dma_transfer(uintptr_t reg_base,
                                               const void *tx_buf,
                                               void *rx_buf,
                                               size_t frames,
                                               uint8_t frame_bytes,
                                               uint16_t fill_word,
                                               uint32_t timeout_ms);

/* Cưỡng bức dừng ngay lập tức các kênh DMA của SPI0 để bảo vệ bộ nhớ người dùng */
void hal_spi_platform_dma_abort(uintptr_t reg_base);

/* --------------------------------------------------------------------------
 * Nhóm hàm Tầng 1 (Controller HAL - triển khai trong hal_dw_spi.c)
 * -------------------------------------------------------------------------- */

/* Khởi tạo SPI cơ bản; chưa đăng ký IRQ và chưa chạm tài nguyên DMAC. */
hal_spi_status_t hal_spi_init(hal_spi_t *hal, uintptr_t reg_base, uint32_t ref_clock_hz);

/* Gọi khi controller idle, trước begin/CS. Lỗi được trả lên caller;
 * chuẩn bị thành công được giữ lại cho các message sau. */
hal_spi_status_t hal_spi_prepare_irq(hal_spi_t *hal);
hal_spi_status_t hal_spi_prepare_dma(hal_spi_t *hal, bool resources_reserved);

/* Kiểm tra tính hợp lệ của thông số cấu hình mà không ghi vào thanh ghi phần cứng */
hal_spi_status_t hal_spi_validate_config(const hal_spi_t *hal, const hal_spi_config_t *cfg);

/* Áp dụng cấu hình tốc độ SCK, Mode, số bit/khung và độ trễ lấy mẫu vào thanh ghi */
hal_spi_status_t hal_spi_configure(hal_spi_t *hal, const hal_spi_config_t *cfg);

/* Chuẩn bị FIFO, xóa cờ lỗi và kích hoạt SER = 1 để bắt đầu một phân đoạn truyền nhận */
hal_spi_status_t hal_spi_begin(hal_spi_t *hal,
                               const void *tx,
                               void *rx,
                               size_t frames,
                               uint16_t fill_word);

/* Cơ chế 1 (Polling): Bơm một lượt dữ liệu qua TX/RX FIFO bằng CPU */
hal_spi_status_t hal_spi_poll(hal_spi_t *hal);

/* Hàm phục vụ ngắt (ISR) của Tầng 1: rút RX FIFO, nạp TX FIFO và điều chỉnh ngưỡng ngắt */
void hal_spi_isr(hal_spi_t *hal);

/* Cơ chế 2 (Interrupt): Truyền nhận toàn bộ phân đoạn bằng ngắt phần cứng */
hal_spi_status_t hal_spi_transfer_irq(hal_spi_t *hal, uint32_t timeout_ms);

/* Cơ chế 3 (DMA): Truyền nhận toàn bộ phân đoạn bằng phần cứng System DMA */
hal_spi_status_t hal_spi_transfer_dma(hal_spi_t *hal, uint32_t timeout_ms);

/* Đóng phân đoạn hiện tại sau khi đã truyền nhận hoàn tất (tắt IMR, DMACR, SER) */
void hal_spi_finish(hal_spi_t *hal);

/* Đọc trạng thái tức thời của các thanh ghi phần cứng vào cấu trúc snapshot */
void hal_spi_snapshot(const hal_spi_t *hal, hal_spi_snapshot_t *out);

/* Hủy khẩn cấp giao dịch đang chạy, dừng DMA/IRQ, Reset FIFO, nhả CS và khôi phục cấu hình */
void hal_spi_abort(hal_spi_t *hal);

#endif /* HAL_DW_SPI_H */
