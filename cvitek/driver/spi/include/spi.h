/*
 * Tầng 2 (T2 - Public RTOS SPI Driver API): Giao diện lập trình Bus & Thiết bị SPI
 * dành cho các Driver Cảm biến ngoại vi (Tầng 3).
 *
 * Tác dụng: Cung cấp API an toàn đa luồng (FreeRTOS Static Mutex), quản lý giao dịch
 * đa phân đoạn, điều khiển chân Chip-Select, tự động chọn Backend (POLL/IRQ/DMA/AUTO)
 * và tự động thu hồi lỗi phần cứng.
 */

#ifndef CVI_DRIVER_SPI_H
#define CVI_DRIVER_SPI_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "semphr.h"

#if defined(__has_include) && __has_include("hal_dw_spi.h")
#include "hal_dw_spi.h"
#else
#include "../../hal/spi/hal_dw_spi.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* --------------------------------------------------------------------------
 * 1. Hằng số mặc định & Mã trạng thái trả về của Tầng 2
 * -------------------------------------------------------------------------- */

/* Mã định danh chân Chip-Select mặc định (điều khiển qua GPIOC17 trên cổng FPC) */
#define SPI_CS0_GPIO                        0U

/* Ngưỡng số khung mặc định để chế độ SPI_BACKEND_AUTO chuyển sang dùng IRQ và DMA */
#define SPI_DEFAULT_IRQ_THRESHOLD_FRAMES    32U
#define SPI_DEFAULT_DMA_THRESHOLD_FRAMES    64U

/* Mã kết quả trả về của các hàm spi_* */
typedef enum {
    SPI_OK = 0,               /* Giao dịch hoàn tất thành công */
    SPI_ERR_INVALID = -1,     /* Tham số cấu hình hoặc mô tả phân đoạn không hợp lệ */
    SPI_ERR_BUSY = -2,        /* Không chiếm được khóa Mutex của Bus trong thời gian chờ */
    SPI_ERR_FIFO = -3,        /* Lỗi phần cứng tràn/rỗng FIFO (Bus đã được tự động thu hồi) */
    SPI_ERR_TIMEOUT = -4,     /* Quá thời gian chờ hoàn tất truyền nhận (Bus đã được thu hồi) */
    SPI_ERR_UNSUPPORTED = -5, /* Chế độ hoặc tính năng không được hỗ trợ */
} spi_status_t;

/* Các cơ chế thực thi truyền nhận dữ liệu của Tầng 2 */
typedef enum {
    SPI_BACKEND_POLL = 0,     /* Luôn truyền bằng CPU Polling */
    SPI_BACKEND_IRQ = 1,      /* Luôn truyền bằng Ngắt phần cứng (Interrupt + Semaphore) */
    SPI_BACKEND_DMA = 2,      /* Luôn truyền bằng System DMA (DW_axi_dmac + D-Cache sync) */
    SPI_BACKEND_AUTO = 3,     /* Tự động chọn POLL / IRQ / DMA theo kích thước từng phân đoạn */
} spi_backend_t;

/* --------------------------------------------------------------------------
 * 2. Cấu trúc cấu hình Bus, Thiết bị và Phân đoạn giao dịch
 * -------------------------------------------------------------------------- */

/* Tác dụng: Cấu hình khởi tạo một Bus SPI vật lý và cấp quyền cho các backend IRQ/DMA */
typedef struct {
    uintptr_t reg_base;            /* Địa chỉ cơ sở thanh ghi SPI (0x04180000 cho SPI0) */
    uint32_t ref_clock_hz;         /* Xung nhịp gốc ssi_clk (truyền 0 để đọc từ clock tree T0) */
    uint16_t irq_threshold_frames; /* Ngưỡng khung bật IRQ khi dùng AUTO (0 = mặc định 32) */
    uint16_t dma_threshold_frames; /* Ngưỡng khung bật DMA khi dùng AUTO (0 = mặc định 64) */
    bool enable_irq;               /* Cho phép sử dụng ngắt PLIC */
    bool enable_dma;               /* Cho phép sử dụng System DMA */
    bool dma_resources_reserved;   /* Xác nhận board đã dành riêng kênh DMA 0/1 cho SPI0 */
} spi_bus_config_t;

/* Tác dụng: Cấu hình thông số truyền dẫn khi gắn một Thiết bị ngoại vi vào Bus SPI */
typedef struct {
    uint32_t sck_hz;        /* Tần số xung nhịp SCK tối đa thiết bị cho phép (Hz) */
    uint8_t mode;           /* Chế độ SPI Motorola (0..3: Mode 0, Mode 1, Mode 2, Mode 3) */
    uint8_t bits_per_word;  /* Độ rộng mỗi khung dữ liệu tính bằng bit (4..16 bit) */
    uint8_t cs_id;          /* Chân Chip-Select sử dụng (mặc định SPI_CS0_GPIO = 0) */
    spi_backend_t backend;  /* Cơ chế truyền nhận: POLL, IRQ, DMA, hoặc AUTO */
    uint8_t rx_sample_dly;  /* Độ trễ lấy mẫu MISO tính theo chu kỳ ssi_clk (0..8) */
    bool lsb_first;         /* false = MSB-first (mặc định), true = LSB-first */
} spi_device_config_t;

/* Tác dụng: Mô tả một phân đoạn (Segment) trong giao dịch SPI đa phân đoạn */
typedef struct {
    const void *tx;     /* Bộ đệm phát (hoặc NULL nếu phân đoạn này chỉ nhận dữ liệu) */
    void *rx;           /* Bộ đệm nhận (hoặc NULL nếu phân đoạn này chỉ phát dữ liệu) */
    size_t frames;      /* Số lượng khung dữ liệu cần trao đổi trong phân đoạn (phải > 0) */
    uint16_t fill_word; /* Giá trị nhồi phát trên MOSI khi tx == NULL (thường là 0xFF hoặc 0x00) */
    uint16_t delay_us;  /* Khoảng nghỉ (micro-giây) sau khi phân đoạn này hoàn tất */
    bool cs_change;     /* true = Nhả CS lên HIGH một nhịp ngắn sau phân đoạn này rồi kéo LOW lại */
} spi_segment_t;

typedef struct spi_bus spi_bus_t;

/* Tác dụng: Đại diện cho một Thiết bị ngoại vi đã đăng ký trên Bus SPI */
typedef struct {
    spi_bus_t *bus;           /* Con trỏ tới Bus SPI chủ quản */
    hal_spi_config_t hal_cfg; /* Cấu hình phần cứng Tầng 1 của thiết bị */
    uint8_t cs_id;            /* Chân Chip-Select của thiết bị */
    spi_backend_t backend;    /* Cơ chế truyền nhận đang áp dụng */
    bool initialized;         /* Đánh dấu thiết bị đã được khởi tạo hợp lệ */
} spi_device_t;

/* Tác dụng: Quản lý trạng thái runtime, khóa Mutex và thống kê của một Bus SPI vật lý */
struct spi_bus {
    hal_spi_t hal;                    /* Đối tượng điều khiển phần cứng Tầng 1 */
    StaticSemaphore_t mutex_storage;  /* Bộ nhớ tĩnh của khóa Mutex bảo vệ Bus */
    SemaphoreHandle_t mutex;          /* Handle khóa Mutex của FreeRTOS */
    const spi_device_t *active_dev;   /* Thiết bị vừa giao tiếp gần nhất trên Bus */
    hal_spi_snapshot_t last_snapshot; /* Ảnh chụp thanh ghi tại thời điểm gần nhất */
    spi_status_t last_error;          /* Mã kết quả / lỗi gần nhất */
    uint16_t irq_threshold_frames;    /* Ngưỡng khung chuyển sang IRQ của chế độ AUTO */
    uint16_t dma_threshold_frames;    /* Ngưỡng khung chuyển sang DMA của chế độ AUTO */
    uint32_t poll_segment_count;      /* Số phân đoạn đã chạy bằng Polling */
    uint32_t irq_segment_count;       /* Số phân đoạn đã chạy bằng Interrupt */
    uint32_t dma_segment_count;       /* Số phân đoạn đã chạy bằng DMA */
    uint32_t error_count;             /* Tổng số lần xảy ra lỗi phải thu hồi Bus */
    bool enable_irq;                  /* Quyền sử dụng IRQ của Bus */
    bool enable_dma;                  /* Quyền sử dụng DMA của Bus */
    bool dma_resources_reserved;      /* Trạng thái cấp quyền kênh DMA của board */
    bool initialized;                 /* Đánh dấu Bus đã được khởi tạo hợp lệ */
};

/* --------------------------------------------------------------------------
 * 3. Các hàm khởi tạo, quản lý và cấu hình động Bus / Thiết bị
 * -------------------------------------------------------------------------- */

/* Tác dụng: Khởi tạo Bus SPI, tạo static mutex bảo vệ và liên kết tài nguyên T0/T1 */
spi_status_t spi_bus_init(spi_bus_t *bus, const spi_bus_config_t *cfg);

/* Tác dụng: Đưa Bus SPI về trạng thái nghỉ an toàn và đóng trạng thái khởi tạo */
spi_status_t spi_bus_deinit(spi_bus_t *bus);

/* Tác dụng: Cưỡng bức thu hồi và làm mới Bus SPI an toàn đa luồng khi ngoại vi gặp sự cố */
spi_status_t spi_bus_recover(spi_bus_t *bus);

/* Tác dụng: Gắn và kiểm tra tính hợp lệ của một Thiết bị ngoại vi trên Bus SPI */
spi_status_t spi_device_init(spi_device_t *dev,
                             spi_bus_t *bus,
                             const spi_device_config_t *cfg);

/* Tác dụng: Thay đổi động cơ chế truyền nhận (POLL, IRQ, DMA, AUTO) cho thiết bị */
spi_status_t spi_device_set_backend(spi_device_t *dev, spi_backend_t backend);

/* Tác dụng: Đổi nóng tần số xung nhịp SCK và độ trễ lấy mẫu MISO của thiết bị lúc đang chạy */
spi_status_t spi_device_set_freq(spi_device_t *dev,
                                 uint32_t sck_hz,
                                 uint8_t rx_sample_dly);

/* --------------------------------------------------------------------------
 * 4. Các hàm truyền nhận dữ liệu dành cho Driver Cảm biến (Tầng 3)
 * -------------------------------------------------------------------------- */

/* Tác dụng: Thực thi giao dịch SPI nguyên tử đa phân đoạn, giữ/nhả CS và tự phục hồi lỗi */
spi_status_t spi_transfer(spi_device_t *dev,
                          const spi_segment_t *segments,
                          size_t count,
                          uint32_t timeout_ms);

/* Tác dụng: Truyền nhận 1 phân đoạn đơn lẻ (Full-Duplex, TX-only hoặc RX-only) */
spi_status_t spi_transfer_one(spi_device_t *dev,
                              const void *tx,
                              void *rx,
                              size_t frames,
                              uint16_t fill_word,
                              uint32_t timeout_ms);

/* Tác dụng: Gửi gói lệnh (tx_cmd), chờ delay_us, rồi đọc dữ liệu (rx_data) trong 1 lần giữ CS */
spi_status_t spi_write_then_read(spi_device_t *dev,
                                 const void *tx_cmd,
                                 size_t tx_frames,
                                 void *rx_data,
                                 size_t rx_frames,
                                 uint16_t delay_us,
                                 uint32_t timeout_ms);

/* Tác dụng: Chỉ phát dữ liệu (TX-only), bỏ qua dữ liệu thu về trên đường MISO */
static inline spi_status_t spi_write(spi_device_t *dev,
                                     const void *tx,
                                     size_t frames,
                                     uint32_t timeout_ms)
{
    if (tx == NULL) {
        return SPI_ERR_INVALID;
    }
    return spi_transfer_one(dev, tx, NULL, frames, 0U, timeout_ms);
}

/* Tác dụng: Chỉ đọc dữ liệu (RX-only), tự động phát từ nhồi fill_word trên đường MOSI */
static inline spi_status_t spi_read(spi_device_t *dev,
                                    void *rx,
                                    size_t frames,
                                    uint16_t fill_word,
                                    uint32_t timeout_ms)
{
    if (rx == NULL) {
        return SPI_ERR_INVALID;
    }
    return spi_transfer_one(dev, NULL, rx, frames, fill_word, timeout_ms);
}

#ifdef __cplusplus
}
#endif

#endif /* CVI_DRIVER_SPI_H */
