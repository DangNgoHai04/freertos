/* Clock/reset và board I/O nằm trong hal_spi_platform_io.c.
 * IRQ/DMA chỉ được chuẩn bị khi T1 yêu cầu
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "arch_helpers.h"
#include "dw_spi_regs.h"
#include "hal_dw_spi.h"
#include "hal_spi_platform_limits.h"
#include "intr_conf.h"
#include "mmio.h"
#include "top_reg.h"

/* --------------------------------------------------------------------------
 * 1. Địa chỉ thanh ghi hệ thống SoC SG2002 (Clock, Reset, D-PHY, Pinmux, GPIO)
 * -------------------------------------------------------------------------- */

#define SPI0_BASE_ADDR                  0x04180000UL

/* Tài nguyên SDMA, chỉ dùng trong prepare_dma có reservation. */
#define CLK_EN_1_REG                    0x03002004UL
#define CLK_EN_1_SDMA_AXI_BIT           (1U << 1)
#define RESET_BANK0_REG                 0x03003000UL
#define RESET_BANK0_SDMA_BIT            (1U << 18)

/* --------------------------------------------------------------------------
 * 2. Địa chỉ thanh ghi bộ điều khiển System DMA (Synopsys DW_axi_dmac)
 * -------------------------------------------------------------------------- */

/* Thanh ghi ánh xạ kênh DMA và định tuyến ngắt nằm trong khối TOP (0x03000000) */
#define TOP_SDMA_CH_REMAP0_REG          (TOP_BASE + 0x0154UL)
#define TOP_SDMA_INT_MUX_REG            (TOP_BASE + 0x0298UL)
#define SDMA_REMAP_UPDATE_BIT           (1U << 31)  /* Bit 31: Chốt cấu hình ánh xạ kênh */
#define SDMA_INT_MUX_C906L_SHIFT        20U         /* Bits [28:20] định tuyến ngắt DMA về C906L */

/* Mã yêu cầu phần cứng (Hardware Handshake ID) của SPI0 theo TRM Bảng 9-1 */
#define CVI_DMA_REQ_SPI0_RX             16U         /* dma_rx_req_spi0 = ID 16 */
#define CVI_DMA_REQ_SPI0_TX             17U         /* dma_tx_req_spi0 = ID 17 */

/* Phân bổ Kênh DMA 0 cho chiều Nhận (RX) và Kênh DMA 1 cho chiều Phát (TX) */
#define SPI0_DMA_RX_CHAN                0U
#define SPI0_DMA_TX_CHAN                1U

/* Khối thanh ghi trung tâm của DW AXI DMAC (0x04330000) */
#define DW_DMAC_BASE_ADDR               0x04330000UL
#define DW_DMAC_CFG_OFF                 0x0010UL    /* Thanh ghi cấu hình chung DMAC_CFG */
#define DW_DMAC_CHEN_L_OFF              0x0018UL    /* Thanh ghi bật/tắt kênh (32-bit thấp) */
#define DW_DMAC_CHEN_H_OFF              0x001CUL    /* Thanh ghi hủy kênh khẩn cấp (32-bit cao) */

#define DW_DMAC_CFG_EN                  (1U << 0)   /* Bit 0: Bật bộ điều khiển DMAC */
#define DW_DMAC_CFG_INT_EN              (1U << 1)   /* Bit 1: Cho phép phát tín hiệu ngắt DMAC */

/* Địa chỉ cơ sở của từng Kênh DMA x (ch = 0..7): 0x04330100 + ch * 0x100 */
#define DW_DMAC_CH_BASE(ch)             (DW_DMAC_BASE_ADDR + 0x0100UL + ((uintptr_t)(ch) * 0x0100UL))

#define DW_DMAC_CH_SAR_L_OFF            0x0000UL    /* Địa chỉ nguồn (32-bit thấp) */
#define DW_DMAC_CH_SAR_H_OFF            0x0004UL    /* Địa chỉ nguồn (32-bit cao) */
#define DW_DMAC_CH_DAR_L_OFF            0x0008UL    /* Địa chỉ đích (32-bit thấp) */
#define DW_DMAC_CH_DAR_H_OFF            0x000CUL    /* Địa chỉ đích (32-bit cao) */
#define DW_DMAC_CH_BLOCK_TS_OFF         0x0010UL    /* Số phần tử trong block trừ 1 (frames - 1) */
#define DW_DMAC_CH_CTL_L_OFF            0x0018UL    /* Điều khiển kênh (32-bit thấp) */
#define DW_DMAC_CH_CTL_H_OFF            0x001CUL    /* Điều khiển kênh (32-bit cao) */
#define DW_DMAC_CH_CFG_L_OFF            0x0020UL    /* Cấu hình kênh (32-bit thấp) */
#define DW_DMAC_CH_CFG_H_OFF            0x0024UL    /* Cấu hình kênh (32-bit cao) */
#define DW_DMAC_CH_LLP_L_OFF            0x0028UL    /* Con trỏ danh sách liên kết LLI (thấp) */
#define DW_DMAC_CH_LLP_H_OFF            0x002CUL    /* Con trỏ danh sách liên kết LLI (cao) */
#define DW_DMAC_CH_STATUS_L_OFF         0x0030UL    /* Trạng thái kênh */
#define DW_DMAC_CH_INTSTATUS_EN_L_OFF   0x0080UL    /* Cho phép cập nhật các cờ trạng thái kênh */
#define DW_DMAC_CH_INTSTATUS_L_OFF      0x0088UL    /* Thanh ghi cờ trạng thái của kênh */
#define DW_DMAC_CH_INTSIGNAL_EN_L_OFF   0x0090UL    /* Cho phép xuất tín hiệu ngắt ra ngoài */
#define DW_DMAC_CH_INTCLEAR_L_OFF       0x0098UL    /* Ghi 1 để xóa các cờ trạng thái kênh */

/* Các trường điều khiển trong thanh ghi CH_CTL_L */
#define DW_DMAC_CTL_L_SINC_INC          (0U << 4)   /* Tự động tăng địa chỉ nguồn sau mỗi phần tử */
#define DW_DMAC_CTL_L_SINC_NOCHANGE     (1U << 4)   /* Giữ nguyên địa chỉ nguồn */
#define DW_DMAC_CTL_L_DINC_INC          (0U << 6)   /* Tự động tăng địa chỉ đích sau mỗi phần tử */
#define DW_DMAC_CTL_L_DINC_NOCHANGE     (1U << 6)   /* Giữ nguyên địa chỉ đích */
#define DW_DMAC_CTL_L_SRC_WIDTH(w)      (((uint32_t)(w) & 0x7U) << 8)
#define DW_DMAC_CTL_L_DST_WIDTH(w)      (((uint32_t)(w) & 0x7U) << 11)
#define DW_DMAC_CTL_L_SRC_MSIZE(m)      (((uint32_t)(m) & 0xFU) << 14)
#define DW_DMAC_CTL_L_DST_MSIZE(m)      (((uint32_t)(m) & 0xFU) << 18)

/* Các trường điều khiển trong thanh ghi CH_CTL_H */
#define DW_DMAC_CTL_H_ARLEN_EN          (1U << 6)
#define DW_DMAC_CTL_H_ARLEN(v)          (((uint32_t)(v) & 0xFFU) << 7)
#define DW_DMAC_CTL_H_AWLEN_EN          (1U << 15)
#define DW_DMAC_CTL_H_AWLEN(v)          (((uint32_t)(v) & 0xFFU) << 16)
#define DW_DMAC_CTL_H_LLI_LAST          (1U << 30)
#define DW_DMAC_CTL_H_LLI_VALID         (1U << 31)

/* Cấu hình truyền đơn khối trực tiếp qua thanh ghi (SRC_MULTBLK_TYPE=0, DST_MULTBLK_TYPE=0) */
#define DW_DMAC_CFG_L_SINGLE_BLOCK      0x00000000U

/* Các trường cấu hình chiều truyền và bắt tay phần cứng trong CH_CFG_H */
#define DW_DMAC_CFG_H_TT_FC_MEM2PER     (1U << 0)   /* Bộ nhớ -> Ngoại vi (DMA điều khiển luồng) */
#define DW_DMAC_CFG_H_TT_FC_PER2MEM     (2U << 0)   /* Ngoại vi -> Bộ nhớ (DMA điều khiển luồng) */
#define DW_DMAC_CFG_H_HS_SEL_SRC_HW     (0U << 3)   /* Bắt tay phần cứng cho phía Nguồn */
#define DW_DMAC_CFG_H_HS_SEL_DST_HW     (0U << 4)   /* Bắt tay phần cứng cho phía Đích */
#define DW_DMAC_CFG_H_SRC_PER(id)       (((uint32_t)(id) & 0xFU) << 7)
#define DW_DMAC_CFG_H_DST_PER(id)       (((uint32_t)(id) & 0xFU) << 12)
#define DW_DMAC_CFG_H_PRIO(p)           (((uint32_t)(p) & 0x7U) << 17)

/* Các cờ trạng thái hoàn tất và lỗi trong CH_INTSTATUS_L */
#define DW_DMAC_INT_BLOCK_DONE          (1U << 0)   /* Đã truyền xong 1 Block */
#define DW_DMAC_INT_DMA_DONE            (1U << 1)   /* Đã hoàn tất toàn bộ phiên DMA */
#define DW_DMAC_INT_ERR_MASK            0x003FFFC0U /* Mặt nạ các lỗi truy cập bus AXI */
#define DW_DMAC_INT_ALL_MASK            0xFFFFFFFFU

/* Độ dài một dòng bộ nhớ đệm D-Cache trên nhân RISC-V C906 là 64 byte */
#define C906_CACHE_LINE_BYTES           64UL

/* --------------------------------------------------------------------------
 * 3. Biến trạng thái nội bộ của Tầng 0 (Cấp phát tĩnh)
 * -------------------------------------------------------------------------- */

typedef struct {
    StaticSemaphore_t irq_sem_buf;     /* Bộ nhớ tĩnh của Binary Semaphore đồng bộ ngắt */
    SemaphoreHandle_t irq_sem;         /* Handle của Binary Semaphore */
    hal_spi_irq_handler_t irq_handler; /* Con trỏ hàm ISR của Tầng 1 (hal_spi_isr) */
    void *irq_ctx;                     /* Con trỏ đối tượng hal_spi_t truyền vào ISR */
    volatile bool irq_signaled;        /* Cờ báo hoàn tất dự phòng khi Scheduler chưa chạy */
    bool irq_registered;               /* Đánh dấu đã đăng ký vector ngắt với PLIC */
    bool dma_initialized;              /* Đánh dấu đã khởi tạo phần cứng System DMA */
} spi0_platform_state_t;

// g_spi0_plat là biến trạng thái tĩnh toàn cục của Tầng 0, dùng để quản lý các tài nguyên phần cứng
static spi0_platform_state_t g_spi0_plat;

/*
 * Hai dòng bộ nhớ nháp căn lề 64-byte độc lập dùng cho DMA khi truyền một chiều:
 * - g_dma_tx_dummy_line: Chứa giá trị fill_word để DMA TX đọc lặp lại khi chỉ nhận (tx_buf == NULL).
 * - g_dma_rx_dummy_line: Làm nơi chứa dữ liệu bỏ đi cho DMA RX khi chỉ phát (rx_buf == NULL).
 * Việc tách thành 2 dòng cache 64-byte riêng biệt giúp tránh xung đột dòng cache (False-Sharing)
 * giữa chiều đọc và chiều ghi.
 */
static uint8_t g_dma_tx_dummy_line[C906_CACHE_LINE_BYTES] __attribute__((aligned(64)));
static uint8_t g_dma_rx_dummy_line[C906_CACHE_LINE_BYTES] __attribute__((aligned(64)));

/* --------------------------------------------------------------------------
 * 4. Các hàm phụ trợ đồng bộ bộ nhớ đệm D-Cache & tạo trễ ngắn
 * -------------------------------------------------------------------------- */

static void platform_short_delay(unsigned int loops)
{
    for (volatile unsigned int i = 0; i < loops; ++i) {
        __asm__ volatile("nop");
    }
}

/* Đẩy dữ liệu từ D-Cache của CPU xuống RAM vật lý trước khi DMA TX đọc dữ liệu */
static void platform_dcache_clean(const void *buf, size_t bytes)
{
    uintptr_t start;
    uintptr_t end;

    if (buf == NULL || bytes == 0U) {
        return;
    }
    start = ((uintptr_t)buf) & ~(C906_CACHE_LINE_BYTES - 1UL);
    end = (((uintptr_t)buf) + bytes + C906_CACHE_LINE_BYTES - 1UL) &
          ~(C906_CACHE_LINE_BYTES - 1UL);
    clean_dcache_range(start, end - start);
}

/*
 * Chuẩn bị vùng nhớ nhận trước khi DMA RX ghi vào RAM:
 * Thực hiện Flush (Clean + Invalidate) để bảo toàn dữ liệu của các biến lân cận
 * nằm chung dòng cache 64-byte nếu bộ đệm của người dùng không căn lề 64-byte.
 */
static void platform_dcache_prepare_rx(void *buf, size_t bytes)
{
    uintptr_t start;
    uintptr_t end;

    if (buf == NULL || bytes == 0U) {
        return;
    }
    start = ((uintptr_t)buf) & ~(C906_CACHE_LINE_BYTES - 1UL);
    end = (((uintptr_t)buf) + bytes + C906_CACHE_LINE_BYTES - 1UL) &
          ~(C906_CACHE_LINE_BYTES - 1UL);
    flush_dcache_range(start, end - start);
}

/* Vô hiệu hóa D-Cache sau khi DMA RX hoàn tất để buộc CPU đọc dữ liệu mới từ RAM */
static void platform_dcache_invalidate_rx(void *buf, size_t bytes)
{
    uintptr_t start;
    uintptr_t end;

    if (buf == NULL || bytes == 0U) {
        return;
    }
    start = ((uintptr_t)buf) & ~(C906_CACHE_LINE_BYTES - 1UL);
    end = (((uintptr_t)buf) + bytes + C906_CACHE_LINE_BYTES - 1UL) &
          ~(C906_CACHE_LINE_BYTES - 1UL);
    inv_dcache_range(start, end - start);
}

/* --------------------------------------------------------------------------
 * 5. Triển khai quản lý Ngắt phần cứng (PLIC IRQ cho chế độ SPI_BACKEND_IRQ)
 * -------------------------------------------------------------------------- */

static int spi0_plic_irq_trampoline(int irqn, void *priv)
{
    (void)irqn;
    (void)priv;
    if (g_spi0_plat.irq_handler != NULL) {
        g_spi0_plat.irq_handler(g_spi0_plat.irq_ctx);
    }
    return 0;
}

hal_spi_status_t hal_spi_platform_irq_init(uintptr_t reg_base,
                                           hal_spi_irq_handler_t handler,
                                           void *ctx)
{
    if (reg_base != SPI0_BASE_ADDR || handler == NULL) {
        return HAL_SPI_ERR_INVALID;
    }

    if (g_spi0_plat.irq_sem == NULL) {
        g_spi0_plat.irq_sem = xSemaphoreCreateBinaryStatic(&g_spi0_plat.irq_sem_buf);
        if (g_spi0_plat.irq_sem == NULL) {
            return HAL_SPI_ERR_INVALID;
        }
    }

    /* Xóa token completion cũ còn sót (nếu có) trước khi nhận phiên mới */
    while (xSemaphoreTake(g_spi0_plat.irq_sem, 0U) == pdTRUE) {
    }

    g_spi0_plat.irq_handler = handler;
    g_spi0_plat.irq_ctx = ctx;
    g_spi0_plat.irq_signaled = false;

    if (!g_spi0_plat.irq_registered) {
        int rc = request_irq(SPI_0_SSI_INTR,
                             spi0_plic_irq_trampoline,
                             0UL,
                             "spi0_dw_ssi",
                             &g_spi0_plat);
        if (rc != 0) {
            return HAL_SPI_ERR_INVALID;
        }
        g_spi0_plat.irq_registered = true;
    }

    return HAL_SPI_OK;
}

hal_spi_status_t hal_spi_platform_irq_wait(uintptr_t reg_base, uint32_t timeout_ms)
{
    uint32_t effective_ms = (timeout_ms == 0U) ? 1000U : timeout_ms;

    if (reg_base != SPI0_BASE_ADDR || g_spi0_plat.irq_sem == NULL) {
        return HAL_SPI_ERR_INVALID;
    }

    /* Khi bộ lập lịch FreeRTOS đang hoạt động, đưa Task vào trạng thái ngủ qua Semaphore */
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        TickType_t ticks = pdMS_TO_TICKS(effective_ms);
        if (ticks == 0U) {
            ticks = 1U;
        }
        if (xSemaphoreTake(g_spi0_plat.irq_sem, ticks) == pdTRUE) {
            g_spi0_plat.irq_signaled = false;
            return HAL_SPI_OK;
        }
        return HAL_SPI_ERR_TIMEOUT;
    }

    /* Dự phòng khi hàm được gọi trước lúc vTaskStartScheduler() khởi chạy */
    for (uint32_t spin = 0; spin < effective_ms * 20000U; ++spin) {
        if (g_spi0_plat.irq_signaled) {
            g_spi0_plat.irq_signaled = false;
            return HAL_SPI_OK;
        }
        __asm__ volatile("nop");
    }
    return HAL_SPI_ERR_TIMEOUT;
}

void hal_spi_platform_irq_notify(uintptr_t reg_base)
{
    BaseType_t higher_prio_woken = pdFALSE;

    if (reg_base != SPI0_BASE_ADDR) {
        return;
    }

    g_spi0_plat.irq_signaled = true;
    if (g_spi0_plat.irq_sem != NULL) {
        (void)xSemaphoreGiveFromISR(g_spi0_plat.irq_sem, &higher_prio_woken);
        portYIELD_FROM_ISR(higher_prio_woken);
    }
}

/* --------------------------------------------------------------------------
 * 6. Triển khai điều khiển System DMA (DW_axi_dmac cho chế độ SPI_BACKEND_DMA)
 * -------------------------------------------------------------------------- */

static void dmac_channel_enable(uint8_t ch)
{
    uint32_t val = (1U << ch) | (1U << (ch + 8U));
    mmio_write_32(DW_DMAC_BASE_ADDR + DW_DMAC_CHEN_L_OFF, val);
}

static void dmac_channel_disable(uint8_t ch)
{
    uint32_t disable_mask = (1U << (ch + 8U));
    mmio_write_32(DW_DMAC_BASE_ADDR + DW_DMAC_CHEN_L_OFF, disable_mask);

    if ((mmio_read_32(DW_DMAC_BASE_ADDR + DW_DMAC_CHEN_L_OFF) & (1U << ch)) != 0U) {
        uint32_t abort_val = (1U << ch) | (1U << (ch + 8U));
        mmio_write_32(DW_DMAC_BASE_ADDR + DW_DMAC_CHEN_H_OFF, abort_val);
    }

    for (unsigned int i = 0; i < 4096U; ++i) {
        if ((mmio_read_32(DW_DMAC_BASE_ADDR + DW_DMAC_CHEN_L_OFF) & (1U << ch)) == 0U) {
            break;
        }
        __asm__ volatile("nop");
    }
}

hal_spi_status_t hal_spi_platform_dma_init(uintptr_t reg_base, bool resources_reserved)
{
    uint32_t clk;
    uint32_t rst;
    uint32_t remap0;
    uint32_t int_mux;

    if (reg_base != SPI0_BASE_ADDR) {
        return HAL_SPI_ERR_INVALID;
    }

    /* Không được sửa clock/remap/DMAC khi board chưa cấp quyền sở hữu.
     * Cờ này là contract với board integration, không phải khóa liên lõi. */
    if (!resources_reserved)
        return HAL_SPI_ERR_UNSUPPORTED;
    if (g_spi0_plat.dma_initialized)
        return HAL_SPI_OK;

    /* 1. Cấp xung nhịp AXI và nhả tín hiệu Reset cho bộ điều khiển System DMA */
    clk = mmio_read_32(CLK_EN_1_REG);
    mmio_write_32(CLK_EN_1_REG, clk | CLK_EN_1_SDMA_AXI_BIT);

    rst = mmio_read_32(RESET_BANK0_REG);
    if ((rst & RESET_BANK0_SDMA_BIT) == 0U) {
        mmio_write_32(RESET_BANK0_REG, rst | RESET_BANK0_SDMA_BIT);
        platform_short_delay(64U);
    }

    /* Không chiếm hay hủy một transfer đang chạy trên DMAC dùng chung.
     * Board vẫn phải bảo đảm không có writer khác bắt đầu trong lúc init. */
    if ((mmio_read_32(DW_DMAC_BASE_ADDR + DW_DMAC_CHEN_L_OFF) & 0xFFU) != 0U)
        return HAL_SPI_ERR_BUSY;

    /*
     * 2. Lập trình bảng ánh xạ kênh DMA (sdma_dma_ch_remap0 tại 0x03000154):
     *    - Gán Kênh 0 (bits [5:0])  <- CVI_DMA_REQ_SPI0_RX (16)
     *    - Gán Kênh 1 (bits [13:8]) <- CVI_DMA_REQ_SPI0_TX (17)
     *    - Bật bit 31 (SDMA_REMAP_UPDATE_BIT) để phần cứng áp dụng cấu hình mới.
     */
    remap0 = mmio_read_32(TOP_SDMA_CH_REMAP0_REG);
    remap0 &= ~0x0000FFFFU;
    remap0 |= (CVI_DMA_REQ_SPI0_RX & 0x3FU) |
              (((CVI_DMA_REQ_SPI0_TX & 0x3FU) << 8)) |
              SDMA_REMAP_UPDATE_BIT;
    mmio_write_32(TOP_SDMA_CH_REMAP0_REG, remap0);

    /* 3. Định tuyến tín hiệu ngắt của Kênh 0 và Kênh 1 về nhân C906L (CPU2) */
    int_mux = mmio_read_32(TOP_SDMA_INT_MUX_REG);
    int_mux |= ((1U << SPI0_DMA_RX_CHAN) | (1U << SPI0_DMA_TX_CHAN) | (1U << 8))
               << SDMA_INT_MUX_C906L_SHIFT;
    mmio_write_32(TOP_SDMA_INT_MUX_REG, int_mux);

    /* 4. Bật DMAC sau khi đã xác nhận các channel idle; không hủy transfer
     * của một chủ sở hữu khác để lấy channel cho SPI. */
    mmio_write_32(DW_DMAC_BASE_ADDR + DW_DMAC_CFG_OFF, DW_DMAC_CFG_EN | DW_DMAC_CFG_INT_EN);
    g_spi0_plat.dma_initialized = true;
    return HAL_SPI_OK;
}

void hal_spi_platform_dma_abort(uintptr_t reg_base)
{
    if (reg_base != SPI0_BASE_ADDR || !g_spi0_plat.dma_initialized) {
        return;
    }

    dmac_channel_disable(SPI0_DMA_TX_CHAN);
    dmac_channel_disable(SPI0_DMA_RX_CHAN);

    mmio_write_32(DW_DMAC_CH_BASE(SPI0_DMA_RX_CHAN) + DW_DMAC_CH_INTCLEAR_L_OFF,
                  DW_DMAC_INT_ALL_MASK);
    mmio_write_32(DW_DMAC_CH_BASE(SPI0_DMA_TX_CHAN) + DW_DMAC_CH_INTCLEAR_L_OFF,
                  DW_DMAC_INT_ALL_MASK);
}

/* Thực thi một khối truyền nhận DMA song công (tối đa 4096 khung mỗi khối) */
static hal_spi_status_t dmac_run_single_block(uintptr_t reg_base,
                                              uintptr_t tx_src_addr,
                                              bool tx_inc,
                                              uintptr_t rx_dst_addr,
                                              bool rx_inc,
                                              size_t block_frames,
                                              uint8_t frame_bytes,
                                              uint32_t timeout_ms)
{
    uintptr_t rx_ch_base = DW_DMAC_CH_BASE(SPI0_DMA_RX_CHAN);
    uintptr_t tx_ch_base = DW_DMAC_CH_BASE(SPI0_DMA_TX_CHAN);
    uintptr_t spi_dr_addr = reg_base + DW_SPI_DR;
    uint32_t tr_width = (frame_bytes == 2U) ? 1U : 0U; /* 0 = 8-bit, 1 = 16-bit */
    uint32_t msize = 0U; /* MSIZE = 0 (1 phần tử/lần bắt tay) để không sót khung lẻ ở cuối gói */
    uint32_t rx_ctl_l;
    uint32_t tx_ctl_l;
    uint32_t ctl_h;
    uint32_t rx_cfg_h;
    uint32_t tx_cfg_h;
    uint32_t max_polls;

    /* 1. Xóa toàn bộ cờ trạng thái cũ của Kênh RX (Ch0) và Kênh TX (Ch1) */
    mmio_write_32(rx_ch_base + DW_DMAC_CH_INTCLEAR_L_OFF, DW_DMAC_INT_ALL_MASK);
    mmio_write_32(tx_ch_base + DW_DMAC_CH_INTCLEAR_L_OFF, DW_DMAC_INT_ALL_MASK);

    /* 2. Thiết lập Kênh RX: Đọc từ thanh ghi SPI0_DR và ghi vào bộ nhớ RAM */
    rx_ctl_l = DW_DMAC_CTL_L_SINC_NOCHANGE |
               (rx_inc ? DW_DMAC_CTL_L_DINC_INC : DW_DMAC_CTL_L_DINC_NOCHANGE) |
               DW_DMAC_CTL_L_SRC_WIDTH(tr_width) |
               DW_DMAC_CTL_L_DST_WIDTH(tr_width) |
               DW_DMAC_CTL_L_SRC_MSIZE(msize) |
               DW_DMAC_CTL_L_DST_MSIZE(msize);

    ctl_h = DW_DMAC_CTL_H_ARLEN_EN | DW_DMAC_CTL_H_ARLEN(0U) |
            DW_DMAC_CTL_H_AWLEN_EN | DW_DMAC_CTL_H_AWLEN(3U) |
            DW_DMAC_CTL_H_LLI_LAST | DW_DMAC_CTL_H_LLI_VALID;

    rx_cfg_h = DW_DMAC_CFG_H_TT_FC_PER2MEM |
               DW_DMAC_CFG_H_HS_SEL_SRC_HW |
               DW_DMAC_CFG_H_SRC_PER(SPI0_DMA_RX_CHAN) |
               DW_DMAC_CFG_H_PRIO(1U);

    mmio_write_32(rx_ch_base + DW_DMAC_CH_SAR_L_OFF, (uint32_t)(spi_dr_addr & 0xFFFFFFFFUL));
    mmio_write_32(rx_ch_base + DW_DMAC_CH_SAR_H_OFF, (uint32_t)((uint64_t)spi_dr_addr >> 32));
    mmio_write_32(rx_ch_base + DW_DMAC_CH_DAR_L_OFF, (uint32_t)(rx_dst_addr & 0xFFFFFFFFUL));
    mmio_write_32(rx_ch_base + DW_DMAC_CH_DAR_H_OFF, (uint32_t)((uint64_t)rx_dst_addr >> 32));
    mmio_write_32(rx_ch_base + DW_DMAC_CH_BLOCK_TS_OFF, (uint32_t)(block_frames - 1U));
    mmio_write_32(rx_ch_base + DW_DMAC_CH_CTL_L_OFF, rx_ctl_l);
    mmio_write_32(rx_ch_base + DW_DMAC_CH_CTL_H_OFF, ctl_h);
    mmio_write_32(rx_ch_base + DW_DMAC_CH_CFG_L_OFF, DW_DMAC_CFG_L_SINGLE_BLOCK);
    mmio_write_32(rx_ch_base + DW_DMAC_CH_CFG_H_OFF, rx_cfg_h);
    mmio_write_32(rx_ch_base + DW_DMAC_CH_LLP_L_OFF, 0U);
    mmio_write_32(rx_ch_base + DW_DMAC_CH_LLP_H_OFF, 0U);
    mmio_write_32(rx_ch_base + DW_DMAC_CH_INTSTATUS_EN_L_OFF,
                  DW_DMAC_INT_BLOCK_DONE | DW_DMAC_INT_DMA_DONE | DW_DMAC_INT_ERR_MASK);

    /* 3. Thiết lập Kênh TX: Đọc từ bộ nhớ RAM và ghi vào thanh ghi SPI0_DR */
    tx_ctl_l = (tx_inc ? DW_DMAC_CTL_L_SINC_INC : DW_DMAC_CTL_L_SINC_NOCHANGE) |
               DW_DMAC_CTL_L_DINC_NOCHANGE |
               DW_DMAC_CTL_L_SRC_WIDTH(tr_width) |
               DW_DMAC_CTL_L_DST_WIDTH(tr_width) |
               DW_DMAC_CTL_L_SRC_MSIZE(msize) |
               DW_DMAC_CTL_L_DST_MSIZE(msize);

    tx_cfg_h = DW_DMAC_CFG_H_TT_FC_MEM2PER |
               DW_DMAC_CFG_H_HS_SEL_DST_HW |
               DW_DMAC_CFG_H_DST_PER(SPI0_DMA_TX_CHAN) |
               DW_DMAC_CFG_H_PRIO(0U);

    mmio_write_32(tx_ch_base + DW_DMAC_CH_SAR_L_OFF, (uint32_t)(tx_src_addr & 0xFFFFFFFFUL));
    mmio_write_32(tx_ch_base + DW_DMAC_CH_SAR_H_OFF, (uint32_t)((uint64_t)tx_src_addr >> 32));
    mmio_write_32(tx_ch_base + DW_DMAC_CH_DAR_L_OFF, (uint32_t)(spi_dr_addr & 0xFFFFFFFFUL));
    mmio_write_32(tx_ch_base + DW_DMAC_CH_DAR_H_OFF, (uint32_t)((uint64_t)spi_dr_addr >> 32));
    mmio_write_32(tx_ch_base + DW_DMAC_CH_BLOCK_TS_OFF, (uint32_t)(block_frames - 1U));
    mmio_write_32(tx_ch_base + DW_DMAC_CH_CTL_L_OFF, tx_ctl_l);
    mmio_write_32(tx_ch_base + DW_DMAC_CH_CTL_H_OFF, ctl_h);
    mmio_write_32(tx_ch_base + DW_DMAC_CH_CFG_L_OFF, DW_DMAC_CFG_L_SINGLE_BLOCK);
    mmio_write_32(tx_ch_base + DW_DMAC_CH_CFG_H_OFF, tx_cfg_h);
    mmio_write_32(tx_ch_base + DW_DMAC_CH_LLP_L_OFF, 0U);
    mmio_write_32(tx_ch_base + DW_DMAC_CH_LLP_H_OFF, 0U);
    mmio_write_32(tx_ch_base + DW_DMAC_CH_INTSTATUS_EN_L_OFF,
                  DW_DMAC_INT_BLOCK_DONE | DW_DMAC_INT_DMA_DONE | DW_DMAC_INT_ERR_MASK);

    /* 4. Bật Kênh RX trước, Kênh TX sau để luôn sẵn sàng đón khung dữ liệu đầu tiên */
    dmac_channel_enable(SPI0_DMA_RX_CHAN);
    dmac_channel_enable(SPI0_DMA_TX_CHAN);

    /* 5. Đợi cả 2 kênh hoàn tất, đồng thời giám sát lỗi FIFO của SPI và lỗi AXI của DMA */
    max_polls = ((timeout_ms == 0U) ? 1000U : timeout_ms) * 50000U;
    for (uint32_t poll = 0; poll < max_polls; ++poll) {
        uint32_t rx_int = mmio_read_32(rx_ch_base + DW_DMAC_CH_INTSTATUS_L_OFF);
        uint32_t tx_int = mmio_read_32(tx_ch_base + DW_DMAC_CH_INTSTATUS_L_OFF);
        uint32_t chen = mmio_read_32(DW_DMAC_BASE_ADDR + DW_DMAC_CHEN_L_OFF);
        uint32_t spi_risr = mmio_read_32(reg_base + DW_SPI_RISR);

        if ((spi_risr & DW_SPI_FIFO_ERR_MASK) != 0U ||
            ((rx_int | tx_int) & DW_DMAC_INT_ERR_MASK) != 0U) {
            hal_spi_platform_dma_abort(reg_base);
            return HAL_SPI_ERR_FIFO;
        }

        bool rx_done = ((rx_int & (DW_DMAC_INT_BLOCK_DONE | DW_DMAC_INT_DMA_DONE)) != 0U) ||
                       ((chen & (1U << SPI0_DMA_RX_CHAN)) == 0U);
        bool tx_done = ((tx_int & (DW_DMAC_INT_BLOCK_DONE | DW_DMAC_INT_DMA_DONE)) != 0U) ||
                       ((chen & (1U << SPI0_DMA_TX_CHAN)) == 0U);

        if (rx_done && tx_done) {
            mmio_write_32(rx_ch_base + DW_DMAC_CH_INTCLEAR_L_OFF, DW_DMAC_INT_ALL_MASK);
            mmio_write_32(tx_ch_base + DW_DMAC_CH_INTCLEAR_L_OFF, DW_DMAC_INT_ALL_MASK);
            return HAL_SPI_OK;
        }
    }

    hal_spi_platform_dma_abort(reg_base);
    return HAL_SPI_ERR_TIMEOUT;
}

hal_spi_status_t hal_spi_platform_dma_transfer(uintptr_t reg_base,
                                               const void *tx_buf,
                                               void *rx_buf,
                                               size_t frames,
                                               uint8_t frame_bytes,
                                               uint16_t fill_word,
                                               uint32_t timeout_ms)
{
    size_t total_bytes;
    size_t done_frames = 0;

    if (reg_base != SPI0_BASE_ADDR || frames == 0U ||
        (frame_bytes != 1U && frame_bytes != 2U) ||
        frames > (SIZE_MAX / (size_t)frame_bytes)) {
        return HAL_SPI_ERR_INVALID;
    }

    if (frame_bytes == 2U &&
        ((((uintptr_t)tx_buf & 1UL) != 0UL) || (((uintptr_t)rx_buf & 1UL) != 0UL))) {
        return HAL_SPI_ERR_INVALID;
    }

    if (!g_spi0_plat.dma_initialized) {
        return HAL_SPI_ERR_UNSUPPORTED;
    }

    total_bytes = frames * (size_t)frame_bytes;

    /* Bước 1: Đồng bộ D-Cache cho vùng nhớ phát (TX) và vùng nhớ nhận (RX) */
    if (tx_buf != NULL) {
        platform_dcache_clean(tx_buf, total_bytes);
    } else {
        if (frame_bytes == 1U) {
            g_dma_tx_dummy_line[0] = (uint8_t)(fill_word & 0xFFU);
        } else {
            *((uint16_t *)(void *)g_dma_tx_dummy_line) = fill_word;
        }
        platform_dcache_clean(g_dma_tx_dummy_line, C906_CACHE_LINE_BYTES);
    }

    if (rx_buf != NULL) {
        platform_dcache_prepare_rx(rx_buf, total_bytes);
    } else {
        platform_dcache_prepare_rx(g_dma_rx_dummy_line, C906_CACHE_LINE_BYTES);
    }

    /* Bước 2: Chia gói dữ liệu lớn thành các block <= 4096 khung và thực thi tuần tự */
    while (done_frames < frames) {
        size_t chunk = frames - done_frames;
        uintptr_t tx_addr;
        uintptr_t rx_addr;
        hal_spi_status_t status;

        if (chunk > HAL_SPI_DMA_MAX_BLOCK_FRAMES) {
            chunk = HAL_SPI_DMA_MAX_BLOCK_FRAMES;
        }

        tx_addr = (tx_buf != NULL)
                      ? (((uintptr_t)tx_buf) + (done_frames * (size_t)frame_bytes))
                      : (uintptr_t)g_dma_tx_dummy_line;
        rx_addr = (rx_buf != NULL)
                      ? (((uintptr_t)rx_buf) + (done_frames * (size_t)frame_bytes))
                      : (uintptr_t)g_dma_rx_dummy_line;

        status = dmac_run_single_block(reg_base,
                                       tx_addr,
                                       (tx_buf != NULL),
                                       rx_addr,
                                       (rx_buf != NULL),
                                       chunk,
                                       frame_bytes,
                                       timeout_ms);
        if (status != HAL_SPI_OK) {
            if (rx_buf != NULL) {
                platform_dcache_invalidate_rx(rx_buf, total_bytes);
            }
            return status;
        }

        done_frames += chunk;
    }

    /* Bước 3: Vô hiệu hóa D-Cache vùng nhận để CPU đọc dữ liệu mới nhất từ RAM */
    if (rx_buf != NULL) {
        platform_dcache_invalidate_rx(rx_buf, total_bytes);
    }

    return HAL_SPI_OK;
}
