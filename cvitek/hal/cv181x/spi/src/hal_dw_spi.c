/*
 * Tầng 1 (T1 - Controller HAL): Điều khiển khối IP Synopsys DesignWare APB SSI (DW_apb_ssi).
 * Tác dụng: Quản lý thanh ghi phần cứng, hàng đợi FIFO và 3 engine truyền nhận (POLL, IRQ, DMA).
 */

#include "hal_dw_spi.h"

#include "dw_spi_regs.h"
#include "hal_spi_platform_limits.h"
#include "mmio.h"

/* Số chu kỳ đọc thanh ghi SR tối đa khi chờ bus SPI về trạng thái nghỉ */
#define HAL_SPI_FINAL_QUIESCE_SPINS     200000U

/* Kích thước bộ đệm trung gian trên stack khi truyền DMA kết hợp đảo bit LSB-first */
#define HAL_SPI_LSB_DMA_CHUNK_FRAMES    64U

/* --------------------------------------------------------------------------
 * 1. Các hàm nội bộ: truy cập thanh ghi & xử lý khung dữ liệu
 * -------------------------------------------------------------------------- */

/* Tác dụng: Đọc giá trị 32-bit từ thanh ghi của khối IP SPI */
static inline uint32_t reg_read(const hal_spi_t *hal, uint32_t offset)
{
    return mmio_read_32(hal->reg_base + offset);
}

/* Tác dụng: Ghi giá trị 32-bit vào thanh ghi của khối IP SPI */
static inline void reg_write(const hal_spi_t *hal, uint32_t offset, uint32_t value)
{
    mmio_write_32(hal->reg_base + offset, value);
}

/* Tác dụng: Tạo mặt nạ bit theo độ rộng khung (4-16 bit) để lọc bỏ bit thừa */
static uint16_t frame_mask(uint8_t bits_per_word)
{
    if (bits_per_word >= 16U) {
        return 0xFFFFU;
    }
    return (uint16_t)((1U << bits_per_word) - 1U);
}

/* Tác dụng: Đảo ngược thứ tự bit trong khung bằng thuật toán O(1) phục vụ chế độ LSB-first */
static uint16_t reverse_frame_bits(uint16_t word, uint8_t width)
{
    uint16_t v = word & frame_mask(width);

    v = (uint16_t)(((v & 0xAAAAU) >> 1) | ((v & 0x5555U) << 1));
    v = (uint16_t)(((v & 0xCCCCU) >> 2) | ((v & 0x3333U) << 2));
    v = (uint16_t)(((v & 0xF0F0U) >> 4) | ((v & 0x0F0FU) << 4));
    v = (uint16_t)((v >> 8) | (v << 8));
    return (uint16_t)(v >> (16U - width));
}

/* Tác dụng: Tính hệ số chia chẵn (SCKDV >= 2) cho thanh ghi BAUDR sao cho SCK <= yêu cầu */
static hal_spi_status_t compute_even_divider(uint32_t ref_clock_hz,
                                             uint32_t requested_sck_hz,
                                             uint32_t *divider_out,
                                             uint32_t *actual_sck_out)
{
    uint64_t div;

    if (ref_clock_hz == 0U || requested_sck_hz == 0U || requested_sck_hz > ref_clock_hz ||
        requested_sck_hz > HAL_SPI_MAX_SCK_HZ) {
        return HAL_SPI_ERR_INVALID;
    }

    div = ((uint64_t)ref_clock_hz + (uint64_t)requested_sck_hz - 1ULL) /
          (uint64_t)requested_sck_hz;
    if (div < HAL_SPI_MIN_DIVIDER) {
        div = HAL_SPI_MIN_DIVIDER;
    }
    if ((div & 1ULL) != 0ULL) {
        div += 1ULL;
    }
    if (div > HAL_SPI_MAX_DIVIDER) {
        return HAL_SPI_ERR_INVALID;
    }

    *divider_out = (uint32_t)div;
    *actual_sck_out = ref_clock_hz / (uint32_t)div;
    return HAL_SPI_OK;
}

/* Tác dụng: Lấy khung dữ liệu tiếp theo, áp dụng mask, fill_word và LSB-first để nạp vào TX FIFO */
static uint16_t next_tx_word(const hal_spi_t *hal)
{
    uint16_t word;

    if (hal->tx == NULL) {
        word = hal->fill_word;
    } else if (hal->frame_bytes == 1U) {
        const uint8_t *src = (const uint8_t *)hal->tx;
        word = src[hal->tx_done];
    } else {
        const uint16_t *src = (const uint16_t *)hal->tx;
        word = src[hal->tx_done];
    }

    word &= frame_mask(hal->bits_per_word);
    if (hal->lsb_first) {
        word = reverse_frame_bits(word, hal->bits_per_word);
    }
    return word;
}

/* Tác dụng: Lưu khung dữ liệu đọc từ RX FIFO vào buffer người dùng (sau khi đảo bit nếu LSB-first) */
static void store_rx_word(hal_spi_t *hal, uint16_t word)
{
    uint16_t masked = word & frame_mask(hal->bits_per_word);

    if (hal->rx == NULL) {
        return;
    }
    if (hal->lsb_first) {
        masked = reverse_frame_bits(masked, hal->bits_per_word);
    }
    if (hal->frame_bytes == 1U) {
        uint8_t *dst = (uint8_t *)hal->rx;
        dst[hal->rx_done] = (uint8_t)masked;
    } else {
        uint16_t *dst = (uint16_t *)hal->rx;
        dst[hal->rx_done] = masked;
    }
}

/* Tác dụng: Rút cạn toàn bộ các khung đang có sẵn trong RX FIFO phần cứng để tránh tràn bộ đệm */
static void drain_rx_fifo(hal_spi_t *hal)
{
    while (hal->rx_done < hal->frames &&
           (reg_read(hal, DW_SPI_SR) & DW_SPI_SR_RFNE) != 0U) {
        uint16_t word = (uint16_t)reg_read(hal, DW_SPI_DR);
        store_rx_word(hal, word);
        hal->rx_done += 1U;
    }
}

/* Tác dụng: Bơm khung vào TX FIFO, duy trì (tx_done - rx_done) < FIFO_DEPTH để ngăn tràn RX FIFO */
static void fill_tx_fifo(hal_spi_t *hal)
{
    while (hal->tx_done < hal->frames &&
           (hal->tx_done - hal->rx_done) < HAL_SPI_FIFO_DEPTH &&
           (reg_read(hal, DW_SPI_SR) & DW_SPI_SR_TFNF) != 0U) {
        reg_write(hal, DW_SPI_DR, next_tx_word(hal));
        hal->tx_done += 1U;
    }
}

/* Tác dụng: Chờ cả TX FIFO rỗng và bus hết bận (BUSY == 0) để đảm bảo đường truyền đã hoàn tất */
static hal_spi_status_t wait_for_bus_quiescent(const hal_spi_t *hal)
{
    for (uint32_t spin = 0; spin < HAL_SPI_FINAL_QUIESCE_SPINS; ++spin) {
        uint32_t sr = reg_read(hal, DW_SPI_SR);
        if ((sr & DW_SPI_SR_TFE) != 0U && (sr & DW_SPI_SR_BUSY) == 0U) {
            return HAL_SPI_OK;
        }
    }
    return HAL_SPI_ERR_TIMEOUT;
}

/* Tác dụng: Tắt IP (SSIENR=0), ngắt mặt nạ, tắt DMA và xóa cờ ngắt để cô lập an toàn bộ điều khiển */
static void quiesce_controller(const hal_spi_t *hal)
{
    reg_write(hal, DW_SPI_SSIENR, 0U);
    reg_write(hal, DW_SPI_IMR, 0U);
    reg_write(hal, DW_SPI_DMACR, 0U);
    reg_write(hal, DW_SPI_SER, 0U);
    (void)reg_read(hal, DW_SPI_ICR);
}

/* --------------------------------------------------------------------------
 * 2. Khởi tạo, kiểm tra và nạp cấu hình phần cứng
 * -------------------------------------------------------------------------- */

/* Tác dụng: Khởi tạo T1 SPI HAL, cấp clock, soft-reset và cấu hình an toàn cho chân CS/Pinmux qua T0 */
hal_spi_status_t hal_spi_init(hal_spi_t *hal, uintptr_t reg_base, uint32_t ref_clock_hz)
{
    uint32_t resolved_clock = hal_spi_platform_get_ref_clock_hz();
    hal_spi_status_t status;

    if (hal == NULL || reg_base == 0U) {
        return HAL_SPI_ERR_INVALID;
    }

    if (resolved_clock == 0U || (ref_clock_hz != 0U && ref_clock_hz != resolved_clock)) {
        return HAL_SPI_ERR_INVALID;
    }

    hal->reg_base = reg_base;
    hal->ref_clock_hz = resolved_clock;
    hal->actual_sck_hz = 0U;
    hal->tx = NULL;
    hal->rx = NULL;
    hal->frames = 0U;
    hal->tx_done = 0U;
    hal->rx_done = 0U;
    hal->fill_word = 0xFFFFU;
    hal->bits_per_word = 8U;
    hal->frame_bytes = 1U;
    hal->mode = 0U;
    hal->rx_sample_dly = 0U;
    hal->lsb_first = false;
    hal->configured = false;
    hal->active = false;
    hal->irq_ready = false;
    hal->dma_ready = false;
    hal->irq_status = (int32_t)HAL_SPI_OK;

    status = hal_spi_platform_clock_enable(reg_base);
    if (status != HAL_SPI_OK) return status;
    status = hal_spi_platform_reset(reg_base);
    if (status != HAL_SPI_OK) return status;

    /* Nạp CS mức HIGH trước khi chuyển mux để ngăn xung chọn sai (CS glitch) lúc khởi động */
    hal_spi_platform_cs_init();
    status = hal_spi_platform_pinmux_init();
    if (status != HAL_SPI_OK) return status;
    hal_spi_platform_cs_set(false);

    quiesce_controller(hal);
    return HAL_SPI_OK;
}

/* Tác dụng: Trampoline chuyển tiếp ngắt từ T0 sang hàm phục vụ ngắt T1 (hal_spi_isr) */
static void platform_irq_handler(void *ctx)
{
    hal_spi_isr((hal_spi_t *)ctx);
}

/* Tác dụng: Khởi tạo trễ tài nguyên ngắt (PLIC & Semaphore) ở T0 khi phân đoạn có nhu cầu dùng IRQ */
hal_spi_status_t hal_spi_prepare_irq(hal_spi_t *hal)
{
    hal_spi_status_t status;
    if (hal == NULL || hal->reg_base == 0U)
        return HAL_SPI_ERR_INVALID;
    if (hal->active)
        return HAL_SPI_ERR_BUSY;
    if (hal->irq_ready)
        return HAL_SPI_OK;
    status = hal_spi_platform_irq_init(hal->reg_base, platform_irq_handler, hal);
    if (status == HAL_SPI_OK)
        hal->irq_ready = true;
    return status;
}

/* Tác dụng: Xác thực và chuẩn bị các kênh System DMA ở T0 khi phân đoạn có nhu cầu dùng DMA */
hal_spi_status_t hal_spi_prepare_dma(hal_spi_t *hal, bool resources_reserved)
{
    hal_spi_status_t status;
    if (hal == NULL || hal->reg_base == 0U)
        return HAL_SPI_ERR_INVALID;
    if (!resources_reserved)
        return HAL_SPI_ERR_UNSUPPORTED;
    if (hal->active)
        return HAL_SPI_ERR_BUSY;
    if (hal->dma_ready)
        return HAL_SPI_OK;
    status = hal_spi_platform_dma_init(hal->reg_base, resources_reserved);
    if (status == HAL_SPI_OK)
        hal->dma_ready = true;
    return status;
}

/* Tác dụng: Kiểm tra tính hợp lệ của cấu hình SPI (mode, bits, divider) mà không ghi thanh ghi */
hal_spi_status_t hal_spi_validate_config(const hal_spi_t *hal, const hal_spi_config_t *cfg)
{
    uint32_t divider = 0U;
    uint32_t actual_sck = 0U;

    if (hal == NULL || cfg == NULL || hal->reg_base == 0U || hal->ref_clock_hz == 0U) {
        return HAL_SPI_ERR_INVALID;
    }
    if (cfg->mode > 3U) {
        return HAL_SPI_ERR_INVALID;
    }
    if (cfg->bits_per_word < 4U || cfg->bits_per_word > 16U) {
        return HAL_SPI_ERR_INVALID;
    }
    if (cfg->rx_sample_dly > HAL_SPI_MAX_RX_SAMPLE_DLY) {
        return HAL_SPI_ERR_INVALID;
    }

    return compute_even_divider(hal->ref_clock_hz, cfg->sck_hz, &divider, &actual_sck);
}

/* Tác dụng: Nạp cấu hình (mode, bits_per_word, baudrate, thresholds) vào các thanh ghi của IP SPI */
hal_spi_status_t hal_spi_configure(hal_spi_t *hal, const hal_spi_config_t *cfg)
{
    uint32_t divider = 0U;
    uint32_t actual_sck = 0U;
    uint32_t ctrlr0 = 0U;
    hal_spi_status_t status;

    status = hal_spi_validate_config(hal, cfg);
    if (status != HAL_SPI_OK) {
        return status;
    }

    if (hal->active || (reg_read(hal, DW_SPI_SR) & DW_SPI_SR_BUSY) != 0U) {
        return HAL_SPI_ERR_BUSY;
    }

    (void)compute_even_divider(hal->ref_clock_hz, cfg->sck_hz, &divider, &actual_sck);

    ctrlr0 = ((uint32_t)(cfg->bits_per_word - 1U) & DW_SPI_CTRLR0_DFS_MASK) |
             DW_SPI_CTRLR0_FRF_SPI |
             DW_SPI_CTRLR0_TMOD_TR;
    if ((cfg->mode & 0x1U) != 0U) {
        ctrlr0 |= DW_SPI_CTRLR0_SCPH;
    }
    if ((cfg->mode & 0x2U) != 0U) {
        ctrlr0 |= DW_SPI_CTRLR0_SCPOL;
    }

    /* Bắt buộc tắt SSIENR = 0 trước khi nạp CTRLR0, BAUDR và RX_SAMPLE_DLY theo chuẩn DW SSI */
    reg_write(hal, DW_SPI_SSIENR, 0U);
    reg_write(hal, DW_SPI_IMR, 0U);
    reg_write(hal, DW_SPI_DMACR, 0U);
    reg_write(hal, DW_SPI_SER, 0U);
    reg_write(hal, DW_SPI_CTRLR0, ctrlr0);
    reg_write(hal, DW_SPI_BAUDR, divider);
    reg_write(hal, DW_SPI_TXFTLR, HAL_SPI_DEFAULT_TX_THRESHOLD);
    reg_write(hal, DW_SPI_RXFTLR, 0U);
    reg_write(hal, DW_SPI_RX_SAMPLE_DLY,
              (uint32_t)cfg->rx_sample_dly & DW_SPI_RX_SAMPLE_DLY_MASK);
    (void)reg_read(hal, DW_SPI_ICR);
    reg_write(hal, DW_SPI_SSIENR, 1U);

    hal->actual_sck_hz = actual_sck;
    hal->bits_per_word = cfg->bits_per_word;
    hal->frame_bytes = (cfg->bits_per_word <= 8U) ? 1U : 2U;
    hal->mode = cfg->mode;
    hal->rx_sample_dly = cfg->rx_sample_dly;
    hal->lsb_first = cfg->lsb_first;
    hal->configured = true;
    return HAL_SPI_OK;
}

/* Tác dụng: Xóa sạch dữ liệu cũ trong FIFO, bật bộ dịch bit (SER) và chuẩn bị bộ đệm cho phân đoạn mới */
hal_spi_status_t hal_spi_begin(hal_spi_t *hal,
                               const void *tx,
                               void *rx,
                               size_t frames,
                               uint16_t fill_word)
{
    if (hal == NULL || !hal->configured || frames == 0U) {
        return HAL_SPI_ERR_INVALID;
    }
    if (hal->active || (reg_read(hal, DW_SPI_SR) & DW_SPI_SR_BUSY) != 0U) {
        return HAL_SPI_ERR_BUSY;
    }

    reg_write(hal, DW_SPI_IMR, 0U);
    reg_write(hal, DW_SPI_DMACR, 0U);

    /* Xóa sạch dữ liệu còn tồn trong RX FIFO và xóa cờ lỗi ngắt cũ */
    while ((reg_read(hal, DW_SPI_SR) & DW_SPI_SR_RFNE) != 0U) {
        (void)reg_read(hal, DW_SPI_DR);
    }
    (void)reg_read(hal, DW_SPI_ICR);

    /* Kích hoạt máy trạng thái dịch dữ liệu của IP SPI */
    reg_write(hal, DW_SPI_SER, 1U);

    hal->tx = tx;
    hal->rx = rx;
    hal->frames = frames;
    hal->tx_done = 0U;
    hal->rx_done = 0U;
    hal->fill_word = fill_word & frame_mask(hal->bits_per_word);
    hal->irq_status = (int32_t)HAL_SPI_PENDING;
    hal->active = true;
    return HAL_SPI_OK;
}

/* --------------------------------------------------------------------------
 * 3. Cơ chế thực thi 1: CPU Polling (hal_spi_poll)
 * -------------------------------------------------------------------------- */

/* Tác dụng: Thực hiện một bước truyền nhận dữ liệu Polling không khóa cứng (non-blocking step) */
hal_spi_status_t hal_spi_poll(hal_spi_t *hal)
{
    uint32_t sr;

    if (hal == NULL || !hal->configured || !hal->active) {
        return HAL_SPI_ERR_INVALID;
    }

    if ((reg_read(hal, DW_SPI_RISR) & DW_SPI_FIFO_ERR_MASK) != 0U) {
        return HAL_SPI_ERR_FIFO;
    }

    drain_rx_fifo(hal);
    if ((reg_read(hal, DW_SPI_RISR) & DW_SPI_FIFO_ERR_MASK) != 0U) {
        return HAL_SPI_ERR_FIFO;
    }

    fill_tx_fifo(hal);

    drain_rx_fifo(hal);
    if ((reg_read(hal, DW_SPI_RISR) & DW_SPI_FIFO_ERR_MASK) != 0U) {
        return HAL_SPI_ERR_FIFO;
    }

    if (hal->rx_done < hal->frames) {
        return HAL_SPI_PENDING;
    }

    sr = reg_read(hal, DW_SPI_SR);
    if ((sr & DW_SPI_SR_TFE) == 0U || (sr & DW_SPI_SR_BUSY) != 0U) {
        return HAL_SPI_PENDING;
    }

    return HAL_SPI_OK;
}

/* --------------------------------------------------------------------------
 * 4. Cơ chế thực thi 2: Ngắt PLIC (hal_spi_isr & hal_spi_transfer_irq)
 * -------------------------------------------------------------------------- */

/* Tác dụng: Cập nhật ngưỡng RXFTLR theo số khung còn lại để đảm bảo khung cuối cùng kích hoạt ngắt */
static void update_irq_thresholds_and_mask(const hal_spi_t *hal)
{
    size_t rem_rx = hal->frames - hal->rx_done;
    uint32_t rx_tl = 0U;
    uint32_t imr = DW_SPI_INT_RXFI | DW_SPI_FIFO_ERR_MASK;

    if (rem_rx > HAL_SPI_DEFAULT_TX_THRESHOLD) {
        rx_tl = HAL_SPI_DEFAULT_TX_THRESHOLD - 1U;
    } else if (rem_rx > 1U) {
        rx_tl = (uint32_t)(rem_rx - 1U);
    } else {
        rx_tl = 0U;
    }

    reg_write(hal, DW_SPI_RXFTLR, rx_tl);
    reg_write(hal, DW_SPI_TXFTLR, HAL_SPI_DEFAULT_TX_THRESHOLD);

    if (hal->tx_done < hal->frames) {
        imr |= DW_SPI_INT_TXEI;
    }
    reg_write(hal, DW_SPI_IMR, imr);
}

/* Tác dụng: Trình phục vụ ngắt SPI: xử lý lỗi FIFO, nạp/rút dữ liệu và notify đánh thức task */
void hal_spi_isr(hal_spi_t *hal)
{
    uint32_t risr;

    if (hal == NULL || !hal->active) {
        if (hal != NULL && hal->reg_base != 0U) {
            reg_write(hal, DW_SPI_IMR, 0U);
            (void)reg_read(hal, DW_SPI_ICR);
        }
        return;
    }

    risr = reg_read(hal, DW_SPI_RISR);
    drain_rx_fifo(hal);
    risr |= reg_read(hal, DW_SPI_RISR);

    if ((risr & DW_SPI_FIFO_ERR_MASK) != 0U) {
        reg_write(hal, DW_SPI_IMR, 0U);
        (void)reg_read(hal, DW_SPI_ICR);
        hal->irq_status = (int32_t)HAL_SPI_ERR_FIFO;
        hal_spi_platform_irq_notify(hal->reg_base);
        return;
    }

    fill_tx_fifo(hal);
    drain_rx_fifo(hal);

    if (hal->rx_done >= hal->frames) {
        reg_write(hal, DW_SPI_IMR, 0U);
        hal->irq_status = (int32_t)HAL_SPI_OK;
        hal_spi_platform_irq_notify(hal->reg_base);
        return;
    }

    update_irq_thresholds_and_mask(hal);
}

/* Tác dụng: Thực thi phân đoạn dữ liệu qua cơ chế ngắt PLIC kết hợp ngủ chờ bằng semaphore */
hal_spi_status_t hal_spi_transfer_irq(hal_spi_t *hal, uint32_t timeout_ms)
{
    hal_spi_status_t wait_rc;

    if (hal == NULL || !hal->configured || !hal->active) {
        return HAL_SPI_ERR_INVALID;
    }
    if (!hal->irq_ready)
        return HAL_SPI_ERR_UNSUPPORTED;

    if ((reg_read(hal, DW_SPI_RISR) & DW_SPI_FIFO_ERR_MASK) != 0U) {
        return HAL_SPI_ERR_FIFO;
    }

    hal->irq_status = (int32_t)HAL_SPI_PENDING;

    /* Nạp đợt khung đầu tiên vào TX FIFO để kích phát xung SCK trước khi bật ngắt */
    fill_tx_fifo(hal);
    drain_rx_fifo(hal);
    if ((reg_read(hal, DW_SPI_RISR) & DW_SPI_FIFO_ERR_MASK) != 0U) {
        return HAL_SPI_ERR_FIFO;
    }

    if (hal->rx_done < hal->frames) {
        update_irq_thresholds_and_mask(hal);
        wait_rc = hal_spi_platform_irq_wait(hal->reg_base, timeout_ms);
        reg_write(hal, DW_SPI_IMR, 0U);

        if (hal->irq_status == (int32_t)HAL_SPI_ERR_FIFO ||
            (reg_read(hal, DW_SPI_RISR) & DW_SPI_FIFO_ERR_MASK) != 0U) {
            return HAL_SPI_ERR_FIFO;
        }
        if (wait_rc != HAL_SPI_OK && hal->rx_done < hal->frames) {
            return wait_rc;
        }
        if (hal->rx_done < hal->frames) {
            return HAL_SPI_ERR_TIMEOUT;
        }
    }

    return wait_for_bus_quiescent(hal);
}

/* --------------------------------------------------------------------------
 * 5. Cơ chế thực thi 3: System DMA (hal_spi_transfer_dma)
 * -------------------------------------------------------------------------- */

/* Tác dụng: Cấu hình ngưỡng bắt tay DMA ngoại vi của IP SPI và chuyển giao truyền nhận cho T0 */
static hal_spi_status_t execute_raw_dma_pass(hal_spi_t *hal,
                                             const void *tx_ptr,
                                             void *rx_ptr,
                                             size_t count,
                                             uint16_t raw_fill_word,
                                             uint32_t timeout_ms)
{
    hal_spi_status_t status;

    reg_write(hal, DW_SPI_DMATDLR, HAL_SPI_DEFAULT_TX_THRESHOLD);
    reg_write(hal, DW_SPI_DMARDLR, 0U);
    reg_write(hal, DW_SPI_DMACR, DW_SPI_DMACR_RDMAE | DW_SPI_DMACR_TDMAE);

    status = hal_spi_platform_dma_transfer(hal->reg_base,
                                           tx_ptr,
                                           rx_ptr,
                                           count,
                                           hal->frame_bytes,
                                           raw_fill_word,
                                           timeout_ms);

    reg_write(hal, DW_SPI_DMACR, 0U);
    if (status != HAL_SPI_OK) {
        return status;
    }

    if ((reg_read(hal, DW_SPI_RISR) & DW_SPI_FIFO_ERR_MASK) != 0U) {
        return HAL_SPI_ERR_FIFO;
    }

    return wait_for_bus_quiescent(hal);
}

/* Tác dụng: Thực thi phân đoạn dữ liệu qua DMA, hỗ trợ bounce buffer khi truyền LSB-first */
hal_spi_status_t hal_spi_transfer_dma(hal_spi_t *hal, uint32_t timeout_ms)
{
    uint16_t raw_fill;
    hal_spi_status_t status;

    if (hal == NULL || !hal->configured || !hal->active) {
        return HAL_SPI_ERR_INVALID;
    }
    if (!hal->dma_ready)
        return HAL_SPI_ERR_UNSUPPORTED;

    if ((reg_read(hal, DW_SPI_RISR) & DW_SPI_FIFO_ERR_MASK) != 0U) {
        return HAL_SPI_ERR_FIFO;
    }

    raw_fill = hal->lsb_first ? reverse_frame_bits(hal->fill_word, hal->bits_per_word)
                              : hal->fill_word;

    /* Trường hợp MSB-first hoặc chỉ nhận: truyền trực tiếp qua DMA */
    if (!hal->lsb_first || hal->tx == NULL) {
        status = execute_raw_dma_pass(hal,
                                      hal->tx,
                                      hal->rx,
                                      hal->frames,
                                      raw_fill,
                                      timeout_ms);
        if (status != HAL_SPI_OK) {
            return status;
        }

        if (hal->lsb_first && hal->rx != NULL) {
            if (hal->frame_bytes == 1U) {
                uint8_t *dst8 = (uint8_t *)hal->rx;
                for (size_t i = 0; i < hal->frames; ++i) {
                    dst8[i] = (uint8_t)reverse_frame_bits(dst8[i], hal->bits_per_word);
                }
            } else {
                uint16_t *dst16 = (uint16_t *)hal->rx;
                for (size_t i = 0; i < hal->frames; ++i) {
                    dst16[i] = reverse_frame_bits(dst16[i], hal->bits_per_word);
                }
            }
        }

        hal->tx_done = hal->frames;
        hal->rx_done = hal->frames;
        return HAL_SPI_OK;
    }

    /* Đảo bit qua bounce buffer trên stack (128 byte) để không làm thay đổi buffer hằng (tx) */
    {
        uint16_t tx_bounce[HAL_SPI_LSB_DMA_CHUNK_FRAMES];
        size_t offset = 0U;

        while (offset < hal->frames) {
            size_t chunk = hal->frames - offset;
            void *rx_chunk_ptr = NULL;

            if (chunk > HAL_SPI_LSB_DMA_CHUNK_FRAMES) {
                chunk = HAL_SPI_LSB_DMA_CHUNK_FRAMES;
            }

            if (hal->frame_bytes == 1U) {
                const uint8_t *src8 = (const uint8_t *)hal->tx;
                uint8_t *bounce8 = (uint8_t *)(void *)tx_bounce;
                for (size_t i = 0; i < chunk; ++i) {
                    bounce8[i] = (uint8_t)reverse_frame_bits(src8[offset + i],
                                                             hal->bits_per_word);
                }
            } else {
                const uint16_t *src16 = (const uint16_t *)hal->tx;
                for (size_t i = 0; i < chunk; ++i) {
                    tx_bounce[i] = reverse_frame_bits(src16[offset + i],
                                                      hal->bits_per_word);
                }
            }

            if (hal->rx != NULL) {
                rx_chunk_ptr = (uint8_t *)hal->rx + (offset * (size_t)hal->frame_bytes);
            }

            status = execute_raw_dma_pass(hal,
                                          tx_bounce,
                                          rx_chunk_ptr,
                                          chunk,
                                          raw_fill,
                                          timeout_ms);
            if (status != HAL_SPI_OK) {
                return status;
            }

            if (rx_chunk_ptr != NULL) {
                if (hal->frame_bytes == 1U) {
                    uint8_t *dst8 = (uint8_t *)rx_chunk_ptr;
                    for (size_t i = 0; i < chunk; ++i) {
                        dst8[i] = (uint8_t)reverse_frame_bits(dst8[i], hal->bits_per_word);
                    }
                } else {
                    uint16_t *dst16 = (uint16_t *)rx_chunk_ptr;
                    for (size_t i = 0; i < chunk; ++i) {
                        dst16[i] = reverse_frame_bits(dst16[i], hal->bits_per_word);
                    }
                }
            }

            offset += chunk;
            hal->tx_done = offset;
            hal->rx_done = offset;
        }
    }

    return HAL_SPI_OK;
}

/* --------------------------------------------------------------------------
 * 6. Kết thúc phân đoạn, chụp ảnh chẩn đoán và thu hồi khẩn cấp (Abort)
 * -------------------------------------------------------------------------- */

/* Tác dụng: Đóng phân đoạn thành công, tắt dịch bit (SER) và giữ nguyên cấu hình/CS cho phân đoạn kế */
void hal_spi_finish(hal_spi_t *hal)
{
    if (hal == NULL || hal->reg_base == 0U) {
        return;
    }

    reg_write(hal, DW_SPI_IMR, 0U);
    reg_write(hal, DW_SPI_DMACR, 0U);
    reg_write(hal, DW_SPI_SER, 0U);
    hal->tx = NULL;
    hal->rx = NULL;
    hal->frames = 0U;
    hal->tx_done = 0U;
    hal->rx_done = 0U;
    hal->active = false;
}

/* Tác dụng: Đọc chụp nhanh trạng thái các thanh ghi chính của SPI phục vụ chẩn đoán lỗi */
void hal_spi_snapshot(const hal_spi_t *hal, hal_spi_snapshot_t *out)
{
    if (hal == NULL || out == NULL || hal->reg_base == 0U) {
        return;
    }

    out->ctrlr0 = reg_read(hal, DW_SPI_CTRLR0);
    out->baudr = reg_read(hal, DW_SPI_BAUDR);
    out->sr = reg_read(hal, DW_SPI_SR);
    out->risr = reg_read(hal, DW_SPI_RISR);
    out->txflr = reg_read(hal, DW_SPI_TXFLR);
    out->rxflr = reg_read(hal, DW_SPI_RXFLR);
    out->imr = reg_read(hal, DW_SPI_IMR);
    out->dmacr = reg_read(hal, DW_SPI_DMACR);
    out->rx_sample_dly = reg_read(hal, DW_SPI_RX_SAMPLE_DLY);
}

/* Tác dụng: Dừng khẩn cấp DMA/ngắt, reset phần cứng controller và tự động khôi phục cấu hình ban đầu */
void hal_spi_abort(hal_spi_t *hal)
{
    bool was_configured;
    hal_spi_config_t saved_cfg;

    if (hal == NULL || hal->reg_base == 0U) {
        return;
    }

    was_configured = hal->configured;
    saved_cfg.sck_hz = hal->actual_sck_hz;
    saved_cfg.mode = hal->mode;
    saved_cfg.bits_per_word = hal->bits_per_word;
    saved_cfg.rx_sample_dly = hal->rx_sample_dly;
    saved_cfg.lsb_first = hal->lsb_first;

    /* 1. Khóa ngắt và dừng ngay các kênh DMA */
    reg_write(hal, DW_SPI_IMR, 0U);
    reg_write(hal, DW_SPI_DMACR, 0U);
    if (hal->dma_ready)
        hal_spi_platform_dma_abort(hal->reg_base);

    /* 2. Cô lập IP, nhả CS lên mức HIGH và soft-reset phần cứng */
    quiesce_controller(hal);
    hal_spi_platform_cs_set(false);
    if (hal_spi_platform_reset(hal->reg_base) != HAL_SPI_OK) {
        hal->configured = false;
        hal->active = false;
        hal->irq_status = (int32_t)HAL_SPI_ERR_INVALID;
        return;
    }
    quiesce_controller(hal);

    hal->tx = NULL;
    hal->rx = NULL;
    hal->frames = 0U;
    hal->tx_done = 0U;
    hal->rx_done = 0U;
    hal->active = false;
    hal->irq_status = (int32_t)HAL_SPI_OK;

    /* 3. Khôi phục lại cấu hình gần nhất để bus sẵn sàng cho giao dịch tiếp theo */
    if (was_configured && saved_cfg.sck_hz > 0U) {
        hal->configured = false;
        (void)hal_spi_configure(hal, &saved_cfg);
    }
}
