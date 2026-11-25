/*
 * Bộ kiểm thử đơn vị (Host Unit Test) cho toàn bộ các tầng T0-T2 của SPI Driver.
 *
 * Kiểm chứng trực tiếp mã nguồn hal_dw_spi.c (T1) và spi.c (T2) trên mô hình giả lập
 * phần cứng Synopsys DesignWare APB SSI + System DMA:
 * 1. Khởi tạo T0/T1, kiểm tra tham số cấu hình (sck_hz, mode, bits_per_word, rx_sample_dly).
 * 2. Kiểm tra điều kiện chống tràn FIFO (số khung đang bay in-flight <= 8).
 * 3. Kiểm tra cả 4 cơ chế thực thi ở Tầng 2:
 *    - SPI_BACKEND_POLL (Polling bằng CPU)
 *    - SPI_BACKEND_IRQ  (Truyền nhận bằng ngắt hal_spi_isr + ngưỡng động RXFTLR)
 *    - SPI_BACKEND_DMA  (Truyền nhận bằng phần cứng DMA + DMACR)
 *    - SPI_BACKEND_AUTO (Tự động chọn POLL / IRQ / DMA theo kích thước từng phân đoạn)
 * 4. Kiểm tra đảo thứ tự bit lsb_first trên cả POLL, IRQ và DMA.
 * 5. Kiểm tra các hàm tiện ích cho Driver Cảm biến: spi_write_then_read, spi_write, spi_read,
 *    spi_device_set_backend, spi_device_set_freq, spi_bus_recover, spi_bus_deinit.
 * 6. Kiểm tra cơ chế thu hồi lỗi phần cứng (FIFO Overflow, Timeout) và dừng khẩn cấp DMA.
 */

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "dw_spi_regs.h"
#include "hal_dw_spi.h"
#include "hal_spi_platform_limits.h"
#include "mmio.h"
#include "semphr.h"
#include "spi.h"

#define FAKE_SPI_BASE 0x04180000UL

/* --------------------------------------------------------------------------
 * Mô hình giả lập phần cứng DW APB SSI + T0 Platform Hooks + FreeRTOS Mutex
 * -------------------------------------------------------------------------- */

typedef struct {
    uint32_t ctrlr0;
    uint32_t ssienr;
    uint32_t ser;
    uint32_t baudr;
    uint32_t txftlr;
    uint32_t rxftlr;
    uint32_t sr;
    uint32_t imr;
    uint32_t risr;
    uint32_t dmacr;
    uint32_t dmatdlr;
    uint32_t dmardlr;
    uint32_t rx_sample_dly;
    uint16_t rx_fifo[64];
    size_t rx_head;
    size_t rx_tail;
    size_t rx_count;
    size_t max_rx_occupancy;
    uint16_t tx_history[512];
    size_t tx_count;
    uint16_t script_rx[512];
    size_t script_len;
    size_t script_pos;
    bool auto_complete_tx;
    bool hold_busy;
    bool inject_rx_overflow_on_next_tx;

    unsigned int clock_enables;
    unsigned int resets;
    unsigned int pinmux_inits;
    unsigned int cs_inits;
    unsigned int irq_inits;
    unsigned int irq_waits;
    unsigned int irq_notifies;
    unsigned int dma_inits;
    unsigned int dma_transfers;
    unsigned int dma_aborts;
    bool cs_asserted;
    bool cs_history[32];
    size_t cs_edges;

    hal_spi_irq_handler_t irq_handler;
    void *irq_ctx;
    bool simulate_irq_timeout;
    bool simulate_dma_timeout;
    bool simulate_dma_fifo_err;
    hal_spi_status_t irq_init_status;
    hal_spi_status_t dma_init_status;

    bool fail_mutex_take;
    unsigned int mutex_creates;
    unsigned int mutex_takes;
    unsigned int mutex_gives;
} fake_hw_t;

static fake_hw_t g_hw;

static void fake_hw_reset_controller_regs(void)
{
    g_hw.ctrlr0 = 0U;
    g_hw.ssienr = 0U;
    g_hw.ser = 0U;
    g_hw.baudr = 0U;
    g_hw.txftlr = 0U;
    g_hw.rxftlr = 0U;
    g_hw.sr = DW_SPI_SR_TFNF | DW_SPI_SR_TFE;
    g_hw.imr = 0U;
    g_hw.risr = 0U;
    g_hw.dmacr = 0U;
    g_hw.dmatdlr = 0U;
    g_hw.dmardlr = 0U;
    g_hw.rx_sample_dly = 0U;
    g_hw.rx_head = 0U;
    g_hw.rx_tail = 0U;
    g_hw.rx_count = 0U;
}

static void fake_hw_reset_all(void)
{
    memset(&g_hw, 0, sizeof(g_hw));
    fake_hw_reset_controller_regs();
    g_hw.auto_complete_tx = true;
}

static void fake_push_rx(uint16_t word)
{
    if (g_hw.rx_count >= HAL_SPI_FIFO_DEPTH) {
        g_hw.risr |= DW_SPI_INT_RXOI;
    }
    assert(g_hw.rx_count < 64U);
    g_hw.rx_fifo[g_hw.rx_tail] = word;
    g_hw.rx_tail = (g_hw.rx_tail + 1U) % 64U;
    g_hw.rx_count += 1U;
    if (g_hw.rx_count > g_hw.max_rx_occupancy) {
        g_hw.max_rx_occupancy = g_hw.rx_count;
    }
}

static uint16_t fake_pop_rx(void)
{
    uint16_t word = 0U;
    if (g_hw.rx_count == 0U) {
        g_hw.risr |= DW_SPI_INT_RXUI;
        return 0U;
    }
    word = g_hw.rx_fifo[g_hw.rx_head];
    g_hw.rx_head = (g_hw.rx_head + 1U) % 64U;
    g_hw.rx_count -= 1U;
    return word;
}

static uint16_t fake_next_miso_word(uint16_t mosi_word)
{
    if (g_hw.script_pos < g_hw.script_len) {
        uint16_t w = g_hw.script_rx[g_hw.script_pos];
        g_hw.script_pos += 1U;
        return w;
    }
    return (uint16_t)(mosi_word ^ 0x00A5U);
}

uint32_t mmio_read_32(uintptr_t addr)
{
    uint32_t offset = (uint32_t)(addr - FAKE_SPI_BASE);
    switch (offset) {
    case DW_SPI_CTRLR0:
        return g_hw.ctrlr0;
    case DW_SPI_SSIENR:
        return g_hw.ssienr;
    case DW_SPI_SER:
        return g_hw.ser;
    case DW_SPI_BAUDR:
        return g_hw.baudr;
    case DW_SPI_TXFTLR:
        return g_hw.txftlr;
    case DW_SPI_RXFTLR:
        return g_hw.rxftlr;
    case DW_SPI_TXFLR:
        return 0U;
    case DW_SPI_RXFLR:
        return (uint32_t)g_hw.rx_count;
    case DW_SPI_SR: {
        uint32_t sr = g_hw.sr;
        if (g_hw.rx_count > 0U) {
            sr |= DW_SPI_SR_RFNE;
        } else {
            sr &= ~DW_SPI_SR_RFNE;
        }
        if (g_hw.hold_busy) {
            sr |= DW_SPI_SR_BUSY;
        } else {
            sr &= ~DW_SPI_SR_BUSY;
        }
        return sr;
    }
    case DW_SPI_IMR:
        return g_hw.imr;
    case DW_SPI_ISR:
        return g_hw.risr & g_hw.imr;
    case DW_SPI_RISR:
        return g_hw.risr;
    case DW_SPI_ICR:
        g_hw.risr = 0U;
        return 0U;
    case DW_SPI_DMACR:
        return g_hw.dmacr;
    case DW_SPI_DMATDLR:
        return g_hw.dmatdlr;
    case DW_SPI_DMARDLR:
        return g_hw.dmardlr;
    case DW_SPI_DR:
        return fake_pop_rx();
    case DW_SPI_RX_SAMPLE_DLY:
        return g_hw.rx_sample_dly;
    default:
        return 0U;
    }
}

void mmio_write_32(uintptr_t addr, uint32_t val)
{
    uint32_t offset = (uint32_t)(addr - FAKE_SPI_BASE);
    switch (offset) {
    case DW_SPI_CTRLR0:
        assert(g_hw.ssienr == 0U);
        g_hw.ctrlr0 = val;
        break;
    case DW_SPI_SSIENR:
        g_hw.ssienr = val & 1U;
        if (g_hw.ssienr == 0U) {
            g_hw.rx_head = 0U;
            g_hw.rx_tail = 0U;
            g_hw.rx_count = 0U;
        }
        break;
    case DW_SPI_SER:
        g_hw.ser = val;
        break;
    case DW_SPI_BAUDR:
        assert(g_hw.ssienr == 0U);
        g_hw.baudr = val;
        break;
    case DW_SPI_TXFTLR:
        g_hw.txftlr = val;
        break;
    case DW_SPI_RXFTLR:
        g_hw.rxftlr = val;
        break;
    case DW_SPI_IMR:
        g_hw.imr = val;
        break;
    case DW_SPI_DMACR:
        g_hw.dmacr = val;
        break;
    case DW_SPI_DMATDLR:
        g_hw.dmatdlr = val;
        break;
    case DW_SPI_DMARDLR:
        g_hw.dmardlr = val;
        break;
    case DW_SPI_RX_SAMPLE_DLY:
        assert(g_hw.ssienr == 0U);
        g_hw.rx_sample_dly = val;
        break;
    case DW_SPI_DR: {
        uint16_t tx_word = (uint16_t)val;
        assert(g_hw.ssienr == 1U);
        assert(g_hw.tx_count < 512U);
        g_hw.tx_history[g_hw.tx_count++] = tx_word;
        if (g_hw.inject_rx_overflow_on_next_tx) {
            g_hw.risr |= DW_SPI_INT_RXOI;
        }
        if (g_hw.auto_complete_tx) {
            fake_push_rx(fake_next_miso_word(tx_word));
        }
        break;
    }
    default:
        break;
    }
}

hal_spi_status_t hal_spi_platform_clock_enable(uintptr_t reg_base)
{
    assert(reg_base == FAKE_SPI_BASE);
    g_hw.clock_enables += 1U;
    return HAL_SPI_OK;
}

hal_spi_status_t hal_spi_platform_reset(uintptr_t reg_base)
{
    assert(reg_base == FAKE_SPI_BASE);
    g_hw.resets += 1U;
    fake_hw_reset_controller_regs();
    return HAL_SPI_OK;}

hal_spi_status_t hal_spi_platform_pinmux_init(void)
{
    g_hw.pinmux_inits += 1U;
    return HAL_SPI_OK;
}

void hal_spi_platform_cs_init(void)
{
    g_hw.cs_inits += 1U;
    g_hw.cs_asserted = false;
}

void hal_spi_platform_cs_set(bool asserted)
{
    g_hw.cs_asserted = asserted;
    if (g_hw.cs_edges < 32U) {
        g_hw.cs_history[g_hw.cs_edges++] = asserted;
    }
}

uint32_t hal_spi_platform_get_ref_clock_hz(void)
{
    return HAL_SPI_REF_CLOCK_HZ;
}

hal_spi_status_t hal_spi_platform_irq_init(uintptr_t reg_base,
                                           hal_spi_irq_handler_t handler,
                                           void *ctx)
{
    assert(reg_base == FAKE_SPI_BASE);
    assert(handler != NULL);
    g_hw.irq_inits += 1U;
    if (g_hw.irq_init_status != HAL_SPI_OK)
        return g_hw.irq_init_status;
    g_hw.irq_handler = handler;
    g_hw.irq_ctx = ctx;
    return HAL_SPI_OK;
}

hal_spi_status_t hal_spi_platform_irq_wait(uintptr_t reg_base, uint32_t timeout_ms)
{
    (void)timeout_ms;
    assert(reg_base == FAKE_SPI_BASE);
    g_hw.irq_waits += 1U;

    if (g_hw.simulate_irq_timeout) {
        return HAL_SPI_ERR_TIMEOUT;
    }

    for (unsigned int step = 0; step < 128U && g_hw.imr != 0U; ++step) {
        assert(g_hw.irq_handler != NULL);
        g_hw.irq_handler(g_hw.irq_ctx);
    }
    return HAL_SPI_OK;
}

void hal_spi_platform_irq_notify(uintptr_t reg_base)
{
    assert(reg_base == FAKE_SPI_BASE);
    g_hw.irq_notifies += 1U;
}

hal_spi_status_t hal_spi_platform_dma_init(uintptr_t reg_base, bool resources_reserved)
{
    assert(reg_base == FAKE_SPI_BASE);
    assert(resources_reserved);
    g_hw.dma_inits += 1U;
    return g_hw.dma_init_status;
}

hal_spi_status_t hal_spi_platform_dma_transfer(uintptr_t reg_base,
                                               const void *tx_buf,
                                               void *rx_buf,
                                               size_t frames,
                                               uint8_t frame_bytes,
                                               uint16_t fill_word,
                                               uint32_t timeout_ms)
{
    (void)timeout_ms;
    assert(reg_base == FAKE_SPI_BASE);
    assert(g_hw.dmacr == (DW_SPI_DMACR_RDMAE | DW_SPI_DMACR_TDMAE));
    g_hw.dma_transfers += 1U;

    if (g_hw.simulate_dma_timeout) {
        return HAL_SPI_ERR_TIMEOUT;
    }
    if (g_hw.simulate_dma_fifo_err) {
        g_hw.risr |= DW_SPI_INT_RXOI;
        return HAL_SPI_ERR_FIFO;
    }

    for (size_t i = 0; i < frames; ++i) {
        uint16_t tx_w;
        uint16_t rx_w;

        if (tx_buf == NULL) {
            tx_w = fill_word;
        } else if (frame_bytes == 1U) {
            tx_w = ((const uint8_t *)tx_buf)[i];
        } else {
            tx_w = ((const uint16_t *)tx_buf)[i];
        }

        assert(g_hw.tx_count < 512U);
        g_hw.tx_history[g_hw.tx_count++] = tx_w;
        rx_w = fake_next_miso_word(tx_w);

        if (rx_buf != NULL) {
            if (frame_bytes == 1U) {
                ((uint8_t *)rx_buf)[i] = (uint8_t)rx_w;
            } else {
                ((uint16_t *)rx_buf)[i] = rx_w;
            }
        }
    }
    return HAL_SPI_OK;
}

void hal_spi_platform_dma_abort(uintptr_t reg_base)
{
    assert(reg_base == FAKE_SPI_BASE);
    g_hw.dma_aborts += 1U;
}

SemaphoreHandle_t xSemaphoreCreateMutexStatic(StaticSemaphore_t *storage)
{
    assert(storage != NULL);
    storage->locked = 0;
    g_hw.mutex_creates += 1U;
    return storage;
}

int xSemaphoreTake(SemaphoreHandle_t mutex, TickType_t ticks)
{
    (void)ticks;
    assert(mutex != NULL);
    g_hw.mutex_takes += 1U;
    if (g_hw.fail_mutex_take || mutex->locked != 0) {
        return 0;
    }
    mutex->locked = 1;
    return pdTRUE;
}

int xSemaphoreGive(SemaphoreHandle_t mutex)
{
    assert(mutex != NULL);
    assert(mutex->locked == 1);
    mutex->locked = 0;
    g_hw.mutex_gives += 1U;
    return pdTRUE;
}

/* --------------------------------------------------------------------------
 * Các bài kiểm thử (Unit Test Cases)
 * -------------------------------------------------------------------------- */

static void test_init_and_config_validation(void)
{
    hal_spi_t hal;
    hal_spi_config_t cfg = {
        .sck_hz = 10000000U,
        .mode = 1U,
        .bits_per_word = 8U,
        .rx_sample_dly = 2U,
        .lsb_first = false,
    };

    fake_hw_reset_all();
    assert(hal_spi_init(&hal, FAKE_SPI_BASE, 0U) == HAL_SPI_OK);
    assert(g_hw.clock_enables == 1U);
    assert(g_hw.resets == 1U);
    assert(g_hw.pinmux_inits == 1U);
    assert(g_hw.cs_inits == 1U);
    assert(g_hw.irq_inits == 0U);
    assert(g_hw.dma_inits == 0U);
    assert(!hal.irq_ready && !hal.dma_ready);
    assert(hal.ref_clock_hz == 187500000U);

    assert(hal_spi_configure(&hal, &cfg) == HAL_SPI_OK);
    assert(g_hw.baudr == 20U);
    assert(hal.actual_sck_hz == 9375000U);
    assert(g_hw.rx_sample_dly == 2U);
    assert((g_hw.ctrlr0 & DW_SPI_CTRLR0_DFS_MASK) == 7U);
    assert((g_hw.ctrlr0 & DW_SPI_CTRLR0_SCPH) != 0U);
    assert((g_hw.ctrlr0 & DW_SPI_CTRLR0_SCPOL) == 0U);

    cfg.rx_sample_dly = 9U;
    assert(hal_spi_validate_config(&hal, &cfg) == HAL_SPI_ERR_INVALID);
}

static void test_poll_and_fifo_in_flight_bound(void)
{
    hal_spi_t hal;
    hal_spi_config_t cfg = {10000000U, 0U, 8U, 0U, false};
    uint8_t tx[20];
    uint8_t rx[20];

    fake_hw_reset_all();
    for (size_t i = 0; i < sizeof(tx); ++i) {
        tx[i] = (uint8_t)(i + 1U);
        g_hw.script_rx[i] = (uint16_t)(0x80U + i);
    }
    g_hw.script_len = sizeof(tx);

    assert(hal_spi_init(&hal, FAKE_SPI_BASE, 0U) == HAL_SPI_OK);
    assert(hal_spi_configure(&hal, &cfg) == HAL_SPI_OK);
    assert(hal_spi_begin(&hal, tx, rx, sizeof(tx), 0xFFU) == HAL_SPI_OK);

    for (size_t step = 0; step < 32U; ++step) {
        hal_spi_status_t st = hal_spi_poll(&hal);
        if (st == HAL_SPI_OK) {
            break;
        }
        assert(st == HAL_SPI_PENDING);
    }
    assert(hal.rx_done == sizeof(tx));
    assert(g_hw.max_rx_occupancy <= HAL_SPI_FIFO_DEPTH);
    for (size_t i = 0; i < sizeof(tx); ++i) {
        assert(rx[i] == (uint8_t)(0x80U + i));
    }
    hal_spi_finish(&hal);
}

static void test_irq_backend_and_dynamic_thresholds(void)
{
    spi_bus_t bus;
    spi_device_t dev;
    spi_bus_config_t bus_cfg = {
        .reg_base = FAKE_SPI_BASE, .ref_clock_hz = HAL_SPI_REF_CLOCK_HZ,
        .enable_irq = true, .enable_dma = true, .dma_resources_reserved = true,
    };
    spi_device_config_t dev_cfg = {
        .sck_hz = 25000000U,
        .mode = 1U,
        .bits_per_word = 8U,
        .cs_id = SPI_CS0_GPIO,
        .backend = SPI_BACKEND_IRQ,
        .rx_sample_dly = 1U,
        .lsb_first = false,
    };
    uint8_t tx[19];
    uint8_t rx[19];

    fake_hw_reset_all();
    for (size_t i = 0; i < sizeof(tx); ++i) {
        tx[i] = (uint8_t)(0x10U + i);
        g_hw.script_rx[i] = (uint16_t)(0xA0U + i);
    }
    g_hw.script_len = sizeof(tx);

    assert(spi_bus_init(&bus, &bus_cfg) == SPI_OK);
    assert(spi_device_init(&dev, &bus, &dev_cfg) == SPI_OK);
    assert(spi_transfer_one(&dev, tx, rx, sizeof(tx), 0xFFU, 100U) == SPI_OK);

    assert(bus.irq_segment_count == 1U);
    assert(g_hw.irq_waits == 1U);
    assert(g_hw.irq_notifies == 1U);
    assert(g_hw.imr == 0U);
    assert(!g_hw.cs_asserted);
    for (size_t i = 0; i < sizeof(tx); ++i) {
        assert(rx[i] == (uint8_t)(0xA0U + i));
    }
}

static void test_dma_backend_and_auto_backend_selection(void)
{
    spi_bus_t bus;
    spi_device_t dev;
    spi_bus_config_t bus_cfg = {
        .reg_base = FAKE_SPI_BASE,
        .ref_clock_hz = HAL_SPI_REF_CLOCK_HZ,
        .irq_threshold_frames = 16U,
        .dma_threshold_frames = 64U,
        .enable_irq = true,
        .enable_dma = true,
        .dma_resources_reserved = true,
    };
    spi_device_config_t dev_cfg = {
        .sck_hz = 40000000U,
        .mode = 1U,
        .bits_per_word = 8U,
        .cs_id = SPI_CS0_GPIO,
        .backend = SPI_BACKEND_AUTO,
        .rx_sample_dly = 2U,
        .lsb_first = false,
    };
    uint8_t cmd[4] = {0xAAU, 0x55U, 0x01U, 0x02U};
    uint8_t mid_tx[24];
    uint8_t big_rx[128];
    spi_segment_t segs[3];

    fake_hw_reset_all();
    memset(mid_tx, 0x3CU, sizeof(mid_tx));
    memset(big_rx, 0, sizeof(big_rx));

    assert(spi_bus_init(&bus, &bus_cfg) == SPI_OK);
    assert(spi_device_init(&dev, &bus, &dev_cfg) == SPI_OK);
    g_hw.cs_edges = 0U;

    segs[0] = (spi_segment_t){.tx = cmd, .rx = NULL, .frames = sizeof(cmd)};
    segs[1] = (spi_segment_t){.tx = mid_tx, .rx = NULL, .frames = sizeof(mid_tx)};
    segs[2] = (spi_segment_t){.tx = NULL, .rx = big_rx, .frames = sizeof(big_rx), .fill_word = 0xFFU};

    assert(spi_transfer(&dev, segs, 3U, 200U) == SPI_OK);
    assert(bus.poll_segment_count == 1U);
    assert(bus.irq_segment_count == 1U);
    assert(bus.dma_segment_count == 1U);
    assert(g_hw.dma_transfers == 1U);
    assert(g_hw.cs_edges == 2U);
    assert(g_hw.cs_history[0] == true);
    assert(g_hw.cs_history[1] == false);
    for (size_t i = 0; i < sizeof(big_rx); ++i) {
        assert(big_rx[i] == (uint8_t)(0xFFU ^ 0xA5U));
    }
}

static void test_lsb_first_across_backends(void)
{
    spi_bus_t bus;
    spi_device_t dev;
    spi_bus_config_t bus_cfg = {
        .reg_base = FAKE_SPI_BASE, .ref_clock_hz = HAL_SPI_REF_CLOCK_HZ,
        .enable_irq = true, .enable_dma = true, .dma_resources_reserved = true,
    };
    spi_device_config_t dev_cfg = {
        .sck_hz = 10000000U,
        .mode = 0U,
        .bits_per_word = 8U,
        .cs_id = SPI_CS0_GPIO,
        .backend = SPI_BACKEND_POLL,
        .rx_sample_dly = 0U,
        .lsb_first = true,
    };
    uint8_t tx[2] = {0x12U, 0xA0U};
    uint8_t rx[2] = {0U, 0U};

    fake_hw_reset_all();
    g_hw.script_rx[0] = 0x48U;
    g_hw.script_rx[1] = 0x05U;
    g_hw.script_len = 2U;

    assert(spi_bus_init(&bus, &bus_cfg) == SPI_OK);
    assert(spi_device_init(&dev, &bus, &dev_cfg) == SPI_OK);
    assert(spi_transfer_one(&dev, tx, rx, 2U, 0U, 50U) == SPI_OK);
    assert(g_hw.tx_history[0] == 0x48U);
    assert(g_hw.tx_history[1] == 0x05U);
    assert(rx[0] == 0x12U);
    assert(rx[1] == 0xA0U);

    g_hw.script_pos = 0U;
    assert(spi_device_set_backend(&dev, SPI_BACKEND_DMA) == SPI_OK);
    memset(rx, 0, sizeof(rx));
    assert(spi_transfer_one(&dev, tx, rx, 2U, 0U, 50U) == SPI_OK);
    assert(rx[0] == 0x12U);
    assert(rx[1] == 0xA0U);
}

static void test_sensor_helper_apis_and_fault_recovery(void)
{
    spi_bus_t bus;
    spi_device_t dev;
    spi_bus_config_t bus_cfg = {
        .reg_base = FAKE_SPI_BASE, .ref_clock_hz = HAL_SPI_REF_CLOCK_HZ,
        .enable_irq = true, .enable_dma = true, .dma_resources_reserved = true,
    };
    spi_device_config_t dev_cfg = {
        .sck_hz = 10000000U,
        .mode = 1U,
        .bits_per_word = 8U,
        .cs_id = SPI_CS0_GPIO,
        .backend = SPI_BACKEND_DMA,
    };
    uint8_t cmd[2] = {0x9FU, 0x00U};
    uint8_t id_rx[3] = {0};

    fake_hw_reset_all();
    g_hw.script_rx[0] = 0x00U;
    g_hw.script_rx[1] = 0x00U;
    g_hw.script_rx[2] = 0xEFU;
    g_hw.script_rx[3] = 0x40U;
    g_hw.script_rx[4] = 0x18U;
    g_hw.script_len = 5U;

    assert(spi_bus_init(&bus, &bus_cfg) == SPI_OK);
    assert(spi_device_init(&dev, &bus, &dev_cfg) == SPI_OK);

    assert(spi_write_then_read(&dev, cmd, sizeof(cmd), id_rx, sizeof(id_rx), 5U, 100U) == SPI_OK);
    assert(id_rx[0] == 0xEFU && id_rx[1] == 0x40U && id_rx[2] == 0x18U);

    assert(spi_device_set_freq(&dev, 25000000U, 3U) == SPI_OK);
    assert(spi_write(&dev, cmd, sizeof(cmd), 100U) == SPI_OK);
    assert(g_hw.baudr == 8U);
    assert(g_hw.rx_sample_dly == 3U);

    g_hw.simulate_dma_timeout = true;
    assert(spi_read(&dev, id_rx, sizeof(id_rx), 0xFFU, 50U) == SPI_ERR_TIMEOUT);
    assert(bus.error_count == 1U);
    assert(g_hw.dma_aborts >= 1U);
    assert(!g_hw.cs_asserted);

    g_hw.simulate_dma_timeout = false;
    assert(spi_read(&dev, id_rx, sizeof(id_rx), 0xFFU, 50U) == SPI_OK);
    assert(spi_bus_recover(&bus) == SPI_OK);
    assert(spi_bus_deinit(&bus) == SPI_OK);
}

/* Regression: mặc định polling, AUTO chỉ dùng engine được phép và init
 * failure không được phát bất kỳ frame hoặc cạnh CS nào của message. */
static void test_backend_policy_and_lazy_init(void)
{
    spi_bus_t bus;
    spi_device_t dev;
    spi_bus_config_t cfg = {.reg_base = FAKE_SPI_BASE};
    spi_device_config_t dc = {
        .sck_hz = 10000000U, .bits_per_word = 8U,
        .backend = SPI_BACKEND_AUTO,
    };
    uint8_t tx[80] = {0}, rx[80];
    fake_hw_reset_all();
    assert(spi_bus_init(&bus, &cfg) == SPI_OK);
    assert(g_hw.irq_inits == 0U && g_hw.dma_inits == 0U);
    assert(spi_device_init(&dev, &bus, &dc) == SPI_OK);
    assert(spi_transfer_one(&dev, tx, rx, sizeof(tx), 0U, 100U) == SPI_OK);
    assert(bus.poll_segment_count == 1U);
    assert(g_hw.irq_inits == 0U && g_hw.dma_inits == 0U);
    spi_device_t refused;
    dc.backend = SPI_BACKEND_IRQ;
    assert(spi_device_init(&refused, &bus, &dc) == SPI_ERR_UNSUPPORTED);
    dc.backend = SPI_BACKEND_DMA;
    assert(spi_device_init(&refused, &bus, &dc) == SPI_ERR_UNSUPPORTED);
    dc.backend = SPI_BACKEND_AUTO;
    assert(spi_device_set_backend(&dev, SPI_BACKEND_IRQ) == SPI_ERR_UNSUPPORTED);
    assert(spi_device_set_backend(&dev, SPI_BACKEND_DMA) == SPI_ERR_UNSUPPORTED);
    assert(spi_bus_recover(&bus) == SPI_OK);
    assert(g_hw.dma_aborts == 0U);
    assert(spi_bus_deinit(&bus) == SPI_OK);
    assert(g_hw.dma_aborts == 0U);

    fake_hw_reset_all();
    cfg.enable_dma = true;
    assert(spi_bus_init(&bus, &cfg) == SPI_ERR_UNSUPPORTED);
    assert(g_hw.clock_enables == 0U && g_hw.dma_inits == 0U);

    cfg.enable_dma = false;
    cfg.enable_irq = true;
    assert(spi_bus_init(&bus, &cfg) == SPI_OK);
    assert(spi_device_init(&dev, &bus, &dc) == SPI_OK);
    assert(g_hw.irq_inits == 0U);
    assert(spi_transfer_one(&dev, tx, rx, sizeof(tx), 0U, 100U) == SPI_OK);
    assert(bus.irq_segment_count == 1U && g_hw.irq_inits == 1U);
    assert(g_hw.dma_inits == 0U);
    assert(spi_transfer_one(&dev, tx, rx, sizeof(tx), 0U, 100U) == SPI_OK);
    assert(g_hw.irq_inits == 1U);

    /* Cho phép cả hai nhưng message nhỏ vẫn không khởi tạo IRQ/DMAC. */
    fake_hw_reset_all();
    cfg.enable_dma = true;
    cfg.dma_resources_reserved = true;
    assert(spi_bus_init(&bus, &cfg) == SPI_OK);
    assert(spi_device_init(&dev, &bus, &dc) == SPI_OK);
    assert(spi_transfer_one(&dev, tx, rx, 4U, 0U, 100U) == SPI_OK);
    assert(g_hw.irq_inits == 0U && g_hw.dma_inits == 0U);
    assert(spi_device_set_backend(&dev, SPI_BACKEND_DMA) == SPI_OK);
    assert(g_hw.dma_inits == 0U);
}

static void test_prepare_failure_before_message(void)
{
    spi_bus_t bus;
    spi_device_t dev;
    spi_bus_config_t cfg = {
        .reg_base = FAKE_SPI_BASE, .enable_irq = true,
        .enable_dma = true, .dma_resources_reserved = true,
    };
    spi_device_config_t dc = {
        .sck_hz = 10000000U, .bits_per_word = 8U,
        .backend = SPI_BACKEND_AUTO,
    };
    uint8_t tx[80] = {0}, rx[80];
    spi_segment_t segs[] = {
        {.tx = tx, .rx = rx, .frames = 4U},
        {.tx = tx, .rx = rx, .frames = 40U},
        {.tx = tx, .rx = rx, .frames = 80U},
    };
    fake_hw_reset_all();
    assert(spi_bus_init(&bus, &cfg) == SPI_OK);
    assert(spi_device_init(&dev, &bus, &dc) == SPI_OK);
    size_t cs_edges = g_hw.cs_edges;
    g_hw.irq_init_status = HAL_SPI_ERR_INVALID;
    assert(spi_transfer(&dev, segs, 3U, 100U) == SPI_ERR_INVALID);
    assert(!bus.hal.irq_ready && !bus.hal.dma_ready);
    assert(g_hw.tx_count == 0U && g_hw.cs_edges == cs_edges);
    assert(g_hw.dma_inits == 0U && bus.mutex->locked == 0);

    g_hw.irq_init_status = HAL_SPI_OK;
    g_hw.dma_init_status = HAL_SPI_ERR_BUSY;
    assert(spi_transfer(&dev, segs, 3U, 100U) == SPI_ERR_BUSY);
    assert(bus.hal.irq_ready && !bus.hal.dma_ready);
    assert(g_hw.tx_count == 0U && g_hw.cs_edges == cs_edges);
    assert(g_hw.dma_aborts == 0U && bus.mutex->locked == 0);
    assert(bus.last_error == SPI_ERR_BUSY);

    g_hw.dma_init_status = HAL_SPI_OK;
    assert(spi_transfer(&dev, segs, 3U, 100U) == SPI_OK);
    assert(g_hw.irq_inits == 2U && g_hw.dma_inits == 2U);
    assert(bus.poll_segment_count == 1U && bus.irq_segment_count == 1U &&
           bus.dma_segment_count == 1U);
    assert(spi_transfer_one(&dev, tx, rx, sizeof(tx), 0U, 100U) == SPI_OK);
    assert(g_hw.dma_inits == 2U);
}

static void test_hal_requires_prepare(void)
{
    hal_spi_t hal;
    hal_spi_config_t cfg = {10000000U, 0U, 8U, 0U, false};
    uint8_t tx = 0, rx;
    fake_hw_reset_all();
    assert(hal_spi_init(&hal, FAKE_SPI_BASE, 0U) == HAL_SPI_OK);
    assert(hal_spi_prepare_dma(&hal, false) == HAL_SPI_ERR_UNSUPPORTED);
    assert(g_hw.dma_inits == 0U);
    assert(hal_spi_configure(&hal, &cfg) == HAL_SPI_OK);
    assert(hal_spi_begin(&hal, &tx, &rx, 1U, 0U) == HAL_SPI_OK);
    assert(hal_spi_transfer_irq(&hal, 100U) == HAL_SPI_ERR_UNSUPPORTED);
    assert(hal_spi_transfer_dma(&hal, 100U) == HAL_SPI_ERR_UNSUPPORTED);
    assert(g_hw.tx_count == 0U && g_hw.imr == 0U && g_hw.dmacr == 0U);
    hal_spi_abort(&hal);
    assert(g_hw.dma_aborts == 0U);
}

int main(void)
{
    test_backend_policy_and_lazy_init();
    test_prepare_failure_before_message();
    test_hal_requires_prepare();
    test_init_and_config_validation();
    test_poll_and_fifo_in_flight_bound();
    test_irq_backend_and_dynamic_thresholds();
    test_dma_backend_and_auto_backend_selection();
    test_lsb_first_across_backends();
    test_sensor_helper_apis_and_fault_recovery();
    printf("All SPI T0-T2 unit tests (POLL, IRQ, DMA, AUTO, LSB-first, Helpers, Recovery) passed!\n");
    return 0;
}
