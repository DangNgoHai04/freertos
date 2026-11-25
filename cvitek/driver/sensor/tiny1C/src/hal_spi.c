/*
 * Tầng 3 (T3 - Tiny1C SPI Transport Adapter): Cầu nối giao tiếp SPI0 tốc độ cao
 * ủy quyền hoàn toàn xuống API Tầng 2 (cvitek/driver/spi/include/spi.h).
 */

#include "hal_spi.h"
#include <stdio.h>
#include <string.h>

#if defined(TINY1C_HAS_T2_SPI)
static spi_bus_t s_default_spi0_bus;
static spi_bus_t *s_bound_spi0_bus = NULL;
static spi_device_t s_tiny1c_spi_dev;
#endif

static bool s_spi0_inited = false;
static bool s_dma_resources_reserved = false; /* Cam kết dành riêng kênh DMA 0/1 cho SPI0 */
static uint32_t s_current_sck_hz = THERMAL_SPI_DEFAULT_SCK_HZ;
#if !defined(TINY1C_HAS_T2_SPI) || !defined(__riscv)
static uint16_t s_host_sim_frame_idx = 0U;
#endif

void hal_spi_set_dma_reserved(bool reserved)
{
    s_dma_resources_reserved = reserved;
}

uint32_t hal_spi_get_freq_hz(void)
{
    return s_current_sck_hz;
}

int hal_spi_set_freq_hz(uint32_t sck_hz)
{
    if (sck_hz == 0U) {
        return -1;
    }
    if (!s_spi0_inited) {
        spi0_dsi_hw_init();
    }
#if defined(TINY1C_HAS_T2_SPI) && defined(__riscv)
    if (!s_spi0_inited) {
        return -1;
    }
    if (spi_device_set_freq(&s_tiny1c_spi_dev, sck_hz, THERMAL_SPI_DEFAULT_RX_DLY) != SPI_OK) {
        return -2;
    }
#endif
    s_current_sck_hz = sck_hz;
    printf("[T3-SPI] Configured SPI0 SCK = %d Hz (RefClk=187.5MHz)\n", (int)s_current_sck_hz);
    return 0;
}

#if defined(TINY1C_HAS_T2_SPI)
int hal_spi_bind_bus(spi_bus_t *bus)
{
    if ((bus == NULL) || (!bus->initialized)) {
        return -1;
    }
    s_bound_spi0_bus = bus;

    const spi_device_config_t cfg = {
        .sck_hz        = THERMAL_SPI_DEFAULT_SCK_HZ,
        .mode          = 3U, /* SPI Mode 3 (CPOL=1, CPHA=1) cho VoSPI Tiny1C */
        .bits_per_word = 8U,
        .cs_id         = SPI_CS0_GPIO,
        .backend       = s_dma_resources_reserved ? SPI_BACKEND_AUTO : SPI_BACKEND_POLL,
        .rx_sample_dly = THERMAL_SPI_DEFAULT_RX_DLY,
        .lsb_first     = false
    };
    if (spi_device_init(&s_tiny1c_spi_dev, s_bound_spi0_bus, &cfg) != SPI_OK) {
        return -2;
    }
    s_spi0_inited = true;
    return 0;
}

spi_device_t *hal_spi_get_device(void)
{
    return s_spi0_inited ? &s_tiny1c_spi_dev : NULL;
}
#endif

void spi0_dsi_hw_init(void)
{
    if (s_spi0_inited) {
        return;
    }

#if defined(TINY1C_HAS_T2_SPI) && defined(__riscv)
    if (s_bound_spi0_bus == NULL) {
        /*
         * Ủy quyền khởi tạo Bus SPI0 (0x04180000, clock nguồn 187.5MHz) xuống Tầng 2:
         * Tầng 2 -> Tầng 1 (hal_spi_init) -> Tầng 0 (hal_spi_platform_init: Clock, Reset,
         * MIPI DSI D-PHY Digital Bypass, Pinmux và GPIOC17 CS0).
         * DMA chỉ được cho phép khi board đã xác nhận tài nguyên dành riêng.
         */
        const spi_bus_config_t bus_cfg = {
            .reg_base               = (uintptr_t)SPI0_BASE,
            .ref_clock_hz           = 0U,
            .irq_threshold_frames   = SPI_DEFAULT_IRQ_THRESHOLD_FRAMES,
            .dma_threshold_frames   = SPI_DEFAULT_DMA_THRESHOLD_FRAMES,
            .enable_irq             = s_dma_resources_reserved,
            .enable_dma             = s_dma_resources_reserved,
            .dma_resources_reserved = s_dma_resources_reserved
        };
        if (spi_bus_init(&s_default_spi0_bus, &bus_cfg) != SPI_OK) {
            return;
        }
        s_bound_spi0_bus = &s_default_spi0_bus;
    }

    const spi_device_config_t cfg = {
        .sck_hz        = THERMAL_SPI_DEFAULT_SCK_HZ,
        .mode          = 3U, /* SPI Mode 3 (CPOL=1, CPHA=1) cho VoSPI Tiny1C */
        .bits_per_word = 8U,
        .cs_id         = SPI_CS0_GPIO,
        .backend       = s_dma_resources_reserved ? SPI_BACKEND_AUTO : SPI_BACKEND_POLL,
        .rx_sample_dly = THERMAL_SPI_DEFAULT_RX_DLY,
        .lsb_first     = false
    };
    if (spi_device_init(&s_tiny1c_spi_dev, s_bound_spi0_bus, &cfg) == SPI_OK) {
        s_spi0_inited = true;
    }
#else
    s_spi0_inited = true;
#endif
}

void spi0_dsi_configure_vospi(uint16_t baud_div, bool use_25m_osc)
{
    (void)use_25m_osc;
    if (!s_spi0_inited) {
        spi0_dsi_hw_init();
    }

    if (baud_div < 2U) {
        baud_div = 2U;
    }
    if ((baud_div & 1U) != 0U) {
        baud_div++;
    }

    /*
     * Đồng bộ chặt chẽ với Tầng 0 (hal_spi_platform.c) và Tầng 2 (spi.c):
     * Tầng 0 cung cấp xung nhịp tham chiếu ssi_clk cố định 187.5 MHz (FPLL 1.5 GHz / 8).
     * Tầng 2 (spi_device_set_freq) luôn tính BAUDR từ bus->hal.ref_clock_hz (187.5 MHz),
     * không tự đổi mux sang XTAL 25 MHz. Do đó tính target_sck_hz trực tiếp từ ref_clock_hz.
     */
    uint32_t src_hz = 187500000U;
#if defined(TINY1C_HAS_T2_SPI) && defined(__riscv)
    if ((s_bound_spi0_bus != NULL) && (s_bound_spi0_bus->hal.ref_clock_hz > 0U)) {
        src_hz = s_bound_spi0_bus->hal.ref_clock_hz;
    }
#endif
    uint32_t target_sck_hz = src_hz / (uint32_t)baud_div;
    (void)hal_spi_set_freq_hz(target_sck_hz);
}

void spi0_dsi_cs_set(int cs_gpio_pin, bool level)
{
    /*
     * Ở kiến trúc phân tầng T0-T1-T2, chân CS0 (GPIOC17) được điều khiển nguyên tử
     * bên trong spi_transfer() / spi_transfer_one() của Tầng 2 & Tầng 1.
     */
    (void)cs_gpio_pin;
    (void)level;
}

int hal_spi_transfer_frame(const void *tx_buf, void *rx_buf, size_t len,
                           uint16_t dummy_fill, uint32_t timeout_ms)
{
    if (len == 0U) {
        return 0;
    }
    if (!s_spi0_inited) {
        spi0_dsi_hw_init();
    }

#if defined(TINY1C_HAS_T2_SPI) && defined(__riscv)
    if (!s_spi0_inited) {
        return -1;
    }
    spi_status_t st = spi_transfer_one(&s_tiny1c_spi_dev,
                                       tx_buf,
                                       rx_buf,
                                       len,
                                       dummy_fill,
                                       timeout_ms);
    return (st == SPI_OK) ? 0 : (int)st;
#else
    (void)dummy_fill;
    (void)timeout_ms;
    if (rx_buf != NULL) {
        if (tx_buf != NULL) {
            memcpy(rx_buf, tx_buf, len);
        } else {
            memset(rx_buf, 0, len);
        }
        /* Giả lập VoSPI Frame Header hợp lệ tại offset 64 khi thu gói Header 512 byte */
        if (len == 512U) {
            uint8_t *b = (uint8_t *)rx_buf;
            uint16_t idx = s_host_sim_frame_idx++;
            b[64] = 1U;                         /* frame_valid = 1 (bit 0 = 1) */
            b[66] = (uint8_t)(idx & 0xFFU);     /* frame_index LSB */
            b[67] = (uint8_t)(idx >> 8U);       /* frame_index MSB */
            b[68] = 1U;                         /* shutter_state = 1 (Open) */
            b[69] = 0x90U;                      /* vtemp LSB (0x0B90 = 2960) */
            b[70] = 0x0BU;                      /* vtemp MSB */
            b[73] = 1U;                         /* gain_state = 1 (High Gain) */
            b[76] = 0U;                         /* pix_freeze_state = 0 (Normal) */
        }
    }
    return 0;
#endif
}

int hal_spi_recover_bus(void)
{
#if defined(TINY1C_HAS_T2_SPI) && defined(__riscv)
    if (s_bound_spi0_bus == NULL) {
        return -1;
    }
    spi_status_t st = spi_bus_recover(s_bound_spi0_bus);
    return (st == SPI_OK) ? 0 : (int)st;
#else
    return 0;
#endif
}

int spi0_dsi_transfer_burst(const uint8_t *tx_buf, uint8_t *rx_buf, uint32_t len)
{
    uint16_t fill = (tx_buf != NULL) ? (uint16_t)tx_buf[0] : 0x0055U;
    return hal_spi_transfer_frame(tx_buf, rx_buf, (size_t)len, fill, THERMAL_SPI_TIMEOUT_MS);
}
