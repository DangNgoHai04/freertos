/*
 * Tầng 3 (T3 - Tiny1C VoSPI Frame Acquisition): Thu nhận khung hình nhiệt độ /
 * hình ảnh qua Tầng 2 SPI0 (polling mặc định, DMA chỉ khi được bật rõ ràng).
 */

#include "vospi.h"
#include "data_rw.h"
#include <stdio.h>
#include <string.h>

static uint8_t s_spi_id = 0U;
static int s_cs_gpio_pin = SPI0_DSI_CS_PIN_DEFAULT;
static uint16_t s_baud_div = SPI0_BAUDR_DIV_DEFAULT; /* 187.5MHz / 8 = 23.4375 MHz */
static bool s_use_25m_osc = false;
static uint8_t s_frame_interval = (uint8_t)THERMAL_DUAL_DEFAULT_INTERVAL;
static bool s_has_prev_idx = false;
static uint16_t s_prev_frame_idx = 0U;

/* Bộ đệm trao đổi gói Header 512-byte căn chỉnh 64-byte theo dòng Cache C906 */
static uint8_t s_tx_dummy[VOSPI_TX_DUMMY_LEN] __attribute__((aligned(64)));
static uint8_t s_rx_dummy[VOSPI_TX_DUMMY_LEN] __attribute__((aligned(64)));

/* Tách riêng Header của khung chính (0xAA) và khung nhiệt phụ trong MODE_DUAL (0xCC) */
static vospi_frame_header_t s_last_vospi_hdr;
static vospi_frame_header_t s_last_temp_vospi_hdr;
static vospi_stats_t s_vospi_stats;

static bool vospi_all_ff(const uint8_t *bytes, size_t len)
{
    for (size_t i = 0; i < len; ++i) {
        if (bytes[i] != 0xFFU) return false;
    }
    return true;
}

void cvi_gpio_set_value(int pin, int value)
{
    spi0_dsi_cs_set(pin, value != 0);
}

int cvi_spi_transfer(uint8_t spi_id, const uint8_t *tx_buf, uint8_t *rx_buf, uint32_t len)
{
    (void)spi_id;
    return spi0_dsi_transfer_burst(tx_buf, rx_buf, len);
}

void spi_stream_set_clock_config(uint16_t baud_div, bool use_25m_osc)
{
    s_baud_div    = baud_div;
    s_use_25m_osc = use_25m_osc;
    spi0_dsi_configure_vospi(s_baud_div, s_use_25m_osc);
}

void spi_set_frame_interval(uint8_t interval)
{
    s_frame_interval = (interval == 0U) ? 1U : interval;
}

uint8_t spi_get_frame_interval(void)
{
    return s_frame_interval;
}

void spi_get_vospi_stats(vospi_stats_t *out_stats)
{
    if (out_stats != NULL) {
        *out_stats = s_vospi_stats;
    }
}

void spi_reset_vospi_stats(void)
{
    memset(&s_vospi_stats, 0, sizeof(s_vospi_stats));
    s_has_prev_idx = false;
}

int spi_stream_init(uint8_t spi_id, int cs_gpio_pin)
{
    s_spi_id      = spi_id;
    s_cs_gpio_pin = (cs_gpio_pin >= 0) ? cs_gpio_pin : SPI0_DSI_CS_PIN_DEFAULT;

    memset(s_tx_dummy, 0, sizeof(s_tx_dummy));
    memset(s_rx_dummy, 0, sizeof(s_rx_dummy));
    memset(&s_last_vospi_hdr, 0, sizeof(s_last_vospi_hdr));
    memset(&s_last_temp_vospi_hdr, 0, sizeof(s_last_temp_vospi_hdr));
    spi_reset_vospi_stats();
    s_last_vospi_hdr.frame_valid   = 0U;
    s_last_vospi_hdr.shutter_state = 1U;
    s_last_vospi_hdr.gain_state    = 1U;

    /* Khởi tạo Bus SPI0 và Thiết bị Tiny1C thông qua Tầng 2 */
    spi0_dsi_hw_init();
    spi0_dsi_configure_vospi(s_baud_div, s_use_25m_osc);
#if defined(__riscv)
    if (hal_spi_get_device() == NULL) return -1;
#endif
    if (hal_spi_set_freq_hz(hal_spi_get_freq_hz()) != 0) return -1;

    printf("[Thermal VOSPI] Initialized on SPI%d via T2 Driver, CS Pin: 0x%x\n",
           (int)s_spi_id, (int)s_cs_gpio_pin);
    return 0;
}

void spi_cs_set(bool level)
{
    cvi_gpio_set_value(s_cs_gpio_pin, level ? 1 : 0);
}

const vospi_frame_header_t *spi_get_last_header(void)
{
    return &s_last_vospi_hdr;
}

const vospi_frame_header_t *spi_get_last_temp_header(void)
{
    return &s_last_temp_vospi_hdr;
}

const uint8_t *spi_get_last_rx_dummy(void)
{
    return s_rx_dummy;
}

/*
 * Thu nhận 1 khung đơn 96 KB (256x192x2 = 98,304 byte) qua Tầng 2:
 *   1. Gửi gói Header 512 byte (byte [0] = header_type: 0xAA hoặc 0xCC) trong một phiên CS LOW.
 *   2. Bóc tách 32-byte vospi_frame_header_t tại offset 64 và kiểm tra dữ liệu lỗi
 *      (header toàn 0xFF hoặc frame_valid bit 0 bằng 0).
 *   3. Chờ 1ms theo đặc tả ASIC-2121W để DMA nội bộ của cảm biến chuẩn bị khung.
 *   4. Gọi hal_spi_transfer_frame(NULL, dst_96kb, FRAME_BYTES, 0x0055, timeout):
 *      Tầng 2/Tầng 1 tự động giữ CS GPIOC17 LOW, phát byte điền 0x55 mà không cần
 *      bộ đệm TX 512B trong vòng lặp 192 hàng. T2 chọn polling hoặc DMA theo cấu hình.
 */
static int vospi_read_single_frame(uint8_t *dst_96kb, uint8_t header_type)
{
    memset(s_tx_dummy, 0, sizeof(s_tx_dummy));
    s_tx_dummy[0] = header_type;

    int ret = hal_spi_transfer_frame(s_tx_dummy,
                                     s_rx_dummy,
                                     VOSPI_TX_DUMMY_LEN,
                                     0x0000U,
                                     THERMAL_SPI_TIMEOUT_MS);
    if (ret != 0) {
        s_vospi_stats.header_err++;
        (void)hal_spi_recover_bus();
        return -2;
    }

    vospi_frame_header_t *target_hdr = (header_type == VOSPI_HEADER_TEMP)
                                           ? &s_last_temp_vospi_hdr
                                           : &s_last_vospi_hdr;
    size_t hdr_off = 64U;
    if ((s_rx_dummy[64] & 0x01U) == 0U) {
        size_t max_off = VOSPI_TX_DUMMY_LEN - sizeof(vospi_frame_header_t);
        for (size_t off = 64U; off <= max_off; ++off) {
            if (s_rx_dummy[off] != 0U) {
                hdr_off = off;
                break;
            }
        }
    }
    memcpy(target_hdr, &s_rx_dummy[hdr_off], sizeof(vospi_frame_header_t));

    /* MISO treo ở mức HIGH trả về 0xFF; bit frame_valid vẫn là 1 giả. */
    if (vospi_all_ff((const uint8_t *)target_hdr, sizeof(*target_hdr))) {
        s_vospi_stats.header_err++;
        return -2;
    }

    /* Kiểm tra frame_valid trước khi chấp nhận payload. */
    if ((target_hdr->frame_valid & 0x01U) == 0U) {
        s_vospi_stats.frames_invalid++;
        return -4;
    }

    /* Theo dõi độ liên tục của frame_index và trạng thái màn trập / đóng băng ảnh trên khung chính */
    if (header_type != VOSPI_HEADER_TEMP) {
        if (s_has_prev_idx) {
            uint16_t expected = (uint16_t)(s_prev_frame_idx + 1U);
            if ((target_hdr->frame_index != expected) &&
                (target_hdr->frame_index != s_prev_frame_idx)) {
                s_vospi_stats.frame_drops++;
            }
        }
        s_has_prev_idx   = true;
        s_prev_frame_idx = target_hdr->frame_index;
        if (target_hdr->shutter_state == 0U) {
            s_vospi_stats.shutter_closed_frames++;
        }
        if (target_hdr->pix_freeze_state != 0U) {
            s_vospi_stats.freeze_frames++;
        }
    }

    delay_ms(1U);

    ret = hal_spi_transfer_frame(NULL,
                                 dst_96kb,
                                 (size_t)FRAME_BYTES,
                                 0x0055U,
                                 THERMAL_SPI_TIMEOUT_MS);
    if (ret != 0) {
        s_vospi_stats.payload_err++;
        (void)hal_spi_recover_bus();
        return -3;
    }

    if (vospi_all_ff(dst_96kb, (size_t)FRAME_BYTES)) {
        s_vospi_stats.payload_err++;
        return -3;
    }

    s_vospi_stats.frames_ok++;
    return 0;
}

int spi_get_frame(uint8_t *out_buffer, uint32_t buf_len, uint8_t header_type)
{
    if ((out_buffer == NULL) || (buf_len < (uint32_t)FRAME_BYTES)) {
        return -1;
    }

    if (buf_len >= (uint32_t)DUAL_FRAME_BYTES) {
        /*
         * Chế độ Kép (MODE_DUAL - 192 KB):
         * Theo chuẩn SDK hãng (image_and_temp_frame_interval_get), thu liên tiếp:
         *   - Nửa đầu 96 KB: Khung ảnh YUV422 (VOSPI_HEADER_IMAGE = 0xAA)
         *   - Nửa sau 96 KB: Ma trận nhiệt độ RAW16 (VOSPI_HEADER_TEMP = 0xCC)
         */
        int ret = vospi_read_single_frame(out_buffer, VOSPI_HEADER_IMAGE);
        if (ret != 0) {
            return ret;
        }
        return vospi_read_single_frame(out_buffer + FRAME_BYTES, VOSPI_HEADER_TEMP);
    }

    return vospi_read_single_frame(out_buffer, header_type);
}
