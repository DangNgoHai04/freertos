/*
 * Tầng 3 (T3 - Tiny1C VoSPI Frame Acquisition): Giao thức thu nhận khung hình
 * nhiệt độ/hình ảnh qua giao tiếp SPI0 (ủy quyền xuống Tầng 2 spi_transfer_one).
 */

#ifndef TINY1C_VOSPI_H
#define TINY1C_VOSPI_H

#include <stdbool.h>
#include <stdint.h>
#include "core.h"
#include "hal_spi.h"

#ifdef __cplusplus
extern "C" {
#endif

#define VOSPI_TX_DUMMY_LEN      (512)
#define VOSPI_ROW_SIZE          (IMG_W * 2)     /* 512 bytes / hàng */
#define VOSPI_COL_SIZE          (IMG_H)         /* 192 hàng / khung hình đơn */
#define VOSPI_DUAL_COL_SIZE     (DUAL_HEIGHT)   /* 384 hàng / khung hình kép (Image + Temp) */

#define VOSPI_HEADER_IMAGE      (0xAAU)         /* Mã lệnh thu khung chính (Image 256x192 hoặc Temp đơn Y16 256x192) */
#define VOSPI_HEADER_TEMP       (0xCCU)         /* Mã lệnh chỉ dùng cho nửa khung nhiệt thứ 2 trong MODE_DUAL (256x384) */

/*
 * Cấu trúc 32-byte VoSPI Frame Header của ASIC-2121W (nằm tại offset 64
 * trong gói phản hồi Header 512-byte).
 */
typedef struct {
    uint8_t  frame_valid;       /* Byte 0: bit 0 = 1 (hợp lệ), 0 (không hợp lệ) */
    uint8_t  reserved0;         /* Byte 1: dự phòng */
    uint16_t frame_index;       /* Byte 2-3: Số thứ tự khung hình (Little-Endian) */
    uint8_t  shutter_state;     /* Byte 4: 0 = Đóng màn trập, 1 = Mở màn trập */
    uint16_t vtemp;             /* Byte 5-6: Nhiệt độ cảm biến nội Vtemp */
    uint16_t shutter_temp;      /* Byte 7-8: Nhiệt độ màn trập */
    uint8_t  gain_state;        /* Byte 9: 1 = High Gain, 0 = Low Gain */
    uint8_t  temp_area;         /* Byte 10: Vùng nhiệt độ */
    uint8_t  over_exposure;     /* Byte 11: Trạng thái quá phơi sáng */
    uint8_t  pix_freeze_state;  /* Byte 12: 1 = Đóng băng ảnh, 0 = Bình thường */
    uint8_t  reserved1[19];     /* Byte 13-31: dự phòng */
} __attribute__((packed)) vospi_frame_header_t;

/* Cấu trúc thống kê chất lượng thu nhận khung VoSPI ở Tầng 3 */
typedef struct {
    uint32_t frames_ok;             /* Số khung thu thành công và hợp lệ (frame_valid bit 0 == 1) */
    uint32_t frames_invalid;        /* Số khung có bit frame_valid == 0 (ASIC chưa sẵn sàng) */
    uint32_t header_err;            /* Số lần lỗi đường truyền SPI khi đọc gói Header 512B */
    uint32_t payload_err;           /* Số lần lỗi đường truyền SPI khi đọc gói Payload 96KB */
    uint32_t frame_drops;           /* Số lần phát hiện nhảy số thứ tự frame_index */
    uint32_t shutter_closed_frames; /* Số khung thu trong lúc màn trập đang đóng (shutter_state == 0) */
    uint32_t freeze_frames;         /* Số khung bị đóng băng điểm ảnh (pix_freeze_state != 0) */
} vospi_stats_t;

/* Khởi tạo luồng thu nhận VoSPI thông qua Tầng 2 SPI0 */
int spi_stream_init(uint8_t spi_id, int cs_gpio_pin);

/* Đổi tốc độ xung nhịp SCK của kênh VoSPI thông qua Tầng 2 */
void spi_stream_set_clock_config(uint16_t baud_div, bool use_25m_osc);

/* Đặt / lấy chu kỳ lấy khung nhiệt trong chế độ Dual Interval (tương thích SDK hãng) */
void spi_set_frame_interval(uint8_t interval);
uint8_t spi_get_frame_interval(void);

/* Điều khiển chân CS (được Tầng 2 giữ tự động trong suốt giao dịch) */
void spi_cs_set(bool level);

/*
 * Thu nhận 1 khung hình hoàn chỉnh (96 KB đơn hoặc 192 KB kép) qua Tầng 2 SPI0:
 * - Gói Header 512B + kiểm tra frame_valid + trễ 1ms + Khung 96KB liền mạch qua DMA Tầng 2/Tầng 1.
 * - Nếu buf_len >= DUAL_FRAME_BYTES (192 KB), tự động thu 2 khung liên tiếp:
 *   96 KB đầu là ảnh YUV422 (0xAA) và 96 KB sau là nhiệt độ RAW16 (0xCC).
 */
int spi_get_frame(uint8_t *out_buffer, uint32_t buf_len, uint8_t header_type);

/* Lấy con trỏ tới cấu trúc 32-byte VoSPI Frame Header của khung chính (0xAA) và khung phụ (0xCC) */
const vospi_frame_header_t *spi_get_last_header(void);
const vospi_frame_header_t *spi_get_last_temp_header(void);
const uint8_t *spi_get_last_rx_dummy(void);

/* Truy vấn và đặt lại bộ đếm thống kê chất lượng khung VoSPI */
void spi_get_vospi_stats(vospi_stats_t *out_stats);
void spi_reset_vospi_stats(void);

/* Các hàm cầu nối tương thích ngược */
int cvi_spi_transfer(uint8_t spi_id, const uint8_t *tx_buf, uint8_t *rx_buf, uint32_t len);
void cvi_gpio_set_value(int pin, int value);

#ifdef __cplusplus
}
#endif

#endif /* TINY1C_VOSPI_H */
