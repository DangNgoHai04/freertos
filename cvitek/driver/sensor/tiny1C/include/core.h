/*
 * Tầng 3 (T3 - Tiny1C High-Level Core API): Điều khiển vòng đời Camera nhiệt,
 * truy vấn bức xạ nhiệt (TPD), tinh chỉnh ISP (Tính năng 5) và Hiệu chuẩn (Tính năng 6).
 */

#ifndef TINY1C_CORE_H
#define TINY1C_CORE_H

#include <stdbool.h>
#include <stdint.h>
#include "all_config.h"
#include "falcon_cmd.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Kích thước khung hình chuẩn của cảm biến Tiny1C (ASIC-2121W) */
#define IMG_W                    (256)
#define IMG_H                    (192)
#define FRAME_BYTES              (IMG_W * IMG_H * 2)        /* 98,304 bytes = 96 KB */
#define DUAL_HEIGHT              (384)
#define DUAL_FRAME_BYTES         (IMG_W * DUAL_HEIGHT * 2)  /* 196,608 bytes = 192 KB */

typedef enum {
    MODE_IMAGE           = 0,  /* Ảnh màu giả lập YUV422 (Header 0xAA, 256x192, SCK 23.4375 MHz) */
    MODE_TEMP            = 1,  /* Ma trận nhiệt độ Y16 (y16_preview_start + Header 0xAA, 256x192) */
    MODE_DUAL            = 2   /* Kép: YUV422 (0xAA, 96KB) + RAW16 (0xCC, 96KB) (256x384, SCK 46.875 MHz) */
} camera_mode_t;

/* Thông tin định danh thiết bị & Firmware đọc từ ASIC-2121W lúc khởi tạo */
typedef struct {
    char fw_compile_date[65];   /* Ngày biên dịch Firmware (DEV_INFO_FW_COMPILE_DATE) */
    char project_info[65];      /* Tên dự án / dòng chip (DEV_INFO_PROJECT_INFO) */
    char fw_build_version[65];  /* Phiên bản bản dựng Firmware (DEV_INFO_FW_BUILD_VERSION_INFO) */
    char pn[65];                /* Mã sản phẩm Part Number (DEV_INFO_GET_PN) */
    char sn[65];                /* Số sê-ri Serial Number (DEV_INFO_GET_SN) */
    bool probed;                /* Đánh dấu đã truy vấn thành công từ ASIC */
} tiny1c_device_info_t;

/* Cực trị nhiệt độ toàn khung (Tọa độ 0-based: X=0..255, Y=0..191) */
typedef struct {
    float    max_temp; /* Nhiệt độ cao nhất toàn khung (°C) */
    float    min_temp; /* Nhiệt độ thấp nhất toàn khung (°C) */
    uint16_t max_x;    /* Tọa độ X của điểm nóng nhất */
    uint16_t max_y;    /* Tọa độ Y của điểm nóng nhất */
    uint16_t min_x;    /* Tọa độ X của điểm lạnh nhất */
    uint16_t min_y;    /* Tọa độ Y của điểm lạnh nhất */
} __attribute__((packed)) extremes_t;

/* Kết quả đo nhiệt độ đoạn thẳng / vùng chữ nhật (đã quy đổi ra °C và tọa độ 0-based) */
typedef struct {
    float    avg_temp; /* Nhiệt độ trung bình (°C) */
    float    max_temp; /* Nhiệt độ cực đại (°C) */
    float    min_temp; /* Nhiệt độ cực tiểu (°C) */
    uint16_t max_x;    /* Tọa độ X điểm nóng nhất */
    uint16_t max_y;    /* Tọa độ Y điểm nóng nhất */
    uint16_t min_x;    /* Tọa độ X điểm lạnh nhất */
    uint16_t min_y;    /* Tọa độ Y điểm lạnh nhất */
} __attribute__((packed)) region_temp_t;

/* =========================================================================
 * 1. ĐIỀU KHIỂN VÒNG ĐỜI CAMERA & MÀN TRẬP (PREVIEW & SHUTTER / FFC)
 * ========================================================================= */
ir_error_t core_init(uint8_t i2c_bus, uint8_t slave_addr);
ir_error_t core_probe_device_info(tiny1c_device_info_t *out_info);
const tiny1c_device_info_t *core_get_device_info(void);
ir_error_t core_start(camera_mode_t mode, uint8_t fps);
ir_error_t core_stop(void);
camera_mode_t core_get_mode(void);
ir_error_t trigger_shutter(void);
ir_error_t core_set_auto_shutter_param(enum prop_auto_shutter_params param, uint16_t value);
ir_error_t core_get_auto_shutter_param(enum prop_auto_shutter_params param, uint16_t *value);
ir_error_t core_set_fall_protection(bool enable, uint16_t high_gain_thr, uint16_t low_gain_thr);

/* =========================================================================
 * 2. TRUY VẤN ĐO NHIỆT ĐỘ BỨC XẠ (TPD - TEMPERATURE MEASUREMENT)
 * ========================================================================= */
ir_error_t query_extremes(extremes_t *extremes);
ir_error_t query_point_temp(uint16_t x, uint16_t y, float *temp_celsius);
ir_error_t query_line_temp(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, region_temp_t *out_info);
ir_error_t query_rect_temp(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, region_temp_t *out_info);

/* =========================================================================
 * 3. TÍNH NĂNG 5: XỬ LÝ HÌNH ẢNH NÂNG CAO & ISP (IMAGE ENHANCEMENT)
 * ========================================================================= */
ir_error_t set_palette(uint8_t palette_id);
ir_error_t get_palette(uint8_t *palette_id);
ir_error_t core_zoom_center_up(enum zoom_scale_step step);
ir_error_t core_zoom_center_down(enum zoom_scale_step step);
ir_error_t core_zoom_position_up(enum zoom_scale_step step, uint16_t x, uint16_t y);
ir_error_t core_zoom_position_down(enum zoom_scale_step step, uint16_t x, uint16_t y);
ir_error_t core_set_image_param(enum prop_image_params param, uint16_t value);
ir_error_t core_get_image_param(enum prop_image_params param, uint16_t *value);
ir_error_t core_set_mirror_flip(enum mirror_flip_types mirror_flip);

/* =========================================================================
 * 4. TÍNH NĂNG 6: CĂN CHỈNH & HIỆU CHUẨN THIẾT BỊ (CALIBRATION)
 * ========================================================================= */
ir_error_t core_dpc_auto_calibrate(uint16_t wait_time_sec);
ir_error_t core_dpc_add_point(uint16_t x, uint16_t y);
ir_error_t core_dpc_remove_point(uint16_t x, uint16_t y);
ir_error_t core_dpc_save_config(void);
ir_error_t core_rmcover_auto_calc(uint8_t zoom_scale);
ir_error_t core_rmcover_enable(bool enable);
ir_error_t core_recal_1point_kelvin(uint16_t target_temp_k);
ir_error_t core_recal_1point_celsius(float target_temp_c);
ir_error_t core_recal_2point_kelvin(enum tpd_ktbt_recal_point_idx point_idx, uint16_t target_temp_k);
ir_error_t core_recal_2point_celsius(enum tpd_ktbt_recal_point_idx point_idx, float target_temp_c);
ir_error_t core_set_tpd_param(enum prop_tpd_params param, uint16_t value);
ir_error_t core_get_tpd_param(enum prop_tpd_params param, uint16_t *value);
ir_error_t core_set_env_params(float emissivity, float distance_m,
                               float refl_temp_c, float atm_temp_c,
                               float transmittance);
ir_error_t core_set_gain_mode(bool high_gain);
ir_error_t core_save_config(enum prop_spi_cfg_module module);
ir_error_t core_load_config(enum prop_spi_cfg_module module);
ir_error_t core_restore_default_cfg(enum prop_default_cfg cfg_type);

/* =========================================================================
 * 5. HÀM TIỆN ÍCH CHUYỂN ĐỔI ĐƠN VỊ NHIỆT ĐỘ
 * ========================================================================= */
static inline float raw_to_celsius(uint16_t raw_val)
{
    return ((float)raw_val / 64.0f) - 273.15f;
}

static inline uint16_t celsius_to_raw(float celsius)
{
    return (uint16_t)((celsius + 273.15f) * 64.0f);
}

static inline float tpd_raw_to_celsius(uint16_t tpd_val)
{
    return ((float)tpd_val / 16.0f) - 273.15f;
}

#ifdef __cplusplus
}
#endif

#endif /* TINY1C_CORE_H */
