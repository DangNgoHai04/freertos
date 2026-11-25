/*
 * Tầng 3 (T3 - Tiny1C High-Level Core Implementation): Triển khai nghiệp vụ
 * điều khiển vòng đời Camera, truy vấn TPD (chuẩn hóa tọa độ 1-based <-> 0-based),
 * xử lý ảnh ISP (Tính năng 5) và Hiệu chuẩn (Tính năng 6).
 */

#include "core.h"
#include "data_rw.h"
#include "vdcmd.h"
#include "falcon_cmd.h"
#include "hal_spi.h"
#include <stdio.h>
#include <string.h>

static camera_mode_t s_current_mode = MODE_TEMP;
static bool s_is_started = false;
static tiny1c_device_info_t s_dev_info = { 0 };

/*
 * Chuẩn hóa tọa độ giữa hệ 0-based của ứng dụng (0..IMG_W-1, 0..IMG_H-1)
 * và hệ 1-based của tập lệnh TPD/Zoom/DPC trên ASIC-2121W (1..IMG_W, 1..IMG_H).
 */
static inline uint16_t to_asic_coord_x(uint16_t x0)
{
    return (x0 >= IMG_W) ? (uint16_t)IMG_W : (uint16_t)(x0 + 1U);
}

static inline uint16_t to_asic_coord_y(uint16_t y0)
{
    return (y0 >= IMG_H) ? (uint16_t)IMG_H : (uint16_t)(y0 + 1U);
}

static inline uint16_t from_asic_coord(uint16_t asic_val, uint16_t max_dim)
{
    if (asic_val == 0U) {
        return 0U;
    }
    uint16_t z = (uint16_t)(asic_val - 1U);
    return (z >= max_dim) ? (uint16_t)(max_dim - 1U) : z;
}

/* =========================================================================
 * 1. ĐIỀU KHIỂN VÒNG ĐỜI CAMERA & MÀN TRẬP (PREVIEW & SHUTTER / FFC)
 * ========================================================================= */

static void copy_dev_info_str(char *dst, size_t dst_size, const uint8_t *src)
{
    if ((dst == NULL) || (dst_size == 0U) || (src == NULL)) {
        return;
    }
    memcpy(dst, src, dst_size - 1U);
    dst[dst_size - 1U] = '\0';
}

ir_error_t core_probe_device_info(tiny1c_device_info_t *out_info)
{
    uint8_t raw[64];
    memset(&s_dev_info, 0, sizeof(s_dev_info));

    memset(raw, 0, sizeof(raw));
    if (get_device_info(DEV_INFO_CHIP_ID, raw) == IR_SUCCESS) {
        data_rw_mark_probe_ok(0U);
        copy_dev_info_str(s_dev_info.project_info, sizeof(s_dev_info.project_info), raw);
    }

    data_rw_set_probe_stage(1U);
    memset(raw, 0, sizeof(raw));
    ir_error_t probe_status = get_device_info(DEV_INFO_PROJECT_INFO, raw);
    if (probe_status != IR_SUCCESS) {
        printf("[Camera] Probe PROJECT failed: ir_status=%d\n", (int)probe_status);
        if (probe_status != IR_CHECK_DONE_FAIL) {
            return IR_NOT_GET_DEVICE_INFO;
        }
    } else {
        data_rw_mark_probe_ok(4U);
        copy_dev_info_str(s_dev_info.project_info, sizeof(s_dev_info.project_info), raw);
    }

    data_rw_set_probe_stage(2U);
    memset(raw, 0, sizeof(raw));
    if (get_device_info(DEV_INFO_FW_COMPILE_DATE, raw) == IR_SUCCESS) {
        data_rw_mark_probe_ok(1U);
        copy_dev_info_str(s_dev_info.fw_compile_date, sizeof(s_dev_info.fw_compile_date), raw);
    }

    data_rw_set_probe_stage(3U);
    memset(raw, 0, sizeof(raw));
    if (get_device_info(DEV_INFO_FW_BUILD_VERSION_INFO, raw) == IR_SUCCESS) {
        data_rw_mark_probe_ok(5U);
        copy_dev_info_str(s_dev_info.fw_build_version, sizeof(s_dev_info.fw_build_version), raw);
    }

    data_rw_set_probe_stage(4U);
    memset(raw, 0, sizeof(raw));
    probe_status = get_device_info(DEV_INFO_GET_PN, raw);
    if (probe_status != IR_SUCCESS) {
        printf("[Camera] Probe PN failed: ir_status=%d\n", (int)probe_status);
        if (probe_status != IR_CHECK_DONE_FAIL) {
            return IR_NOT_GET_DEVICE_INFO;
        }
    } else {
        data_rw_mark_probe_ok(6U);
        copy_dev_info_str(s_dev_info.pn, sizeof(s_dev_info.pn), raw);
    }

    data_rw_set_probe_stage(5U);
    memset(raw, 0, sizeof(raw));
    probe_status = get_device_info(DEV_INFO_GET_SN, raw);
    if (probe_status != IR_SUCCESS) {
        printf("[Camera] Probe SN failed: ir_status=%d\n", (int)probe_status);
        if (probe_status != IR_CHECK_DONE_FAIL) {
            return IR_NOT_GET_DEVICE_INFO;
        }
    } else {
        data_rw_mark_probe_ok(7U);
        copy_dev_info_str(s_dev_info.sn, sizeof(s_dev_info.sn), raw);
    }

    s_dev_info.probed = true;
    printf("[Camera] Device Info Probed:\n");
    printf("         + Project  : %s\n", s_dev_info.project_info);
    printf("         + FW Build : %s (%s)\n", s_dev_info.fw_build_version, s_dev_info.fw_compile_date);
    printf("         + PN / SN  : %s / %s\n", s_dev_info.pn, s_dev_info.sn);

    if (out_info != NULL) {
        *out_info = s_dev_info;
    }
    return IR_SUCCESS;
}

const tiny1c_device_info_t *core_get_device_info(void)
{
    return &s_dev_info;
}

ir_error_t core_init(uint8_t i2c_bus, uint8_t slave_addr)
{
    data_rw_reset_i2c_diag();
    data_rw_set_i2c_bus(i2c_bus);
    data_rw_set_slave_addr(slave_addr);

    /* Khởi tạo Bus I2C thông qua Tầng 2 (soft_i2c_init) và Tầng 0 (bb_gpio_platform_init) */
    ir_error_t ret = data_rw_i2c_hw_init(i2c_bus);
    if (ret != IR_SUCCESS) {
        printf("[Camera] Error: data_rw_i2c_hw_init failed (%d)\n", ret);
        return ret;
    }

    /* Đăng ký con trỏ hàm giao tiếp I2C và khởi tạo Mutex thiết bị Tầng 3 cho VDCMD */
    ret = vdcmd_init_by_type(VDCMD_I2C_VDCMD);
    if (ret != IR_SUCCESS) {
        printf("[Camera] Error: vdcmd_init_by_type failed (%d)\n", ret);
        return ret;
    }

    printf("[Camera] Initialized on I2C bus %d, slave 0x%x (7-bit 0x%x)\n",
           (int)i2c_bus,
           (int)slave_addr,
           (int)(((slave_addr == I2C_SLAVE_ID) || (slave_addr > 0x7FU))
                     ? (slave_addr >> 1U)
                     : slave_addr));

    /* Truy vấn thông tin thiết bị & phiên bản Firmware từ ASIC-2121W */
    return core_probe_device_info(NULL);
}

ir_error_t core_start(camera_mode_t mode, uint8_t fps)
{
    if (mode < MODE_IMAGE || mode > MODE_DUAL || fps == 0 || fps > 25)
        return IR_ERROR_PARAM;
    if (!s_dev_info.probed) return IR_NOT_GET_DEVICE_INFO;
    ir_error_t ret = core_stop();
    if (ret != IR_SUCCESS) return ret;
    uint32_t hz = mode == MODE_DUAL ? THERMAL_SPI_DUAL_SCK_HZ : THERMAL_SPI_DEFAULT_SCK_HZ;
    if (hal_spi_set_freq_hz(hz) != 0) return IR_START_STREAMING_FAIL;
    PreviewStartParam_t cfg = {0};
    cfg.path = PREVIEW_PATH0; cfg.source = 0; cfg.width = IMG_W;
    cfg.height = mode == MODE_DUAL ? DUAL_HEIGHT : IMG_H;
    cfg.fps = fps; cfg.mode = 8;
    data_rw_set_probe_stage(6U);
    ret = preview_start(&cfg);
    if (ret != IR_SUCCESS) return ret;
    if (mode == MODE_TEMP) {
        data_rw_set_probe_stage(7U);
        ret = y16_preview_start(PREVIEW_PATH0, Y16_MODE_TEMPERATURE);
        if (ret != IR_SUCCESS) {
            (void)preview_stop(PREVIEW_PATH0);
            return ret;
        }
    }
    s_current_mode = mode;
    s_is_started = true;
    return IR_SUCCESS;
}

ir_error_t core_stop(void)
{
    if (!s_is_started) return IR_SUCCESS;
    ir_error_t y16 = IR_SUCCESS;
    if (s_current_mode == MODE_TEMP) y16 = y16_preview_stop(PREVIEW_PATH0);
    ir_error_t ret = preview_stop(PREVIEW_PATH0);
    /* Stop succeeded even if disabling Y16 failed; callers still see the error. */
    if (ret == IR_SUCCESS) s_is_started = false;
    return ret != IR_SUCCESS ? ret : y16;
}

camera_mode_t core_get_mode(void)
{
    return s_current_mode;
}

ir_error_t trigger_shutter(void)
{
    printf("[Camera] Triggering shutter calibration (OOC B-Update)...\n");
    ir_error_t ret = ooc_b_update(B_UPDATE);
    if (ret != IR_SUCCESS) {
        printf("[Camera] Shutter trigger failed (%d)\n", ret);
    }
    return ret;
}

ir_error_t core_set_auto_shutter_param(enum prop_auto_shutter_params param, uint16_t value)
{
    return set_prop_auto_shutter_params(param, value);
}

ir_error_t core_get_auto_shutter_param(enum prop_auto_shutter_params param, uint16_t *value)
{
    if (value == NULL) {
        return IR_ERROR_PARAM;
    }
    return get_prop_auto_shutter_params(param, value);
}

ir_error_t core_set_fall_protection(bool enable, uint16_t high_gain_thr, uint16_t low_gain_thr)
{
    ir_error_t ret = set_prop_auto_shutter_params(SHUTTER_PROP_PROTECT_SWITCH, enable ? 1U : 0U);
    if (ret != IR_SUCCESS) {
        return ret;
    }
    if (enable) {
        ret = set_prop_auto_shutter_params(SHUTTER_PROTECT_THR_HIGH_GAIN, high_gain_thr);
        if (ret != IR_SUCCESS) {
            return ret;
        }
        ret = set_prop_auto_shutter_params(SHUTTER_PROTECT_THR_LOW_GAIN, low_gain_thr);
    }
    return ret;
}

/* =========================================================================
 * 2. TRUY VẤN ĐO NHIỆT ĐỘ BỨC XẠ (TPD - TEMPERATURE MEASUREMENT)
 * ========================================================================= */

ir_error_t query_extremes(extremes_t *extremes)
{
    if (extremes == NULL) {
        return IR_ERROR_PARAM;
    }

    MaxMinTempInfo_t info = { 0 };
    ir_error_t ret = tpd_get_max_min_temp_info(&info);
    if (ret != IR_SUCCESS) {
        return ret;
    }

    extremes->max_temp = tpd_raw_to_celsius(info.max_temp);
    extremes->min_temp = tpd_raw_to_celsius(info.min_temp);
    extremes->max_x    = from_asic_coord(info.max_temp_point.x, IMG_W);
    extremes->max_y    = from_asic_coord(info.max_temp_point.y, IMG_H);
    extremes->min_x    = from_asic_coord(info.min_temp_point.x, IMG_W);
    extremes->min_y    = from_asic_coord(info.min_temp_point.y, IMG_H);

    return IR_SUCCESS;
}

ir_error_t query_point_temp(uint16_t x, uint16_t y, float *temp_celsius)
{
    if ((temp_celsius == NULL) || (x >= IMG_W) || (y >= IMG_H)) {
        return IR_ERROR_PARAM;
    }

    IrPoint_t pt = {
        .x = to_asic_coord_x(x),
        .y = to_asic_coord_y(y)
    };
    uint16_t raw_temp = 0U;
    ir_error_t ret = tpd_get_point_temp_info(pt, &raw_temp);
    if (ret != IR_SUCCESS) {
        return ret;
    }

    *temp_celsius = tpd_raw_to_celsius(raw_temp);
    return IR_SUCCESS;
}

ir_error_t query_line_temp(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, region_temp_t *out_info)
{
    if ((out_info == NULL) || (x0 >= IMG_W) || (y0 >= IMG_H) || (x1 >= IMG_W) || (y1 >= IMG_H)) {
        return IR_ERROR_PARAM;
    }

    IrLine_t line = {
        .start_point = { .x = to_asic_coord_x(x0), .y = to_asic_coord_y(y0) },
        .end_point   = { .x = to_asic_coord_x(x1), .y = to_asic_coord_y(y1) }
    };
    TpdLineRectTempInfo_t raw_info = { 0 };
    ir_error_t ret = tpd_get_line_temp_info(line, &raw_info);
    if (ret != IR_SUCCESS) {
        return ret;
    }

    out_info->avg_temp = tpd_raw_to_celsius(raw_info.temp_info_value.ave_temp);
    out_info->max_temp = tpd_raw_to_celsius(raw_info.temp_info_value.max_temp);
    out_info->min_temp = tpd_raw_to_celsius(raw_info.temp_info_value.min_temp);
    out_info->max_x    = from_asic_coord(raw_info.max_temp_point.x, IMG_W);
    out_info->max_y    = from_asic_coord(raw_info.max_temp_point.y, IMG_H);
    out_info->min_x    = from_asic_coord(raw_info.min_temp_point.x, IMG_W);
    out_info->min_y    = from_asic_coord(raw_info.min_temp_point.y, IMG_H);
    return IR_SUCCESS;
}

ir_error_t query_rect_temp(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, region_temp_t *out_info)
{
    if ((out_info == NULL) || (x0 >= IMG_W) || (y0 >= IMG_H) || (x1 >= IMG_W) || (y1 >= IMG_H)) {
        return IR_ERROR_PARAM;
    }

    IrRect_t rect = {
        .start_point = { .x = to_asic_coord_x(x0), .y = to_asic_coord_y(y0) },
        .end_point   = { .x = to_asic_coord_x(x1), .y = to_asic_coord_y(y1) }
    };
    TpdLineRectTempInfo_t raw_info = { 0 };
    ir_error_t ret = tpd_get_rect_temp_info(rect, &raw_info);
    if (ret != IR_SUCCESS) {
        return ret;
    }

    out_info->avg_temp = tpd_raw_to_celsius(raw_info.temp_info_value.ave_temp);
    out_info->max_temp = tpd_raw_to_celsius(raw_info.temp_info_value.max_temp);
    out_info->min_temp = tpd_raw_to_celsius(raw_info.temp_info_value.min_temp);
    out_info->max_x    = from_asic_coord(raw_info.max_temp_point.x, IMG_W);
    out_info->max_y    = from_asic_coord(raw_info.max_temp_point.y, IMG_H);
    out_info->min_x    = from_asic_coord(raw_info.min_temp_point.x, IMG_W);
    out_info->min_y    = from_asic_coord(raw_info.min_temp_point.y, IMG_H);
    return IR_SUCCESS;
}

/* =========================================================================
 * 3. TÍNH NĂNG 5: XỬ LÝ HÌNH ẢNH NÂNG CAO & ISP (IMAGE ENHANCEMENT)
 * ========================================================================= */

ir_error_t set_palette(uint8_t palette_id)
{
    /* Chặn PSEUDO_COLOR_MODE_2 (Reserved trên Tiny1C theo tài liệu hãng) */
    if ((palette_id < PSEUDO_COLOR_MODE_1) ||
        (palette_id > PSEUDO_COLOR_MODE_11) ||
        (palette_id == PSEUDO_COLOR_MODE_2)) {
        return IR_ERROR_PARAM;
    }
    return pseudo_color_set(PREVIEW_PATH0, (enum pseudo_color_types)palette_id);
}

ir_error_t get_palette(uint8_t *palette_id)
{
    if (palette_id == NULL) {
        return IR_ERROR_PARAM;
    }
    return pseudo_color_get(PREVIEW_PATH0, palette_id);
}

ir_error_t core_zoom_center_up(enum zoom_scale_step step)
{
    return zoom_center_up(PREVIEW_PATH0, step);
}

ir_error_t core_zoom_center_down(enum zoom_scale_step step)
{
    return zoom_center_down(PREVIEW_PATH0, step);
}

ir_error_t core_zoom_position_up(enum zoom_scale_step step, uint16_t x, uint16_t y)
{
    if ((x >= IMG_W) || (y >= IMG_H)) {
        return IR_ERROR_PARAM;
    }
    IrPoint_t pos = { .x = to_asic_coord_x(x), .y = to_asic_coord_y(y) };
    return zoom_position_up(PREVIEW_PATH0, step, pos);
}

ir_error_t core_zoom_position_down(enum zoom_scale_step step, uint16_t x, uint16_t y)
{
    if ((x >= IMG_W) || (y >= IMG_H)) {
        return IR_ERROR_PARAM;
    }
    IrPoint_t pos = { .x = to_asic_coord_x(x), .y = to_asic_coord_y(y) };
    return zoom_position_down(PREVIEW_PATH0, step, pos);
}

ir_error_t core_set_image_param(enum prop_image_params param, uint16_t value)
{
    return set_prop_image_params(param, value);
}

ir_error_t core_get_image_param(enum prop_image_params param, uint16_t *value)
{
    if (value == NULL) {
        return IR_ERROR_PARAM;
    }
    return get_prop_image_params(param, value);
}

ir_error_t core_set_mirror_flip(enum mirror_flip_types mirror_flip)
{
    if (mirror_flip > MIRROR_FLIP) {
        return IR_ERROR_PARAM;
    }
    return set_prop_image_params(IMAGE_PROP_SEL_MIRROR_FLIP, (uint16_t)mirror_flip);
}

/* =========================================================================
 * 4. TÍNH NĂNG 6: CĂN CHỈNH & HIỆU CHUẨN THIẾT BỊ (CALIBRATION)
 * ========================================================================= */

ir_error_t core_dpc_auto_calibrate(uint16_t wait_time_sec)
{
    if (wait_time_sec == 0U) {
        wait_time_sec = 5U;
    }
    return dpc_auto_calibration(wait_time_sec);
}

ir_error_t core_dpc_add_point(uint16_t x, uint16_t y)
{
    if ((x >= IMG_W) || (y >= IMG_H)) {
        return IR_ERROR_PARAM;
    }
    IrPoint_t pt = { .x = to_asic_coord_x(x), .y = to_asic_coord_y(y) };
    return dpc_add_point(pt);
}

ir_error_t core_dpc_remove_point(uint16_t x, uint16_t y)
{
    if ((x >= IMG_W) || (y >= IMG_H)) {
        return IR_ERROR_PARAM;
    }
    IrPoint_t pt = { .x = to_asic_coord_x(x), .y = to_asic_coord_y(y) };
    return dpc_remove_point(pt);
}

ir_error_t core_dpc_save_config(void)
{
    return spi_config_save(SPI_MOD_CFG_DEAD_PIX);
}

ir_error_t core_rmcover_auto_calc(uint8_t zoom_scale)
{
    if ((zoom_scale != 1U) && (zoom_scale != 2U) && (zoom_scale != 4U)) {
        zoom_scale = 1U;
    }
    return rmcover_auto_calc(zoom_scale);
}

ir_error_t core_rmcover_enable(bool enable)
{
    return rmcover_sts_switch(enable ? RMCOVER_EN : RMCOVER_DIS);
}

ir_error_t core_recal_1point_kelvin(uint16_t target_temp_k)
{
    return tpd_ktbt_recal_1point(target_temp_k);
}

ir_error_t core_recal_1point_celsius(float target_temp_c)
{
    uint16_t temp_k = (uint16_t)(target_temp_c + 273.15f + 0.5f);
    return core_recal_1point_kelvin(temp_k);
}

ir_error_t core_recal_2point_kelvin(enum tpd_ktbt_recal_point_idx point_idx, uint16_t target_temp_k)
{
    return tpd_ktbt_recal_2point(point_idx, target_temp_k);
}

ir_error_t core_recal_2point_celsius(enum tpd_ktbt_recal_point_idx point_idx, float target_temp_c)
{
    uint16_t temp_k = (uint16_t)(target_temp_c + 273.15f + 0.5f);
    return core_recal_2point_kelvin(point_idx, temp_k);
}

ir_error_t core_set_tpd_param(enum prop_tpd_params param, uint16_t value)
{
    return set_prop_tpd_params(param, value);
}

ir_error_t core_get_tpd_param(enum prop_tpd_params param, uint16_t *value)
{
    if (value == NULL) {
        return IR_ERROR_PARAM;
    }
    return get_prop_tpd_params(param, value);
}

ir_error_t core_set_env_params(float emissivity, float distance_m,
                               float refl_temp_c, float atm_temp_c,
                               float transmittance)
{
    if ((emissivity <= 0.0f) || (emissivity > 1.0f) ||
        (transmittance <= 0.0f) || (transmittance > 1.0f) ||
        (distance_m < 0.0f) || (distance_m > 200.0f)) {
        return IR_ERROR_PARAM;
    }

    uint16_t ems_cnt  = (uint16_t)(emissivity * 128.0f + 0.5f);
    uint16_t tau_cnt  = (uint16_t)(transmittance * 128.0f + 0.5f);
    uint16_t tu_k     = (uint16_t)(refl_temp_c + 273.15f + 0.5f);
    uint16_t ta_k     = (uint16_t)(atm_temp_c + 273.15f + 0.5f);
    (void)distance_m; /* Theo Interface Description1.2.xlsx, Tiny1C chỉ hỗ trợ EMS, TAU, TU, TA, GAIN_SEL */

    if (ems_cnt == 0U) ems_cnt = 1U;
    if (tau_cnt == 0U) tau_cnt = 1U;

    ir_error_t ret = set_prop_tpd_params(TPD_PROP_EMS, ems_cnt);
    if (ret != IR_SUCCESS) return ret;

    ret = set_prop_tpd_params(TPD_PROP_TAU, tau_cnt);
    if (ret != IR_SUCCESS) return ret;

    ret = set_prop_tpd_params(TPD_PROP_TU, tu_k);
    if (ret != IR_SUCCESS) return ret;

    return set_prop_tpd_params(TPD_PROP_TA, ta_k);
}

ir_error_t core_set_gain_mode(bool high_gain)
{
    return set_prop_tpd_params(TPD_PROP_GAIN_SEL, high_gain ? 1U : 0U);
}

ir_error_t core_save_config(enum prop_spi_cfg_module module)
{
    return spi_config_save(module);
}

ir_error_t core_load_config(enum prop_spi_cfg_module module)
{
    return spi_config_load(module);
}

ir_error_t core_restore_default_cfg(enum prop_default_cfg cfg_type)
{
    return restore_default_cfg(cfg_type);
}
