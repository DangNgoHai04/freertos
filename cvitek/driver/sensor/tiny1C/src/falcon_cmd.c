/*
 * Tầng 3 (T3 - Tiny1C Falcon ASIC-2121W Command Implementation): Triển khai toàn bộ
 * tập lệnh điều khiển ISP, TPD, DPC, Zoom và SPI Flash nội bộ của ASIC-2121W.
 */

#include "falcon_cmd.h"
#include <stdio.h>
#include <stdint.h>
#include "vdcmd.h"

#if defined(FALCON_CMD)

typedef struct {
    uint8_t bySdpcDeadPixEn;
    uint8_t bySdpRpEn;
    uint8_t bySdpcDeadPixType;
    uint8_t bySdpcInterpolationTh;
    uint8_t bySdpcMedianFilterEn;
} SdpcParma_t;

typedef struct {
    uint8_t byBpEn;
    uint8_t byBpOptEn;
    int16_t swBpMaxTshd;
    int16_t swBpMinTshd;
    int16_t swBpMaxOptTshd;
    int16_t swBpMinOptTshd;
    int8_t  sbyBpRpMaxOffset;
    int8_t  sbyBpRpMinOffset;
    int8_t  sbyBpRpMaxOptOffset;
    int8_t  sbyBpRpMinOptOffset;
} DdpcParma_t;

enum isp_param_module {
    CDC             = 0,
    HBC             = 1,
    SDPC            = 2,
    VBC             = 3,
    TNR             = 4,
    DUMMY           = 5,
    RMVCOVER        = 6,
    TPD             = 7,
    SNR_GAUSS       = 8,
    SNR_BP          = 9,
    DDPC            = 10,
    FOCUS           = 11,
    AGC             = 12,
    DDE             = 13,
    GAMMA           = 14,
    MIRROR_AND_FLIP = 15,
    KBC             = 16,
    OOC             = 17
};

ir_error_t vdcmd_init_by_type(enum vdcmd_driver_type type)
{
    return vdcmd_rw_handle_register(FALCON_PRODUCT, (enum driver_types)type);
}

ir_error_t restore_default_cfg(enum prop_default_cfg default_cfg_type)
{
    vdcmd_std_header_t ptVdCmdHeader = { 0 };
    ptVdCmdHeader.byCmdType = CMDTYPE_STANDARD_TYPE_SPI;
    ptVdCmdHeader.bySubCmd  = SUBCMD_SPI_DEFAULT_CFG_RESTORE;
    ptVdCmdHeader.byPara    = (uint8_t)default_cfg_type;
    return standard_cmd_write(&ptVdCmdHeader, 0U, NULL);
}

ir_error_t load_prop_default_params(enum prop_default_params default_param)
{
    vdcmd_long_header_t ptVdCmdHeader = { 0 };
    ptVdCmdHeader.byCmdType = CMDTYPE_LONG_TYPE_PROP_PAGE;
    ptVdCmdHeader.bySubCmd  = SUBCMD_PROP_LOAD_DEFAULT_PARAM;
    return long_cmd_write(&ptVdCmdHeader, (uint16_t)default_param, 0U, 0U, 0U);
}

ir_error_t set_prop_auto_shutter_params(enum prop_auto_shutter_params shutter_param, uint16_t value)
{
    vdcmd_long_header_t ptVdCmdHeader = { 0 };
    ptVdCmdHeader.byCmdType = CMDTYPE_LONG_TYPE_PROP_PAGE;
    ptVdCmdHeader.bySubCmd  = SUBCMD_PROP_AUTO_SHUTTER_PARAM_SET;

    if ((shutter_param == SHUTTER_PROP_MIN_INTERVAL) &&
        ((value < SHUTTER_PROP_MIN_INTERVAL_MIN) || (value > SHUTTER_PROP_MIN_INTERVAL_MAX))) {
        return IR_ERROR_PARAM;
    }
    if ((shutter_param == SHUTTER_PROP_MAX_INTERVAL) &&
        ((value < SHUTTER_PROP_MAX_INTERVAL_MIN) || (value > SHUTTER_PROP_MAX_INTERVAL_MAX))) {
        return IR_ERROR_PARAM;
    }
    if ((shutter_param == SHUTTER_PROP_ANY_INTERVAL) && (value > SHUTTER_PROP_ANY_INTERVAL_MAX)) {
        return IR_ERROR_PARAM;
    }
    if ((shutter_param == SHUTTER_PREVIEW_START_1ST_DELAY) && (value > SHUTTER_PROP_PREVIEW_START_1ST_DELAY_MAX)) {
        return IR_ERROR_PARAM;
    }
    if ((shutter_param == SHUTTER_PREVIEW_START_2ND_DELAY) && (value > SHUTTER_PROP_PREVIEW_START_2ND_DELAY_MAX)) {
        return IR_ERROR_PARAM;
    }
    if ((shutter_param == SHUTTER_CHANGE_GAIN_1ST_DELAY) && (value > SHUTTER_PROP_CHANGE_GAIN_1ST_DELAY_MAX)) {
        return IR_ERROR_PARAM;
    }
    if ((shutter_param == SHUTTER_CHANGE_GAIN_2ND_DELAY) && (value > SHUTTER_PROP_CHANGE_GAIN_2ND_DELAY_MAX)) {
        return IR_ERROR_PARAM;
    }

    return long_cmd_write(&ptVdCmdHeader, (uint16_t)shutter_param, (uint32_t)value, 0U, 0U);
}

ir_error_t get_prop_auto_shutter_params(enum prop_auto_shutter_params shutter_param, uint16_t *value)
{
    if (value == NULL) {
        return IR_ERROR_PARAM;
    }
    uint8_t data[2] = { 0 };
    vdcmd_long_header_t ptVdCmdHeader = { 0 };
    ptVdCmdHeader.byCmdType = CMDTYPE_LONG_TYPE_PROP_PAGE;
    ptVdCmdHeader.bySubCmd  = SUBCMD_PROP_AUTO_SHUTTER_PARAM_GET;

    ir_error_t rst = long_cmd_read(&ptVdCmdHeader, (uint16_t)shutter_param, 0U, 0U, 2U, data);
    *value = (uint16_t)(((uint16_t)data[0] << 8U) | data[1]);
    return rst;
}

ir_error_t set_prop_image_params(enum prop_image_params image_param, uint16_t value)
{
    vdcmd_long_header_t ptVdCmdHeader = { 0 };
    ptVdCmdHeader.byCmdType = CMDTYPE_LONG_TYPE_PROP_PAGE;
    ptVdCmdHeader.bySubCmd  = SUBCMD_PROP_IMAGE_PARAM_SET;

    if ((image_param == IMAGE_PROP_LEVEL_TNR) && (value > IMG_PROP_TNR_MAX)) return IR_ERROR_PARAM;
    if ((image_param == IMAGE_PROP_LEVEL_SNR) && (value > IMG_PROP_SNR_MAX)) return IR_ERROR_PARAM;
    if ((image_param == IMAGE_PROP_LEVEL_DDE) && (value > IMG_PROP_DDE_MAX)) return IR_ERROR_PARAM;
    if ((image_param == IMAGE_PROP_LEVEL_BRIGHTNESS) && (value > IMG_PROP_BRIGHTNESS_MAX)) return IR_ERROR_PARAM;
    if ((image_param == IMAGE_PROP_LEVEL_CONTRAST) && (value > IMG_PROP_CONTRAST_MAX)) return IR_ERROR_PARAM;
    if ((image_param == IMAGE_PROP_MODE_AGC) && (value > IMG_PROP_AGC_MAX)) return IR_ERROR_PARAM;
    if ((image_param == IMAGE_PROP_LEVEL_MAX_GAIN) && (value > IMG_PROP_AGC_MAXGAIN_MAX)) return IR_ERROR_PARAM;
    if ((image_param == IMAGE_PROP_LEVEL_BOS) && (value > IMG_PROP_AGC_BOS_MAX)) return IR_ERROR_PARAM;
    if ((image_param == IMAGE_PROP_SEL_MIRROR_FLIP) && (value > IMG_PROP_MIRROR_FLIP_MAX)) return IR_ERROR_PARAM;

    return long_cmd_write(&ptVdCmdHeader, (uint16_t)image_param, (uint32_t)value, 0U, 0U);
}

ir_error_t get_prop_image_params(enum prop_image_params image_param, uint16_t *value)
{
    if (value == NULL) {
        return IR_ERROR_PARAM;
    }
    uint8_t data[2] = { 0 };
    vdcmd_long_header_t ptVdCmdHeader = { 0 };
    ptVdCmdHeader.byCmdType = CMDTYPE_LONG_TYPE_PROP_PAGE;
    ptVdCmdHeader.bySubCmd  = SUBCMD_PROP_IMAGE_PARAM_GET;

    ir_error_t rst = long_cmd_read(&ptVdCmdHeader, (uint16_t)image_param, 0U, 0U, 2U, data);
    *value = (uint16_t)(((uint16_t)data[0] << 8U) | data[1]);
    return rst;
}

ir_error_t set_prop_tpd_params(enum prop_tpd_params tpd_param, uint16_t value)
{
    vdcmd_long_header_t ptVdCmdHeader = { 0 };
    ptVdCmdHeader.byCmdType = CMDTYPE_LONG_TYPE_PROP_PAGE;
    ptVdCmdHeader.bySubCmd  = SUBCMD_PROP_TPD_PARAM_SET;

    if ((tpd_param == TPD_PROP_DISTANCE) && (value > TPD_PARAM_DIST_MAX)) return IR_ERROR_PARAM;
    if ((tpd_param == TPD_PROP_TU) && ((value < TPD_PARAM_TU_MIN) || (value > TPD_PARAM_TU_MAX))) return IR_ERROR_PARAM;
    if ((tpd_param == TPD_PROP_TA) && ((value < TPD_PARAM_TA_MIN) || (value > TPD_PARAM_TA_MAX))) return IR_ERROR_PARAM;
    if ((tpd_param == TPD_PROP_EMS) && ((value < TPD_PARAM_EMS_MIN) || (value > TPD_PARAM_EMS_MAX))) return IR_ERROR_PARAM;
    if ((tpd_param == TPD_PROP_TAU) && ((value < TPD_PARAM_TAU_MIN) || (value > TPD_PARAM_TAU_MAX))) return IR_ERROR_PARAM;

    return long_cmd_write(&ptVdCmdHeader, (uint16_t)tpd_param, (uint32_t)value, 0U, 0U);
}

ir_error_t get_prop_tpd_params(enum prop_tpd_params tpd_param, uint16_t *value)
{
    if (value == NULL) {
        return IR_ERROR_PARAM;
    }
    if (tpd_param == TPD_PROP_DIST_COR_FACT) {
        *value = 0U;
        return IR_ERROR_PARAM;
    }
    if ((tpd_param == TPD_PROP_ENV_CORRECT_SEL) ||
        (tpd_param == TPD_PROP_DIST_CORRECT_SEL) ||
        (tpd_param == TPD_PROP_TEMP_MAP_SEL)) {
        *value = 1U;
        return IR_ERROR_PARAM;
    }

    uint8_t data[2] = { 0 };
    vdcmd_long_header_t ptVdCmdHeader = { 0 };
    ptVdCmdHeader.byCmdType = CMDTYPE_LONG_TYPE_PROP_PAGE;
    ptVdCmdHeader.bySubCmd  = SUBCMD_PROP_TPD_PARAM_GET;

    ir_error_t rst = long_cmd_read(&ptVdCmdHeader, (uint16_t)tpd_param, 0U, 0U, 2U, data);
    *value = (uint16_t)(((uint16_t)data[0] << 8U) | data[1]);
    return rst;
}

ir_error_t get_device_info(enum device_id_types id_type, uint8_t *id_content)
{
    if (id_content == NULL) {
        return IR_ERROR_PARAM;
    }
    vdcmd_std_header_t ptVdCmdHeader = { 0 };
    ptVdCmdHeader.byCmdType = CMDTYPE_STANDARD_TYPE_SYS;
    ptVdCmdHeader.bySubCmd  = SUBCMD_SYS_DEV_INFO_GET;
    ptVdCmdHeader.byPara    = (uint8_t)id_type;
    return standard_cmd_read(&ptVdCmdHeader, 64U, id_content);
}

ir_error_t y16_preview_start(enum preview_path path, enum y16_isp_stream_src_types src_type)
{
    vdcmd_std_header_t ptVdCmdHeader = { 0 };
    ptVdCmdHeader.byCmdType = CMDTYPE_STANDARD_TYPE_Y16_MODE;
    ptVdCmdHeader.bySubCmd  = SUBCMD_Y16_MODE_PREVIEW_START;
    ptVdCmdHeader.byPara    = (uint8_t)path;
    ptVdCmdHeader.byAddr_ll = (uint8_t)src_type;
    return standard_cmd_write(&ptVdCmdHeader, 0U, NULL);
}

ir_error_t y16_preview_stop(enum preview_path path)
{
    vdcmd_std_header_t ptVdCmdHeader = { 0 };
    ptVdCmdHeader.byCmdType = CMDTYPE_STANDARD_TYPE_Y16_MODE;
    ptVdCmdHeader.bySubCmd  = SUBCMD_Y16_MODE_PREVIEW_STOP;
    ptVdCmdHeader.byPara    = (uint8_t)path;
    return standard_cmd_write(&ptVdCmdHeader, 0U, NULL);
}

ir_error_t preview_start(PreviewStartParam_t *preview_start_param)
{
    if (preview_start_param == NULL) {
        return IR_ERROR_PARAM;
    }
    uint8_t data[8] = { 0 };
    vdcmd_std_header_t ptVdCmdHeader = { 0 };
    ptVdCmdHeader.byCmdType = CMDTYPE_STANDARD_TYPE_PREVIEW;
    ptVdCmdHeader.bySubCmd  = SUBCMD_PREVIEW_START;

    data[0] = preview_start_param->path;
    data[1] = (preview_start_param->source == 1U) ? 0x80U : 0x00U;
    data[2] = (uint8_t)(preview_start_param->width >> 8U);
    data[3] = (uint8_t)(preview_start_param->width & 0xFFU);
    data[4] = (uint8_t)(preview_start_param->height >> 8U);
    data[5] = (uint8_t)(preview_start_param->height & 0xFFU);
    data[6] = preview_start_param->fps;
    data[7] = preview_start_param->mode;

    return standard_cmd_write(&ptVdCmdHeader, 8U, data);
}

ir_error_t preview_stop(enum preview_path path)
{
    vdcmd_std_header_t ptVdCmdHeader = { 0 };
    ptVdCmdHeader.byCmdType = CMDTYPE_STANDARD_TYPE_PREVIEW;
    ptVdCmdHeader.bySubCmd  = SUBCMD_PREVIEW_STOP;
    ptVdCmdHeader.byPara    = (uint8_t)path;
    return standard_cmd_write(&ptVdCmdHeader, 0U, NULL);
}

ir_error_t ooc_b_update(enum ooc_b_update_types update_type)
{
    vdcmd_std_header_t ptVdCmdHeader = { 0 };
    ptVdCmdHeader.byCmdType = CMDTYPE_STANDARD_TYPE_FW_ISP;
    ptVdCmdHeader.bySubCmd  = SUBCMD_FW_ISP_OOC_B_UPDATE;
    ptVdCmdHeader.byPara    = (uint8_t)update_type;
    return standard_cmd_write(&ptVdCmdHeader, 0U, NULL);
}

ir_error_t shutter_sta_get(uint8_t *shutter_en_sta, uint8_t *shutter_sta)
{
    if ((shutter_en_sta == NULL) || (shutter_sta == NULL)) {
        return IR_ERROR_PARAM;
    }
    uint8_t data[2] = { 0 };
    vdcmd_std_header_t ptVdCmdHeader = { 0 };
    ptVdCmdHeader.byCmdType = CMDTYPE_STANDARD_TYPE_SHUTTER;
    ptVdCmdHeader.bySubCmd  = SUBCMD_SHUTTER_STA_GET;

    ir_error_t rst = standard_cmd_read(&ptVdCmdHeader, 2U, data);
    *shutter_en_sta = data[0];
    *shutter_sta    = data[1];
    return rst;
}

ir_error_t shutter_sta_set(enum shutter_sta_types sta_type)
{
    vdcmd_std_header_t ptVdCmdHeader = { 0 };
    ptVdCmdHeader.byCmdType = CMDTYPE_STANDARD_TYPE_SHUTTER;
    ptVdCmdHeader.bySubCmd  = SUBCMD_SHUTTER_STA_SET;
    ptVdCmdHeader.byPara    = (uint8_t)sta_type;
    return standard_cmd_write(&ptVdCmdHeader, 0U, NULL);
}

ir_error_t shutter_manual_switch(enum shutter_manual_types manual_type)
{
    vdcmd_std_header_t ptVdCmdHeader = { 0 };
    ptVdCmdHeader.byCmdType = CMDTYPE_STANDARD_TYPE_SHUTTER;
    ptVdCmdHeader.bySubCmd  = SUBCMD_SHUTTER_SWITCH;
    ptVdCmdHeader.byPara    = (uint8_t)manual_type;
    return standard_cmd_write(&ptVdCmdHeader, 0U, NULL);
}

ir_error_t shutter_vtemp_get(uint16_t *cur_vtemp)
{
    if (cur_vtemp == NULL) {
        return IR_ERROR_PARAM;
    }
    uint8_t data[2] = { 0 };
    vdcmd_std_header_t ptVdCmdHeader = { 0 };
    ptVdCmdHeader.byCmdType = CMDTYPE_STANDARD_TYPE_SHUTTER;
    ptVdCmdHeader.bySubCmd  = SUBCMD_SHUTTER_VTEMP_GET;

    ir_error_t rst = standard_cmd_read(&ptVdCmdHeader, 2U, data);
    *cur_vtemp = (uint16_t)(((uint16_t)data[0] << 8U) | data[1]);
    return rst;
}

ir_error_t cur_vtemp_get(uint16_t *cur_vtemp)
{
    if (cur_vtemp == NULL) {
        return IR_ERROR_PARAM;
    }
    uint8_t data[2] = { 0 };
    vdcmd_std_header_t ptVdCmdHeader = { 0 };
    ptVdCmdHeader.byCmdType = CMDTYPE_STANDARD_TYPE_FW_ISP;
    ptVdCmdHeader.bySubCmd  = SUBCMD_FW_ISP_GET_CUR_VTEMP;

    ir_error_t rst = standard_cmd_read(&ptVdCmdHeader, 2U, data);
    *cur_vtemp = (uint16_t)(((uint16_t)data[0] << 8U) | data[1]);
    return rst;
}

ir_error_t rmcover_sts_switch(enum rmcover_types en_sts)
{
    vdcmd_std_header_t ptVdCmdHeader = { 0 };
    ptVdCmdHeader.byCmdType = CMDTYPE_STANDARD_TYPE_FW_ISP;
    if (en_sts == RMCOVER_DIS) {
        ptVdCmdHeader.bySubCmd = SUBCMD_FW_ISP_RMCOVER_DISABLE;
    } else if (en_sts == RMCOVER_EN) {
        ptVdCmdHeader.bySubCmd = SUBCMD_FW_ISP_RMCOVER_ENABLE;
    } else {
        return IR_ERROR_PARAM;
    }
    return standard_cmd_write(&ptVdCmdHeader, 0U, NULL);
}

ir_error_t rmcover_auto_calc(uint8_t zoom_scale)
{
    if ((zoom_scale != 1U) && (zoom_scale != 2U) && (zoom_scale != 4U)) {
        return IR_ERROR_PARAM;
    }
    vdcmd_std_header_t ptVdCmdHeader = { 0 };
    ptVdCmdHeader.byCmdType = CMDTYPE_STANDARD_TYPE_FW_ISP;
    ptVdCmdHeader.bySubCmd  = SUBCMD_FW_ISP_AUTO_CAL_RMCOVER;
    ptVdCmdHeader.byPara    = zoom_scale;
    return standard_cmd_write(&ptVdCmdHeader, 0U, NULL);
}

ir_error_t pseudo_color_get(enum preview_path path, uint8_t *color_type)
{
    if (color_type == NULL) {
        return IR_ERROR_PARAM;
    }
    vdcmd_std_header_t ptVdCmdHeader = { 0 };
    ptVdCmdHeader.byCmdType = CMDTYPE_STANDARD_TYPE_ISP_PARAM;
    ptVdCmdHeader.bySubCmd  = SUBCMD_PSEUD_COLOR_GET;
    ptVdCmdHeader.byPara    = (uint8_t)path;
    return standard_cmd_read(&ptVdCmdHeader, 1U, color_type);
}

ir_error_t pseudo_color_set(enum preview_path path, enum pseudo_color_types color_type)
{
    /*
     * Theo tài liệu Interface Description1.2.xlsx của hãng cho Tiny1C:
     * Bảng màu hợp lệ là 1..11 NHƯNG giá trị 2 (PSEUDO_COLOR_MODE_2) là RESERVED
     * (không tồn tại trên Tiny1C). Chặn ngay tại Tầng 3 để tránh lỗi PARAM_ERR dưới ASIC.
     */
    if ((color_type < PSEUDO_COLOR_MODE_1) ||
        (color_type > PSEUDO_COLOR_MODE_11) ||
        (color_type == PSEUDO_COLOR_MODE_2)) {
        printf("[T3-FALCON] Rejecting invalid/reserved pseudo_color_type=%d\n", (int)color_type);
        return IR_ERROR_PARAM;
    }
    uint8_t data = (uint8_t)color_type;
    vdcmd_std_header_t ptVdCmdHeader = { 0 };
    ptVdCmdHeader.byCmdType = CMDTYPE_STANDARD_TYPE_ISP_PARAM;
    ptVdCmdHeader.bySubCmd  = SUBCMD_PSEUD_COLOR_SET;
    ptVdCmdHeader.byPara    = (uint8_t)path;
    return standard_cmd_write(&ptVdCmdHeader, 1U, &data);
}

ir_error_t tpd_ktbt_recal_1point(uint16_t temp)
{
    uint8_t data[2] = { (uint8_t)(temp >> 8U), (uint8_t)(temp & 0xFFU) };
    vdcmd_std_header_t ptVdCmdHeader = { 0 };
    ptVdCmdHeader.byCmdType = CMDTYPE_STANDARD_TYPE_TPD;
    ptVdCmdHeader.bySubCmd  = SUBCMD_TPD_KTBT_RECAL_1_POINT;
    return standard_cmd_write(&ptVdCmdHeader, 2U, data);
}

ir_error_t tpd_ktbt_recal_2point(enum tpd_ktbt_recal_point_idx point_idx, uint16_t temp)
{
    if ((point_idx != TPD_KTBT_RECAL_P1) && (point_idx != TPD_KTBT_RECAL_P2)) {
        return IR_ERROR_PARAM;
    }
    uint8_t data[2] = { (uint8_t)(temp >> 8U), (uint8_t)(temp & 0xFFU) };
    vdcmd_std_header_t ptVdCmdHeader = { 0 };
    ptVdCmdHeader.byCmdType = CMDTYPE_STANDARD_TYPE_TPD;
    ptVdCmdHeader.bySubCmd  = SUBCMD_TPD_KTBT_RECAL_2_POINT;
    ptVdCmdHeader.byPara    = (point_idx == TPD_KTBT_RECAL_P1) ? 0x00U : 0x01U;
    return standard_cmd_write(&ptVdCmdHeader, 2U, data);
}

ir_error_t tpd_get_max_temp(uint16_t *max_temp)
{
    if (max_temp == NULL) {
        return IR_ERROR_PARAM;
    }
    uint8_t data[2] = { 0 };
    vdcmd_long_header_t ptVdCmdHeader = { 0 };
    ptVdCmdHeader.byCmdType = CMDTYPE_LONG_TYPE_TPD;
    ptVdCmdHeader.bySubCmd  = SUBCMD_TPD_GET_MAX_TEMP;

    ir_error_t rst = long_cmd_read(&ptVdCmdHeader, 0U, 0U, 0U, 2U, data);
    *max_temp = (uint16_t)(((uint16_t)data[0] << 8U) | data[1]);
    return rst;
}

ir_error_t tpd_get_min_temp(uint16_t *min_temp)
{
    if (min_temp == NULL) {
        return IR_ERROR_PARAM;
    }
    uint8_t data[2] = { 0 };
    vdcmd_long_header_t ptVdCmdHeader = { 0 };
    ptVdCmdHeader.byCmdType = CMDTYPE_LONG_TYPE_TPD;
    ptVdCmdHeader.bySubCmd  = SUBCMD_TPD_GET_MIN_TEMP;

    ir_error_t rst = long_cmd_read(&ptVdCmdHeader, 0U, 0U, 0U, 2U, data);
    *min_temp = (uint16_t)(((uint16_t)data[0] << 8U) | data[1]);
    return rst;
}

ir_error_t tpd_get_max_min_temp_info(MaxMinTempInfo_t *max_min_temp_info)
{
    if (max_min_temp_info == NULL) {
        return IR_ERROR_PARAM;
    }
    uint8_t data[12] = { 0 };
    vdcmd_long_header_t ptVdCmdHeader = { 0 };
    ptVdCmdHeader.byCmdType = CMDTYPE_LONG_TYPE_TPD;
    ptVdCmdHeader.bySubCmd  = SUBCMD_TPD_GET_MAX_MIN_TEMP_INFO;

    ir_error_t rst = long_cmd_read(&ptVdCmdHeader, 0U, 0U, 0U, 12U, data);
    max_min_temp_info->max_temp         = (uint16_t)(((uint16_t)data[0] << 8U) | data[1]);
    max_min_temp_info->min_temp         = (uint16_t)(((uint16_t)data[2] << 8U) | data[3]);
    max_min_temp_info->max_temp_point.x = (uint16_t)(((uint16_t)data[4] << 8U) | data[5]);
    max_min_temp_info->max_temp_point.y = (uint16_t)(((uint16_t)data[6] << 8U) | data[7]);
    max_min_temp_info->min_temp_point.x = (uint16_t)(((uint16_t)data[8] << 8U) | data[9]);
    max_min_temp_info->min_temp_point.y = (uint16_t)(((uint16_t)data[10] << 8U) | data[11]);
    return rst;
}

ir_error_t tpd_get_point_temp_info(IrPoint_t point_pos, uint16_t *point_temp_value)
{
    if (point_temp_value == NULL) {
        return IR_ERROR_PARAM;
    }
    uint8_t data[12] = { 0 };
    vdcmd_long_header_t ptVdCmdHeader = { 0 };
    ptVdCmdHeader.byCmdType = CMDTYPE_LONG_TYPE_TPD;
    ptVdCmdHeader.bySubCmd  = SUBCMD_TPD_GET_POINT_TEMP;
    uint32_t dwAddr1 = (((uint32_t)point_pos.x) << 16U) | point_pos.y;

    ir_error_t rst = long_cmd_read(&ptVdCmdHeader, 0U, dwAddr1, 0U, 12U, data);
    *point_temp_value = (uint16_t)(((uint16_t)data[0] << 8U) | data[1]);
    return rst;
}

ir_error_t tpd_get_line_temp_info(IrLine_t line_pos, TpdLineRectTempInfo_t *line_temp_info)
{
    if (line_temp_info == NULL) {
        return IR_ERROR_PARAM;
    }
    uint8_t data[14] = { 0 };
    vdcmd_long_header_t ptVdCmdHeader = { 0 };
    ptVdCmdHeader.byCmdType = CMDTYPE_LONG_TYPE_TPD;
    ptVdCmdHeader.bySubCmd  = SUBCMD_TPD_GET_LINE_TEMP;
    uint32_t dwAddr1 = (((uint32_t)line_pos.start_point.x) << 16U) | line_pos.start_point.y;
    uint32_t dwAddr2 = (((uint32_t)line_pos.end_point.x) << 16U) | line_pos.end_point.y;

    ir_error_t rst = long_cmd_read(&ptVdCmdHeader, 0U, dwAddr1, dwAddr2, 14U, data);
    line_temp_info->temp_info_value.ave_temp = (uint16_t)(((uint16_t)data[0] << 8U) | data[1]);
    line_temp_info->temp_info_value.max_temp = (uint16_t)(((uint16_t)data[2] << 8U) | data[3]);
    line_temp_info->temp_info_value.min_temp = (uint16_t)(((uint16_t)data[4] << 8U) | data[5]);
    line_temp_info->max_temp_point.x         = (uint16_t)(((uint16_t)data[6] << 8U) | data[7]);
    line_temp_info->max_temp_point.y         = (uint16_t)(((uint16_t)data[8] << 8U) | data[9]);
    line_temp_info->min_temp_point.x         = (uint16_t)(((uint16_t)data[10] << 8U) | data[11]);
    line_temp_info->min_temp_point.y         = (uint16_t)(((uint16_t)data[12] << 8U) | data[13]);
    return rst;
}

ir_error_t tpd_get_rect_temp_info(IrRect_t rect_pos, TpdLineRectTempInfo_t *rect_temp_info)
{
    if (rect_temp_info == NULL) {
        return IR_ERROR_PARAM;
    }
    uint8_t data[14] = { 0 };
    vdcmd_long_header_t ptVdCmdHeader = { 0 };
    ptVdCmdHeader.byCmdType = CMDTYPE_LONG_TYPE_TPD;
    ptVdCmdHeader.bySubCmd  = SUBCMD_TPD_GET_RECT_TEMP;
    uint32_t dwAddr1 = (((uint32_t)rect_pos.start_point.x) << 16U) | rect_pos.start_point.y;
    uint32_t dwAddr2 = (((uint32_t)rect_pos.end_point.x) << 16U) | rect_pos.end_point.y;

    ir_error_t rst = long_cmd_read(&ptVdCmdHeader, 0U, dwAddr1, dwAddr2, 14U, data);
    rect_temp_info->temp_info_value.ave_temp = (uint16_t)(((uint16_t)data[0] << 8U) | data[1]);
    rect_temp_info->temp_info_value.max_temp = (uint16_t)(((uint16_t)data[2] << 8U) | data[3]);
    rect_temp_info->temp_info_value.min_temp = (uint16_t)(((uint16_t)data[4] << 8U) | data[5]);
    rect_temp_info->max_temp_point.x         = (uint16_t)(((uint16_t)data[6] << 8U) | data[7]);
    rect_temp_info->max_temp_point.y         = (uint16_t)(((uint16_t)data[8] << 8U) | data[9]);
    rect_temp_info->min_temp_point.x         = (uint16_t)(((uint16_t)data[10] << 8U) | data[11]);
    rect_temp_info->min_temp_point.y         = (uint16_t)(((uint16_t)data[12] << 8U) | data[13]);
    return rst;
}

ir_error_t tpd_set_rect_temp_thd(TpdTempThd_t *rect_temp_thd)
{
    (void)rect_temp_thd;
    /*
     * Lưu ý theo đánh giá SDK: Bảng Excel Interface Description1.2.xlsx (dòng 28) có liệt kê
     * tên hàm tpd_set_rect_temp_thd, nhưng mã nguồn phát hành của hãng (falcon_cmd.c/h)
     * đã lược bỏ không cài đặt hàm này. Trả về IR_UNSUPPORTED_CMD rõ ràng.
     */
    printf("[T3-FALCON] Info: tpd_set_rect_temp_thd is omitted in Vendor SDK V1.0.0 -> IR_UNSUPPORTED_CMD\n");
    return IR_UNSUPPORTED_CMD;
}

ir_error_t zoom_center_up(enum preview_path path, enum zoom_scale_step scale_step)
{
    vdcmd_long_header_t ptVdCmdHeader = { 0 };
    ptVdCmdHeader.byCmdType = CMDTYPE_LONG_TYPE_ZOOM;
    ptVdCmdHeader.bySubCmd  = SUBCMD_ZOOM_CENTER_UP;
    uint16_t wParam = (uint16_t)(((uint16_t)path << 8U) | (uint16_t)scale_step);
    return long_cmd_write(&ptVdCmdHeader, wParam, 0U, 0U, 0U);
}

ir_error_t zoom_center_down(enum preview_path path, enum zoom_scale_step scale_step)
{
    vdcmd_long_header_t ptVdCmdHeader = { 0 };
    ptVdCmdHeader.byCmdType = CMDTYPE_LONG_TYPE_ZOOM;
    ptVdCmdHeader.bySubCmd  = SUBCMD_ZOOM_CENTER_DOWN;
    uint16_t wParam = (uint16_t)(((uint16_t)path << 8U) | (uint16_t)scale_step);
    return long_cmd_write(&ptVdCmdHeader, wParam, 0U, 0U, 0U);
}

ir_error_t zoom_position_up(enum preview_path path, enum zoom_scale_step scale_step, IrPoint_t position)
{
    vdcmd_long_header_t ptVdCmdHeader = { 0 };
    ptVdCmdHeader.byCmdType = CMDTYPE_LONG_TYPE_ZOOM;
    ptVdCmdHeader.bySubCmd  = SUBCMD_ZOOM_POSITION_UP;
    uint16_t wParam  = (uint16_t)(((uint16_t)path << 8U) | (uint16_t)scale_step);
    uint32_t dwAddr1 = (((uint32_t)position.x) << 16U) | position.y;
    return long_cmd_write(&ptVdCmdHeader, wParam, dwAddr1, 0U, 0U);
}

ir_error_t zoom_position_down(enum preview_path path, enum zoom_scale_step scale_step, IrPoint_t position)
{
    vdcmd_long_header_t ptVdCmdHeader = { 0 };
    ptVdCmdHeader.byCmdType = CMDTYPE_LONG_TYPE_ZOOM;
    ptVdCmdHeader.bySubCmd  = SUBCMD_ZOOM_POSITION_DOWN;
    uint16_t wParam  = (uint16_t)(((uint16_t)path << 8U) | (uint16_t)scale_step);
    uint32_t dwAddr1 = (((uint32_t)position.x) << 16U) | position.y;
    return long_cmd_write(&ptVdCmdHeader, wParam, dwAddr1, 0U, 0U);
}

ir_error_t dpc_add_point(IrPoint_t position)
{
    vdcmd_long_header_t ptVdCmdHeader = { 0 };
    ptVdCmdHeader.byCmdType = CMDTYPE_LONG_TYPE_DPC;
    ptVdCmdHeader.bySubCmd  = SUBCMD_DPC_ADD_DP;
    return long_cmd_write(&ptVdCmdHeader, 0U, position.x, position.y, 0U);
}

ir_error_t dpc_remove_point(IrPoint_t position)
{
    vdcmd_long_header_t ptVdCmdHeader = { 0 };
    ptVdCmdHeader.byCmdType = CMDTYPE_LONG_TYPE_DPC;
    ptVdCmdHeader.bySubCmd  = SUBCMD_DPC_REMOVE_DP;
    return long_cmd_write(&ptVdCmdHeader, 0U, position.x, position.y, 0U);
}

ir_error_t dpc_auto_calibration(uint16_t wait_time)
{
    vdcmd_std_header_t ptVdCmdHeader = { 0 };
    SdpcParma_t sdpc_param = { 0 };
    DdpcParma_t ddpc_param = { 0 };

    ptVdCmdHeader.byCmdType = CMDTYPE_STANDARD_TYPE_ISP_PARAM;
    ptVdCmdHeader.bySubCmd  = SUBCMD_ISP_PARAM_GET;
    ptVdCmdHeader.byPara    = (uint8_t)SDPC;
    ir_error_t ret = standard_cmd_read(&ptVdCmdHeader, (uint16_t)sizeof(sdpc_param), (uint8_t *)&sdpc_param);
    if (ret != IR_SUCCESS) {
        return ret;
    }

    ptVdCmdHeader.bySubCmd = SUBCMD_ISP_PARAM_SET;
    sdpc_param.bySdpRpEn   = 1U;
    ret = standard_cmd_write(&ptVdCmdHeader, (uint16_t)sizeof(sdpc_param), (uint8_t *)&sdpc_param);
    if (ret != IR_SUCCESS) {
        return ret;
    }

    ptVdCmdHeader.bySubCmd = SUBCMD_ISP_PARAM_GET;
    ptVdCmdHeader.byPara   = (uint8_t)DDPC;
    ret = standard_cmd_read(&ptVdCmdHeader, (uint16_t)sizeof(ddpc_param), (uint8_t *)&ddpc_param);
    if (ret != IR_SUCCESS) {
        return ret;
    }

    ptVdCmdHeader.bySubCmd = SUBCMD_ISP_PARAM_SET;
    ddpc_param.byBpEn      = 1U;
    ret = standard_cmd_write(&ptVdCmdHeader, (uint16_t)sizeof(ddpc_param), (uint8_t *)&ddpc_param);
    if (ret != IR_SUCCESS) {
        return ret;
    }

    HAL_Delay(1000U * (uint32_t)wait_time);

    ptVdCmdHeader.bySubCmd = SUBCMD_ISP_PARAM_SET;
    ptVdCmdHeader.byPara   = (uint8_t)SDPC;
    sdpc_param.bySdpRpEn   = 0U;
    ret = standard_cmd_write(&ptVdCmdHeader, (uint16_t)sizeof(sdpc_param), (uint8_t *)&sdpc_param);
    if (ret != IR_SUCCESS) {
        return ret;
    }

    ptVdCmdHeader.bySubCmd = SUBCMD_ISP_PARAM_SET;
    ptVdCmdHeader.byPara   = (uint8_t)DDPC;
    ddpc_param.byBpEn      = 0U;
    return standard_cmd_write(&ptVdCmdHeader, (uint16_t)sizeof(ddpc_param), (uint8_t *)&ddpc_param);
}

ir_error_t xdata_read(uint16_t addr, uint8_t bylen, uint8_t *pbyData)
{
    vdcmd_short_header_t ptVdCmdShortHeader = {
        .byCmd    = CMDTYPE_SHORT_TYPE_XMEM_READ,
        .byLen    = bylen,
        .byAddr_h = (uint8_t)(addr >> 8U),
        .byAddr_l = (uint8_t)addr
    };
    return short_cmd_read(&ptVdCmdShortHeader, bylen, pbyData);
}

ir_error_t xdata_write(uint16_t addr, uint8_t bylen, uint8_t *pbyData)
{
    vdcmd_short_header_t ptVdCmdShortHeader = {
        .byCmd    = CMDTYPE_SHORT_TYPE_XMEM_WRITE,
        .byLen    = bylen,
        .byAddr_h = (uint8_t)(addr >> 8U),
        .byAddr_l = (uint8_t)addr
    };
    return short_cmd_write(&ptVdCmdShortHeader, bylen, pbyData);
}

ir_error_t falcon_spi_read(uint32_t addr, uint16_t wlen, uint8_t *pbyData)
{
    vdcmd_std_header_t ptVdCmdHeader = {
        .byCmdType = CMDTYPE_STANDARD_TYPE_SPI,
        .bySubCmd  = SUBCMD_SPI_READ,
        .byPara    = (uint8_t)(addr >> 24U),
        .byAddr_h  = (uint8_t)(addr >> 16U),
        .byAddr_l  = (uint8_t)(addr >> 8U),
        .byAddr_ll = (uint8_t)addr
    };
    return standard_cmd_read(&ptVdCmdHeader, wlen, pbyData);
}

ir_error_t falcon_spi_write(uint32_t addr, uint16_t wlen, uint8_t *pbyData)
{
    vdcmd_std_header_t ptVdCmdHeader = {
        .byCmdType = CMDTYPE_STANDARD_TYPE_SPI,
        .bySubCmd  = SUBCMD_SPI_WRITE,
        .byPara    = (uint8_t)(addr >> 24U),
        .byAddr_h  = (uint8_t)(addr >> 16U),
        .byAddr_l  = (uint8_t)(addr >> 8U),
        .byAddr_ll = (uint8_t)addr
    };
    return standard_cmd_write(&ptVdCmdHeader, wlen, pbyData);
}

ir_error_t spi_erase_sector(uint32_t addr, uint16_t sector_cnt)
{
    ir_error_t rst = IR_SUCCESS;
    const uint16_t one_sector_size = 4096U;
    const uint16_t once_sector_cnt = 8U;
    vdcmd_std_header_t ptVdCmdHeader = { 0 };
    uint32_t tmp_addr = addr;

    while (sector_cnt > 0U) {
        uint16_t i = (sector_cnt >= once_sector_cnt) ? once_sector_cnt : sector_cnt;

        ptVdCmdHeader.byCmdType = CMDTYPE_STANDARD_TYPE_SPI;
        ptVdCmdHeader.bySubCmd  = SUBCMD_SPI_ERASE;
        ptVdCmdHeader.byPara    = (uint8_t)(tmp_addr >> 24U);
        ptVdCmdHeader.byAddr_h  = (uint8_t)(tmp_addr >> 16U);
        ptVdCmdHeader.byAddr_l  = (uint8_t)(tmp_addr >> 8U);
        ptVdCmdHeader.byAddr_ll = (uint8_t)tmp_addr;
        ptVdCmdHeader.byLen_h   = (uint8_t)(i >> 8U);
        ptVdCmdHeader.byLen_l   = (uint8_t)i;

        rst = standard_cmd_write(&ptVdCmdHeader, 0U, NULL);
        if (rst < 0) {
            return rst;
        }

        tmp_addr   += (uint32_t)i * (uint32_t)one_sector_size;
        sector_cnt -= i;
    }
    return rst;
}

ir_error_t spi_erase_fw(void)
{
    vdcmd_std_header_t ptVdCmdHeader = {
        .byCmdType = CMDTYPE_STANDARD_TYPE_SPI,
        .bySubCmd  = SUBCMD_SPI_ERASE_FW
    };
    return standard_cmd_write(&ptVdCmdHeader, 0U, NULL);
}

ir_error_t spi_write_status(uint8_t status_reg_type, uint8_t status_val)
{
    uint8_t pbyData = status_val;
    vdcmd_std_header_t ptVdCmdHeader = {
        .byCmdType = CMDTYPE_STANDARD_TYPE_SPI,
        .bySubCmd  = SUBCMD_SPI_WRSR,
        .byPara    = status_reg_type
    };
    return standard_cmd_write(&ptVdCmdHeader, 1U, &pbyData);
}

ir_error_t spi_read_status(uint8_t status_reg_type, uint8_t *pstatus_val)
{
    vdcmd_std_header_t ptVdCmdHeader = {
        .byCmdType = CMDTYPE_STANDARD_TYPE_SPI,
        .bySubCmd  = SUBCMD_SPI_RDSR,
        .byPara    = status_reg_type
    };
    return standard_cmd_read(&ptVdCmdHeader, 1U, pstatus_val);
}

ir_error_t spi_write_tag(void)
{
    vdcmd_std_header_t ptVdCmdHeader = {
        .byCmdType = CMDTYPE_STANDARD_TYPE_SPI,
        .bySubCmd  = SUBCMD_SPI_WRITE_TAG
    };
    return standard_cmd_write(&ptVdCmdHeader, 0U, NULL);
}

ir_error_t spi_read_id(uint8_t *pbyID)
{
    vdcmd_std_header_t ptVdCmdHeader = {
        .byCmdType = CMDTYPE_STANDARD_TYPE_SPI,
        .bySubCmd  = SUBCMD_SPI_READ_ID
    };
    return standard_cmd_read(&ptVdCmdHeader, 6U, pbyID);
}

ir_error_t spi_config_save(enum prop_spi_cfg_module spi_module)
{
    vdcmd_std_header_t ptVdCmdHeader = {
        .byCmdType = CMDTYPE_STANDARD_TYPE_SPI,
        .bySubCmd  = SUBCMD_SPI_CFG_SAVE,
        .byPara    = (uint8_t)spi_module
    };
    return standard_cmd_write(&ptVdCmdHeader, 0U, NULL);
}

ir_error_t spi_config_load(enum prop_spi_cfg_module spi_module)
{
    if (spi_module == SPI_MOD_CFG_DEAD_PIX) {
        return IR_ERROR_PARAM;
    }
    vdcmd_std_header_t ptVdCmdHeader = {
        .byCmdType = CMDTYPE_STANDARD_TYPE_SPI,
        .bySubCmd  = SUBCMD_SPI_CFG_LOAD,
        .byPara    = (uint8_t)spi_module
    };
    return standard_cmd_write(&ptVdCmdHeader, 0U, NULL);
}

ir_error_t sys_reset_to_update_fw(void)
{
    vdcmd_std_header_t ptVdCmdHeader = {
        .byCmdType = CMDTYPE_STANDARD_TYPE_SYS,
        .bySubCmd  = SUBCMD_SYS_RST_TO_UPDATEFW
    };
    return standard_cmd_write(&ptVdCmdHeader, 0U, NULL);
}

ir_error_t sys_reset_to_rom(void)
{
    vdcmd_std_header_t ptVdCmdHeader = {
        .byCmdType = CMDTYPE_STANDARD_TYPE_SYS,
        .bySubCmd  = SUBCMD_SYS_RST_TO_ROM
    };
    return standard_cmd_write(&ptVdCmdHeader, 0U, NULL);
}

ir_error_t i2c_slave_read(uint16_t addr, uint8_t bylen, uint8_t *pbyData)
{
    vdcmd_short_header_t ptVdCmdShortHeader = {
        .byCmd    = CMDTYPE_SHORT_TYPE_XMEM_READ,
        .byLen    = bylen,
        .byAddr_h = (uint8_t)(addr >> 8U),
        .byAddr_l = (uint8_t)addr
    };
    return short_cmd_read(&ptVdCmdShortHeader, bylen, pbyData);
}

ir_error_t ir_read_id(uint8_t *sensor_id)
{
    vdcmd_std_header_t ptVdCmdHeader = {
        .byCmdType = CMDTYPE_STANDARD_TYPE_IR,
        .bySubCmd  = SUBCMD_IR_READ_ID
    };
    return standard_cmd_read(&ptVdCmdHeader, 10U, sensor_id);
}

#endif /* FALCON_CMD */
