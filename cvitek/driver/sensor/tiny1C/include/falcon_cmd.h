/*
 * Tầng 3 (T3 - Tiny1C Falcon ASIC-2121W Command Set): Tập lệnh điều khiển
 * chip xử lý ảnh nhiệt Falcon (ASIC-2121W) thông qua giao thức VDCMD.
 */

#ifndef TINY1C_FALCON_CMD_H
#define TINY1C_FALCON_CMD_H

#include <stdint.h>
#include "all_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#pragma pack(push, 1)

enum vdcmd_driver_type {
    VDCMD_USB_VDCMD = 0,
    VDCMD_I2C_USB_VDCMD,
    VDCMD_I2C_VDCMD,
    VDCMD_UART_VDCMD
};

enum prop_default_cfg {
    DEF_CFG_ALL        = 0, /* Khôi phục toàn bộ cấu hình mặc định */
    DEF_CFG_TECLESS_K  = 1, /* Khôi phục bảng K không TEC */
    DEF_CFG_TPD        = 2, /* Khôi phục tham số đo nhiệt TPD */
    DEF_CFG_PROP_PAGE  = 3, /* Khôi phục trang thuộc tính Property Page */
    DEF_CFG_USER_CFG   = 4, /* Khôi phục cấu hình người dùng */
    DEF_CFG_DEAD_PIXEL = 5  /* Khôi phục bảng điểm mù (mã 0x05 theo Interface Description1.2.xlsx) */
};

enum prop_default_params {
    PROP_SEL_SHUTTER = 0,
    PROP_SEL_IMAGE,
    PROP_SEL_OVEREXPOSURE,
    PROP_SEL_TPD
};

enum prop_spi_cfg_module {
    SPI_MOD_CFG_ALL = 0x00,
    SPI_MOD_CFG_DEAD_PIX = 0x05,
    SPI_MOD_CFG_PROPERTY_PAGE = 0x0C
};

enum prop_auto_shutter_params {
    SHUTTER_PROP_SWITCH = 0,
    SHUTTER_PROP_MIN_INTERVAL = 1,
    SHUTTER_PROP_MAX_INTERVAL = 2,
    SHUTTER_PROP_TEMP_THRESHOLD_B = 4,
    SHUTTER_PROP_PROTECT_SWITCH = 5,
    SHUTTER_PROP_ANY_INTERVAL = 6,
    SHUTTER_PROTECT_THR_HIGH_GAIN = 7,
    SHUTTER_PROTECT_THR_LOW_GAIN = 8,
    SHUTTER_PREVIEW_START_1ST_DELAY = 9,
    SHUTTER_PREVIEW_START_2ND_DELAY = 10,
    SHUTTER_CHANGE_GAIN_1ST_DELAY = 11,
    SHUTTER_CHANGE_GAIN_2ND_DELAY = 12
};

enum prop_image_params {
    IMAGE_PROP_LEVEL_TNR = 0,
    IMAGE_PROP_LEVEL_SNR = 1,
    IMAGE_PROP_LEVEL_DDE = 2,
    IMAGE_PROP_LEVEL_BRIGHTNESS = 3,
    IMAGE_PROP_LEVEL_CONTRAST = 4,
    IMAGE_PROP_MODE_AGC = 5,
    IMAGE_PROP_LEVEL_MAX_GAIN = 6,
    IMAGE_PROP_LEVEL_BOS = 7,
    IMAGE_PROP_ONOFF_AGC = 8,
    IMAGE_PROP_SEL_MIRROR_FLIP = 9
};

enum prop_tpd_params {
    TPD_PROP_DISTANCE = 0,
    TPD_PROP_TU,
    TPD_PROP_TA,
    TPD_PROP_EMS,
    TPD_PROP_TAU,
    TPD_PROP_GAIN_SEL,
    TPD_PROP_DIST_COR_FACT,
    TPD_PROP_ENV_CORRECT_SEL,
    TPD_PROP_DIST_CORRECT_SEL,
    TPD_PROP_TEMP_MAP_SEL
};

enum preview_path {
    PREVIEW_PATH0 = 0,
    PREVIEW_PATH1
};

enum device_id_types {
    DEV_INFO_CHIP_ID = 0,
    DEV_INFO_FW_COMPILE_DATE = 1,
    DEV_INFO_DEV_QUALIFICATION = 2,
    DEV_INFO_IR_INFO = 3,
    DEV_INFO_PROJECT_INFO = 4,
    DEV_INFO_FW_BUILD_VERSION_INFO = 5,
    DEV_INFO_GET_PN = 6,
    DEV_INFO_GET_SN = 7
};

enum ooc_b_update_types {
    B_UPDATE = 0,
    OOC_UPDATE,
    OOC_B_UPDATE
};

enum y16_isp_stream_src_types {
    Y16_MODE_TEMPERATURE = 0,
    Y16_MODE_IR,
    Y16_MODE_KBC,
    Y16_MODE_HBC_DPC,
    Y16_MODE_VBC,
    Y16_MODE_TNR,
    Y16_MODE_SNR,
    Y16_MODE_AGC,
    Y16_MODE_DDE,
    Y16_MODE_GAMMA,
    Y16_MODE_MIRROR
};

enum shutter_sta_types {
    SHUTTER_CTL_DIS = 0,
    SHUTTER_CTL_EN
};

enum shutter_manual_types {
    SHUTTER_OPEN = 0,
    SHUTTER_CLOSE
};

enum rmcover_types {
    RMCOVER_DIS = 0,
    RMCOVER_EN
};

enum tpd_ktbt_recal_point_idx {
    TPD_KTBT_RECAL_P1 = 0,
    TPD_KTBT_RECAL_P2
};

enum tecless_temp_area_types {
    HIGH_TEMP = 0,
    NORMAL_TEMP,
    LOW_TEMP
};

enum mirror_flip_types {
    NO_MIRROR_FLIP = 0,
    ONLY_MIRROR,
    ONLY_FLIP,
    MIRROR_FLIP
};

/*
 * Bảng màu giả (Pseudo Color Palette) của ASIC-2121W:
 * Lưu ý theo tài liệu Interface Description1.2.xlsx của Tiny1C:
 * Giá trị 2 (PSEUDO_COLOR_MODE_2) là RESERVED (không hỗ trợ trên Tiny1C).
 * Driver Tầng 3 sẽ chủ động chặn PSEUDO_COLOR_MODE_2 để tránh lỗi tham số dưới ASIC.
 */
enum pseudo_color_types {
    PSEUDO_COLOR_MODE_1 = 1,  /* White Hot (Trắng nóng) */
    PSEUDO_COLOR_MODE_2,      /* RESERVED trên Tiny1C - Không sử dụng */
    PSEUDO_COLOR_MODE_3,      /* Sexy / Ironbow */
    PSEUDO_COLOR_MODE_4,      /* Rainbow */
    PSEUDO_COLOR_MODE_5,      /* Aurora */
    PSEUDO_COLOR_MODE_6,      /* Lava */
    PSEUDO_COLOR_MODE_7,      /* Glowbow */
    PSEUDO_COLOR_MODE_8,      /* Gradedfire */
    PSEUDO_COLOR_MODE_9,      /* Hottest */
    PSEUDO_COLOR_MODE_10,     /* Medical */
    PSEUDO_COLOR_MODE_11      /* Black Hot (Đen nóng) */
};

enum zoom_scale_step {
    ZOOM_STEP1 = 1,
    ZOOM_STEP2,
    ZOOM_STEP3,
    ZOOM_STEP4
};

typedef struct {
    uint8_t path;
    uint8_t source;
    uint16_t width;
    uint16_t height;
    uint8_t fps;
    uint8_t mode;
} PreviewStartParam_t;

typedef struct {
    uint16_t x;
    uint16_t y;
} IrPoint_t;

typedef struct {
    IrPoint_t start_point;
    IrPoint_t end_point;
} IrLine_t;

typedef struct {
    IrPoint_t start_point;
    IrPoint_t end_point;
} IrRect_t;

typedef struct {
    uint8_t max_point_idx;
    uint8_t min_point_idx;
    uint8_t center_point_idx;
} FrameTempIdx_t;

typedef struct {
    uint16_t max_temp;
    uint16_t min_temp;
    IrPoint_t max_temp_point;
    IrPoint_t min_temp_point;
} MaxMinTempInfo_t;

typedef struct {
    uint16_t ave_temp;
    uint16_t max_temp;
    uint16_t min_temp;
} TpdTempInfoValue_t;

typedef struct {
    TpdTempInfoValue_t temp_info_value;
    IrPoint_t max_temp_point;
    IrPoint_t min_temp_point;
} TpdLineRectTempInfo_t;

typedef struct {
    uint16_t ave_thd;
    uint16_t high_thd;
    uint16_t low_thd;
} TpdTempThd_t;

#pragma pack(pop)

/* Khởi tạo giao thức VDCMD theo loại bus điều khiển */
ir_error_t vdcmd_init_by_type(enum vdcmd_driver_type type);

/* Nhóm lệnh Cấu hình Trang thuộc tính (Property Page) */
ir_error_t restore_default_cfg(enum prop_default_cfg default_cfg_type);
ir_error_t load_prop_default_params(enum prop_default_params default_param);
ir_error_t set_prop_auto_shutter_params(enum prop_auto_shutter_params shutter_param, uint16_t value);
ir_error_t get_prop_auto_shutter_params(enum prop_auto_shutter_params shutter_param, uint16_t *value);
ir_error_t set_prop_image_params(enum prop_image_params image_param, uint16_t value);
ir_error_t get_prop_image_params(enum prop_image_params image_param, uint16_t *value);
ir_error_t set_prop_tpd_params(enum prop_tpd_params tpd_param, uint16_t value);
ir_error_t get_prop_tpd_params(enum prop_tpd_params tpd_param, uint16_t *value);

/* Nhóm lệnh Thông tin thiết bị, Luồng Video & Màn trập */
ir_error_t get_device_info(enum device_id_types id_type, uint8_t *id_content);
ir_error_t ir_read_id(uint8_t *sensor_id);
ir_error_t y16_preview_start(enum preview_path path, enum y16_isp_stream_src_types src_type);
ir_error_t y16_preview_stop(enum preview_path path);
ir_error_t preview_start(PreviewStartParam_t *preview_start_param);
ir_error_t preview_stop(enum preview_path path);
ir_error_t ooc_b_update(enum ooc_b_update_types update_type);
ir_error_t shutter_sta_get(uint8_t *shutter_en_sta, uint8_t *shutter_sta);
ir_error_t shutter_sta_set(enum shutter_sta_types sta_type);
ir_error_t shutter_manual_switch(enum shutter_manual_types manual_type);
ir_error_t shutter_vtemp_get(uint16_t *cur_vtemp);
ir_error_t cur_vtemp_get(uint16_t *cur_vtemp);

/* Nhóm lệnh Bù nắp ống kính (Remove Cover), Bảng màu giả & Đo nhiệt TPD */
ir_error_t rmcover_sts_switch(enum rmcover_types en_sts);
ir_error_t rmcover_auto_calc(uint8_t zoom_scale);
ir_error_t pseudo_color_get(enum preview_path path, uint8_t *color_type);
ir_error_t pseudo_color_set(enum preview_path path, enum pseudo_color_types color_type);
ir_error_t tpd_ktbt_recal_1point(uint16_t temp);
ir_error_t tpd_ktbt_recal_2point(enum tpd_ktbt_recal_point_idx point_idx, uint16_t temp);
ir_error_t tpd_get_max_temp(uint16_t *max_temp);
ir_error_t tpd_get_min_temp(uint16_t *min_temp);
ir_error_t tpd_get_max_min_temp_info(MaxMinTempInfo_t *max_min_temp_info);
ir_error_t tpd_get_point_temp_info(IrPoint_t point_pos, uint16_t *point_temp_value);
ir_error_t tpd_get_line_temp_info(IrLine_t line_pos, TpdLineRectTempInfo_t *line_temp_info);
ir_error_t tpd_get_rect_temp_info(IrRect_t rect_pos, TpdLineRectTempInfo_t *rect_temp_info);
/*
 * Hàm đặt ngưỡng nhiệt độ vùng chữ nhật: có liệt kê trong bảng Excel Interface Description1.2.xlsx
 * (dòng 28) nhưng hãng đã lược bỏ trong mã nguồn SDK V1.0.0 -> trả về IR_UNSUPPORTED_CMD.
 */
ir_error_t tpd_set_rect_temp_thd(TpdTempThd_t *rect_temp_thd);

/* Nhóm lệnh Thu phóng kỹ thuật số (Digital Zoom) & Điểm chết (DPC) */
ir_error_t zoom_center_up(enum preview_path path, enum zoom_scale_step scale_step);
ir_error_t zoom_center_down(enum preview_path path, enum zoom_scale_step scale_step);
ir_error_t zoom_position_up(enum preview_path path, enum zoom_scale_step scale_step, IrPoint_t position);
ir_error_t zoom_position_down(enum preview_path path, enum zoom_scale_step scale_step, IrPoint_t position);
ir_error_t dpc_add_point(IrPoint_t position);
ir_error_t dpc_remove_point(IrPoint_t position);
ir_error_t dpc_auto_calibration(uint16_t wait_time);

/* Nhóm lệnh Truy cập XDATA, I2C Slave & SPI Flash nội bộ của ASIC-2121W */
ir_error_t xdata_read(uint16_t addr, uint8_t bylen, uint8_t *pbyData);
ir_error_t xdata_write(uint16_t addr, uint8_t bylen, uint8_t *pbyData);
ir_error_t i2c_slave_read(uint16_t addr, uint8_t bylen, uint8_t *pbyData);

/*
 * Lưu ý quan trọng về phân tầng:
 * Tầng 2 (cvitek/driver/spi/include/spi.h) có định nghĩa static inline spi_read() và spi_write().
 * Để tránh xung đột biểu tượng khi liên kết chung, hàm đọc/ghi Flash nội bộ của ASIC-2121W
 * được đặt tên chuẩn là falcon_spi_read() / falcon_spi_write(), đồng thời ánh xạ macro
 * spi_read / spi_write khi không nạp trực tiếp spi.h của Tầng 2.
 */
ir_error_t falcon_spi_read(uint32_t addr, uint16_t wlen, uint8_t *pbyData);
ir_error_t falcon_spi_write(uint32_t addr, uint16_t wlen, uint8_t *pbyData);
#if !defined(CVI_DRIVER_SPI_H) && !(defined(__has_include) && __has_include("spi.h") && __has_include("FreeRTOS.h"))
#define spi_read  falcon_spi_read
#define spi_write falcon_spi_write
#endif

ir_error_t spi_erase_sector(uint32_t addr, uint16_t sector_cnt);
ir_error_t spi_erase_fw(void);
ir_error_t spi_write_status(uint8_t status_reg_type, uint8_t status_val);
ir_error_t spi_read_status(uint8_t status_reg_type, uint8_t *pstatus_val);
ir_error_t spi_write_tag(void);
ir_error_t spi_read_id(uint8_t *pbyID);
ir_error_t spi_config_save(enum prop_spi_cfg_module spi_module);
ir_error_t spi_config_load(enum prop_spi_cfg_module spi_module);
ir_error_t sys_reset_to_update_fw(void);
ir_error_t sys_reset_to_rom(void);

#ifdef __cplusplus
}
#endif

#endif /* TINY1C_FALCON_CMD_H */
