/*
 * Tầng 3 (T3 - Tiny1C Thermal Camera Driver): Cấu hình chung và mã lỗi toàn cục.
 *
 * Tác dụng:
 *   - Định nghĩa các hằng số thời gian, thông số kết nối với Tầng 2
 *     (soft_i2c_bus_t / spi_bus_t) và bảng mã trạng thái trả về (ir_error_t)
 *     của khối ngoại vi Camera nhiệt Tiny1C (ASIC-2121W).
 *   - Đồng bộ hóa thời gian chờ lệnh dài (20 giây cho hiệu chuẩn 1/2 điểm)
 *     theo đúng SDK gốc của hãng (Tiny1C_STM32_SPI_SDK/VDCMD/all_config.h)
 *     và bảng Delay time.xlsx.
 */

#ifndef TINY1C_ALL_CONFIG_H
#define TINY1C_ALL_CONFIG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Never silently compile a simulated camera into production firmware. */
#if defined(__riscv)
# if defined(THERMAL_HOST_TEST)
#  error "THERMAL_HOST_TEST is forbidden on the camera target"
# endif
# include "FreeRTOS.h"
# include "soft_i2c.h"
/* spi.h is included by hal_spi.h to avoid vendor flash aliases. */
#elif !defined(THERMAL_HOST_TEST)
# error "Host builds must explicitly define THERMAL_HOST_TEST"
#endif


#ifdef __cplusplus
extern "C" {
#endif

/* Bật tập lệnh điều khiển chip xử lý ảnh nhiệt Falcon (ASIC-2121W) */
#define FALCON_CMD

/* --------------------------------------------------------------------------
 * 1. Thông số thời gian & cấu hình liên kết với Tầng 0 / Tầng 1 / Tầng 2
 * -------------------------------------------------------------------------- */

/* Tần số bộ đếm thời gian thực rdtime trên nhân RISC-V C906L của SG2002 (25 MHz XTAL) */
#ifndef THERMAL_RDTIME_HZ
#define THERMAL_RDTIME_HZ                   25000000U
#endif

/* Tần số xung nhịp I2C Bitbang T1/T2 (Standard-mode tối đa 100 kHz theo hal_i2c_bitbang.c) */
#ifndef THERMAL_I2C_FREQ_HZ
#define THERMAL_I2C_FREQ_HZ                 100000U
#endif

/* Thời gian tối đa cho phép Slave ASIC-2121W giữ SCL (Clock Stretching) tính bằng micro-giây */
#ifndef THERMAL_I2C_STRETCH_US
#define THERMAL_I2C_STRETCH_US              20000U
#endif

/* Thời gian chờ tối đa cho một giao dịch đọc/ghi thanh ghi I2C đơn lẻ ở Tầng 2 (mili-giây) */
#ifndef THERMAL_I2C_TIMEOUT_MS
#define THERMAL_I2C_TIMEOUT_MS              1000U
#endif

/*
 * Tổng thời gian chờ tối đa để ASIC-2121W xử lý xong lệnh VDCMD khi poll thanh ghi 0x0200 (ms).
 * Lưu ý theo SDK hãng (Tiny1C_STM32_SPI_SDK/VDCMD/all_config.h & Delay time.xlsx):
 *   - Lệnh thông thường (preview, ISP, TPD query): hoàn tất sau vài ms ~ vài chục ms.
 *   - Lệnh hiệu chuẩn nắp che (rmcover_auto_calc): cần tối đa ~5,000 ms (5 giây).
 *   - Lệnh hiệu chuẩn nhiệt 1 điểm / 2 điểm (tpd_ktbt_recal_1point/2point): cần tối đa 20,000 ms (20 giây).
 */
#ifndef I2C_TRANSFER_WAIT_TIME_MS
#define I2C_TRANSFER_WAIT_TIME_MS           20000U
#endif

/* Thời gian chờ khóa Mutex thiết bị Tầng 3 cho một chuỗi lệnh VDCMD (lớn hơn I2C_TRANSFER_WAIT_TIME_MS) */
#ifndef THERMAL_VDCMD_MUTEX_TIMEOUT_MS
#define THERMAL_VDCMD_MUTEX_TIMEOUT_MS      25000U
#endif

/*
 * Tần số xung nhịp SPI0 mặc định cho luồng VoSPI đơn (187.5 MHz / 8 = 23.4375 MHz)
 * và tốc độ cao cho luồng kép Image + Temp (187.5 MHz / 4 = 46.875 MHz).
 */
#ifndef THERMAL_SPI_DEFAULT_SCK_HZ
#define THERMAL_SPI_DEFAULT_SCK_HZ          11718750U
#endif

#ifndef THERMAL_SPI_DUAL_SCK_HZ
#define THERMAL_SPI_DUAL_SCK_HZ             11718750U
#endif

/* Độ trễ lấy mẫu đường MISO (rx_sample_dly) tính theo chu kỳ ssi_clk (0..8) */
#ifndef THERMAL_SPI_DEFAULT_RX_DLY
#define THERMAL_SPI_DEFAULT_RX_DLY          1U
#endif

/* Thời gian chờ hoàn tất thu nhận một gói dữ liệu VoSPI qua Tầng 2 (mili-giây) */
#ifndef THERMAL_SPI_TIMEOUT_MS
#define THERMAL_SPI_TIMEOUT_MS              1000U
#endif

/* Chu kỳ mặc định lấy khung nhiệt trong chế độ xuất xen kẽ (Interval Mode) */
#ifndef THERMAL_DUAL_DEFAULT_INTERVAL
#define THERMAL_DUAL_DEFAULT_INTERVAL       1U
#endif

/* --------------------------------------------------------------------------
 * 2. Bảng mã lỗi chuẩn của Driver Camera nhiệt (ir_error_t - khớp SDK hãng)
 * -------------------------------------------------------------------------- */

typedef enum {
    IR_SUCCESS                      = 0,   /* Thực thi thành công */
    IR_ERROR_PARAM                  = -1,  /* Tham số đầu vào không hợp lệ */
    IR_UVC_INIT_FAIL                = -2,  /* Khởi tạo ngữ cảnh dịch vụ thất bại */
    IR_GET_DEVICE_LIST_FAIL         = -3,  /* Lấy danh sách thiết bị thất bại */
    IR_FIND_DEVICE_FAIL             = -4,  /* Không tìm thấy thiết bị */
    IR_NOT_GET_DEVICE_INFO          = -5,  /* Không đọc được thông tin thiết bị */
    IR_DEVICE_OPEN_FAIL             = -6,  /* Mở thiết bị thất bại */
    IR_GET_DEVICE_DESCRIPTOR_FAIL   = -7,  /* Đọc descriptor thiết bị thất bại */
    IR_DEVICE_OPENED                = -8,  /* Thiết bị đã được mở trước đó */
    IR_GET_FORMAT_FAIL              = -9,  /* Định dạng khung hình không khớp */
    IR_USER_CALLBACK_EMPTY          = -10, /* Hàm callback người dùng rỗng */
    IR_START_STREAMING_FAIL         = -11, /* Khởi động luồng dữ liệu thất bại */
    IR_GET_FRAME_OVER_TIME          = -12, /* Quá thời gian chờ thu nhận khung hình */
    IR_VDCMD_NOT_REGISTER           = -13, /* Chưa đăng ký handle giao tiếp VDCMD */
    IR_VDCMD_REGISTER_FAIL          = -14, /* Đăng ký handle VDCMD thất bại */
    IR_CONTROL_TRANSFER_FAIL        = -15, /* Truyền lệnh điều khiển thất bại */
    IR_CHECK_DONE_FAIL              = -16, /* Kiểm tra trạng thái hoàn tất VDCMD thất bại */
    IR_VDCMD_TOO_LONG               = -17, /* Độ dài gói lệnh VDCMD vượt quá giới hạn */
    IR_RESET_DEVICE_FAIL            = -18, /* Reset thiết bị thất bại */
    IR_CLEAR_HALT_FAIL              = -19, /* Xóa trạng thái treo thất bại */
    IR_UNSUPPORTED_CMD              = -20, /* Mã lệnh không được hỗ trợ */
    IR_I2C_TRANSFER_FAIL            = -21, /* Lỗi truyền nhận trên Bus I2C */
    IR_I2C_DEVICE_OPEN_FAIL         = -22, /* Khởi tạo Bus I2C thất bại */
    IR_I2C_GET_REGISTER_FAIL        = -23, /* Đọc thanh ghi I2C thất bại */
    IR_I2C_SET_REGISTER_FAIL        = -24, /* Ghi thanh ghi I2C thất bại */
    IR_MEM_ALLOC_FAIL               = -25  /* Cấp phát bộ nhớ thất bại */
} ir_error_t;

/* Mức độ in nhật ký chẩn đoán (Log Levels) */
typedef enum {
    IR_LOG_DEBUG                    = 0,
    IR_LOG_ERROR                    = 1,
    IR_LOG_NO_PRINT                 = 2
} ir_log_level_t;

#ifdef __cplusplus
}
#endif

#endif /* TINY1C_ALL_CONFIG_H */
