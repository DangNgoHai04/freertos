/*
 * Tầng 3 (T3 - Tiny1C I2C Control Transport Adapter): Giao tiếp đọc/ghi thanh ghi 16-bit
 * của chip ASIC-2121W thông qua API Tầng 2 (soft_i2c_bus_t) và Tầng 0 (bb_gpio_platform_init).
 */

#include "data_rw.h"
#include <stdio.h>
#include <string.h>

#if defined(TINY1C_HAS_T2_SOFT_I2C) && defined(__has_include) && __has_include("hal_i2c_gpio_platform.h")
#include "hal_i2c_gpio_platform.h"
#define TINY1C_HAS_T0_I2C_PLATFORM 1
#endif

#if defined(__riscv) && defined(__has_include) && __has_include("FreeRTOS.h")
#define TINY1C_HAS_FREERTOS 1
#include "FreeRTOS.h"
#include "task.h"
#endif

static uint8_t s_i2c_bus = 5U;              /* Mặc định Bus 5 (Soft-I2C trên GPIOA15/GPIOA27) */
static uint16_t s_slave_id = I2C_SLAVE_ID;  /* Mặc định 0x78 (8-bit) -> 0x3C (7-bit) */
static bool s_i2c_hw_ready = false;
static bool s_gpioa_ownership_confirmed = false; /* Cam kết cô lập bank GPIOA giữa RTOS và Linux */
static uint32_t s_cmd_wait_timeout_ms = I2C_TRANSFER_WAIT_TIME_MS; /* Mặc định 20,000 ms theo SDK hãng */
static tiny1c_i2c_diag_t s_i2c_diag;
#if !defined(__riscv)
static uint64_t s_host_sim_clock_ms = 0ULL;
#endif

#if defined(TINY1C_HAS_T2_SOFT_I2C)
static soft_i2c_bus_t s_default_soft_i2c_bus;
static soft_i2c_bus_t *s_bound_i2c_bus = NULL;
static bb_io_t s_default_i2c_io;
#endif

void data_rw_set_gpioa_ownership(bool confirmed)
{
    s_gpioa_ownership_confirmed = confirmed;
}

void data_rw_reset_i2c_diag(void)
{
    s_i2c_diag = (tiny1c_i2c_diag_t){.nack_phase = 0xffU};
}

void data_rw_set_probe_stage(uint8_t stage)
{
    s_i2c_diag.stage = stage;
}

void data_rw_mark_probe_ok(uint8_t bit)
{
    if (bit < 8U) s_i2c_diag.probe_ok_mask |= (uint8_t)(1U << bit);
}

tiny1c_i2c_diag_t data_rw_get_i2c_diag(void)
{
    return s_i2c_diag;
}

int data_rw_scan_i2c(uint8_t *first_ack, uint8_t *ack_count)
{
    if (first_ack == NULL || ack_count == NULL) return -1;
    *first_ack = 0xffU;
    *ack_count = 0U;
#if defined(TINY1C_HAS_T2_SOFT_I2C) && defined(__riscv)
    if (!s_i2c_hw_ready || s_bound_i2c_bus == NULL) return -1;
    for (uint8_t addr = 0x08U; addr <= 0x77U; ++addr) {
        bb_msg_t msg = {.addr = addr, .read = false, .data = NULL, .len = 0U};
        bb_status_t st = soft_i2c_transfer(s_bound_i2c_bus, &msg, 1U, 20U);
        if (st == BB_OK) {
            if (*ack_count == 0U) *first_ack = addr;
            ++*ack_count;
        } else if (st != BB_NACK) {
            return -1;
        }
    }
    return 0;
#else
    return -1;
#endif
}

void data_rw_set_cmd_wait_timeout_ms(uint32_t wait_ms)
{
    if (wait_ms > 0U) {
        s_cmd_wait_timeout_ms = wait_ms;
    }
}

uint32_t data_rw_get_cmd_wait_timeout_ms(void)
{
    return s_cmd_wait_timeout_ms;
}

uint64_t data_rw_now_ms(void)
{
#if defined(TINY1C_HAS_FREERTOS)
    uint64_t cycles;
    __asm__ volatile("rdtime %0" : "=r"(cycles));
    return cycles / (THERMAL_RDTIME_HZ / 1000U);
#else
    s_host_sim_clock_ms += 1ULL;
    return s_host_sim_clock_ms;
#endif
}

void data_rw_set_i2c_bus(uint8_t i2c_bus_id)
{
    s_i2c_bus = i2c_bus_id;
}

uint8_t data_rw_get_i2c_bus(void)
{
    return s_i2c_bus;
}

void data_rw_set_slave_addr(uint16_t slave_id)
{
    if (slave_id != 0U) {
        s_slave_id = slave_id;
    }
}

uint16_t data_rw_get_slave_addr(void)
{
    return s_slave_id;
}

/*
 * Chuẩn hóa địa chỉ Slave về 7-bit (0x3C) theo đúng hợp đồng Tầng 1 (hal_i2c_bitbang.c).
 * Lưu ý quan trọng: 0x78 (I2C_SLAVE_ID trong SDK gốc) là địa chỉ 8-bit (0x3C << 1),
 * dù nhỏ hơn 0x7F nhưng bắt buộc phải dịch phải 1 bit về 0x3C (I2C_SLAVE_7BIT_ADDR)
 * để Tầng 1 không dịch trái thêm lần nữa thành 0xF0 trên đường truyền vật lý!
 */
static inline uint16_t get_7bit_addr(uint16_t slave_id)
{
    if ((slave_id == I2C_SLAVE_ID) || (slave_id == (I2C_SLAVE_ID | 1U)) || (slave_id > 0x7FU)) {
        return (uint16_t)(slave_id >> 1U);
    }
    return slave_id;
}

void delay_ms(uint32_t ms)
{
    if (ms == 0U) {
        return;
    }
#if defined(TINY1C_HAS_FREERTOS)
    uint64_t begin, now;
    __asm__ volatile("rdtime %0" : "=r"(begin));
    uint64_t duration = (uint64_t)ms * (THERMAL_RDTIME_HZ / 1000U);
    TickType_t ticks = pdMS_TO_TICKS(ms);
    if (ticks > 1U) vTaskDelay(ticks - 1U);
    /* vTaskDelay(1) can return before a full millisecond at a tick boundary. */
    do { __asm__ volatile("rdtime %0" : "=r"(now)); } while (now - begin < duration);
#else
    s_host_sim_clock_ms += (uint64_t)ms;
#endif
}

#if defined(TINY1C_HAS_T2_SOFT_I2C)
ir_error_t data_rw_bind_soft_i2c_bus(soft_i2c_bus_t *bus)
{
    if ((bus == NULL) || (!bus->ready)) {
        return IR_ERROR_PARAM;
    }
    s_bound_i2c_bus = bus;
    s_i2c_hw_ready  = true;
    return IR_SUCCESS;
}

soft_i2c_bus_t *data_rw_get_soft_i2c_bus(void)
{
    return s_bound_i2c_bus;
}
#endif

ir_error_t data_rw_i2c_hw_init(uint8_t i2c_bus_id)
{
    s_i2c_bus = i2c_bus_id;

    if (s_i2c_hw_ready) {
        return IR_SUCCESS;
    }

    if (!s_gpioa_ownership_confirmed) {
        s_i2c_diag.bb_status = -6; /* BB_NOT_READY: also available in host tests. */
        printf("[T3-I2C] Error: GPIOA ownership not confirmed! Refusing to init T0 Soft-I2C.\n");
        return IR_I2C_DEVICE_OPEN_FAIL;
    }

#if defined(TINY1C_HAS_T2_SOFT_I2C) && defined(TINY1C_HAS_T0_I2C_PLATFORM) && defined(__riscv)
    /*
     * Khởi tạo Tầng 0 (GPIOA15 SCL, GPIOA27 SDA + Pinmux + bộ đếm rdtime 25 MHz)
     * và Tầng 2 (soft_i2c_init ở 100 kHz Standard-Mode, clock stretch 20ms).
     */
    if (bb_gpio_platform_init(&s_default_i2c_io,
                              THERMAL_RDTIME_HZ,
                              s_gpioa_ownership_confirmed) != BB_OK) {
        s_i2c_diag.bb_status = BB_NOT_READY;
        return IR_I2C_DEVICE_OPEN_FAIL;
    }

    bb_status_t st = soft_i2c_init(&s_default_soft_i2c_bus,
                                   &s_default_i2c_io,
                                   THERMAL_I2C_FREQ_HZ,
                                   THERMAL_I2C_STRETCH_US);
    if (st != BB_OK) {
        s_i2c_diag.bb_status = st;
        printf("[T3-I2C] soft_i2c_init failed: bb_status=%d\n", (int)st);
        return IR_I2C_DEVICE_OPEN_FAIL;
    }

    s_bound_i2c_bus = &s_default_soft_i2c_bus;
    printf("[T3-I2C] Bitbang frequency=%d Hz, stretch=%d us\n",
           (int)THERMAL_I2C_FREQ_HZ, (int)THERMAL_I2C_STRETCH_US);
#endif

    s_i2c_hw_ready = true;
    return IR_SUCCESS;
}

ir_error_t data_rw_i2c_recover(void)
{
#if defined(TINY1C_HAS_T2_SOFT_I2C) && defined(__riscv)
    if (s_bound_i2c_bus == NULL) {
        return IR_I2C_DEVICE_OPEN_FAIL;
    }
    bb_status_t st = soft_i2c_recover(s_bound_i2c_bus, THERMAL_I2C_TIMEOUT_MS);
    return (st == BB_OK) ? IR_SUCCESS : IR_I2C_TRANSFER_FAIL;
#else
    return IR_SUCCESS;
#endif
}

#if !defined(TINY1C_HAS_T2_SOFT_I2C) || !defined(__riscv)
/*
 * Bộ nhớ giả lập thanh ghi nội bộ ASIC-2121W cho chế độ kiểm thử độc lập trên Host PC.
 * Mô phỏng đầy đủ phản hồi cho:
 *  - Thanh ghi trạng thái 0x0200 (IDLE + PASS).
 *  - Lệnh đọc thông tin thiết bị (DEV_INFO_PROJECT_INFO, FW_COMPILE_DATE, PN, SN).
 *  - Lệnh đọc/ghi bảng màu giả (pseudo_color_get/set).
 *  - Lệnh đọc nhiệt độ TPD (Max/Min, Point, Line, Rect) theo đơn vị Kelvin * 16.
 */
static uint8_t s_host_vdcmd_buf[512];
static uint8_t s_host_sim_palette = 1U;

__attribute__((weak)) int i2c_read(uint8_t i2c_id, uint8_t dev, uint16_t addr,
                                   uint16_t alen, uint8_t *buffer, uint16_t len)
{
    (void)i2c_id;
    (void)dev;
    (void)alen;
    if ((buffer != NULL) && (len > 0U)) {
        memset(buffer, 0, len);
        if (addr == I2C_VD_BUFFER_STATUS) {
            buffer[0] = VCMD_BUSY_STS_IDLE | VCMD_RST_STS_PASS;
        } else if ((addr >= I2C_VD_BUFFER_RW) &&
                   ((uint32_t)(addr - I2C_VD_BUFFER_RW) + len <= sizeof(s_host_vdcmd_buf))) {
            memcpy(buffer, &s_host_vdcmd_buf[addr - I2C_VD_BUFFER_RW], len);
        }
    }
    return 0;
}

__attribute__((weak)) int i2c_write(uint8_t i2c_id, uint8_t dev, uint16_t addr,
                                    uint16_t alen, uint8_t *buffer, uint16_t len)
{
    (void)i2c_id;
    (void)dev;
    (void)alen;
    uint16_t base_addr = (uint16_t)(addr & ~I2C_VD_CHECK_ACCESS);
    if ((buffer != NULL) && (len > 0U) &&
        (base_addr >= I2C_VD_BUFFER_RW) &&
        ((uint32_t)(base_addr - I2C_VD_BUFFER_RW) + len <= sizeof(s_host_vdcmd_buf))) {
        memcpy(&s_host_vdcmd_buf[base_addr - I2C_VD_BUFFER_RW], buffer, len);

        /* Khi gói lệnh kích hoạt thực thi tại 0x1D00 (bit 15 == 0), chuẩn bị dữ liệu phản hồi mẫu */
        if ((addr & I2C_VD_CHECK_ACCESS) == 0U) {
            uint8_t cmd_type = s_host_vdcmd_buf[0];
            uint8_t sub_cmd  = s_host_vdcmd_buf[1];
            uint8_t para     = s_host_vdcmd_buf[2];

            if ((cmd_type == 0x05U) && (sub_cmd == 0x84U)) {
                /* Standard Cmd: get_device_info (offset dữ liệu đọc tại sizeof(vdcmd_std_header_t) = 8) */
                const char *info_str = "TINY1C-ASIC2121W";
                if (para == 1U)      info_str = "2025-05-20 14:30:00";
                else if (para == 5U) info_str = "FW-V3.9.0-RELEASE";
                else if (para == 6U) info_str = "PN-TINY1C-256192";
                else if (para == 7U) info_str = "SN-SG2002-000188";
                memset(&s_host_vdcmd_buf[8], 0, 64U);
                memcpy(&s_host_vdcmd_buf[8], info_str, strlen(info_str));
            } else if ((cmd_type == 0x09U) && (sub_cmd == 0xC4U)) {
                /* Standard Cmd: pseudo_color_set */
                s_host_sim_palette = s_host_vdcmd_buf[8];
            } else if ((cmd_type == 0x09U) && (sub_cmd == 0x84U)) {
                /* Standard Cmd: pseudo_color_get */
                s_host_vdcmd_buf[8] = s_host_sim_palette;
            } else if (cmd_type == 0x11U) {
                /* Long Cmd: TPD temperature queries (offset dữ liệu đọc tại sizeof(vdcmd_long_header_t) = 16) */
                uint8_t *resp = &s_host_vdcmd_buf[16];
                uint16_t raw_max = (uint16_t)((36.5f + 273.15f) * 16.0f); /* 4954 -> 36.48°C */
                uint16_t raw_min = (uint16_t)((24.2f + 273.15f) * 16.0f); /* 4757 -> 24.16°C */
                uint16_t raw_avg = (uint16_t)((29.8f + 273.15f) * 16.0f); /* 4847 -> 29.79°C */
                if ((sub_cmd == 0x81U) || (sub_cmd == 0x86U)) {
                    /* Max temp hoặc Point temp (Big-Endian 16-bit) */
                    resp[0] = (uint8_t)(raw_max >> 8U);
                    resp[1] = (uint8_t)(raw_max & 0xFFU);
                } else if (sub_cmd == 0x82U) {
                    /* Min temp */
                    resp[0] = (uint8_t)(raw_min >> 8U);
                    resp[1] = (uint8_t)(raw_min & 0xFFU);
                } else if (sub_cmd == 0x83U) {
                    /* Max + Min temp info (12 bytes Big-Endian, 1-based coords) */
                    resp[0]  = (uint8_t)(raw_max >> 8U); resp[1]  = (uint8_t)(raw_max & 0xFFU);
                    resp[2]  = (uint8_t)(raw_min >> 8U); resp[3]  = (uint8_t)(raw_min & 0xFFU);
                    resp[4]  = 0U; resp[5]  = 129U; /* max_x (1-based) = 129 -> 0-based = 128 */
                    resp[6]  = 0U; resp[7]  = 97U;  /* max_y (1-based) = 97  -> 0-based = 96 */
                    resp[8]  = 0U; resp[9]  = 17U;  /* min_x (1-based) = 17  -> 0-based = 16 */
                    resp[10] = 0U; resp[11] = 21U;  /* min_y (1-based) = 21  -> 0-based = 20 */
                } else if ((sub_cmd == 0x89U) || (sub_cmd == 0x8CU)) {
                    /* Line / Rect temp info (14 bytes Big-Endian) */
                    resp[0]  = (uint8_t)(raw_avg >> 8U); resp[1]  = (uint8_t)(raw_avg & 0xFFU);
                    resp[2]  = (uint8_t)(raw_max >> 8U); resp[3]  = (uint8_t)(raw_max & 0xFFU);
                    resp[4]  = (uint8_t)(raw_min >> 8U); resp[5]  = (uint8_t)(raw_min & 0xFFU);
                    resp[6]  = 0U; resp[7]  = 141U; /* max_x -> 140 */
                    resp[8]  = 0U; resp[9]  = 101U; /* max_y -> 100 */
                    resp[10] = 0U; resp[11] = 61U;  /* min_x -> 60 */
                    resp[12] = 0U; resp[13] = 56U;  /* min_y -> 55 */
                }
            }
        }
    }
    return 0;
}
#endif

static int t3_i2c_read_reg16(uint16_t dev_7bit, uint16_t reg_addr, uint8_t *buf, uint16_t len)
{
    s_i2c_diag.reg = reg_addr;
    if (!s_i2c_hw_ready) {
        if (data_rw_i2c_hw_init(s_i2c_bus) != IR_SUCCESS) {
            return -1;
        }
    }

#if defined(TINY1C_HAS_T2_SOFT_I2C) && defined(__riscv)
    if (s_bound_i2c_bus == NULL) {
        return -1;
    }

    for (uint32_t attempt = 0U; attempt < 3U; ++attempt) {
        bb_status_t st = soft_i2c_read_reg16(s_bound_i2c_bus,
                                             dev_7bit,
                                             reg_addr,
                                             buf,
                                             (size_t)len,
                                             THERMAL_I2C_TIMEOUT_MS);
        s_i2c_diag.bb_status = st;
        s_i2c_diag.nack_phase = (st == BB_NACK) ? s_bound_i2c_bus->master.nack_phase : 0xffU;
        if (st == BB_OK) {
            return 0;
        }
        printf("[T3-I2C] READ addr=0x%x reg=0x%x attempt=%d bb_status=%d cleanup=%d\n",
               (int)dev_7bit, (int)reg_addr, (int)(attempt + 1U), (int)st,
               (int)s_bound_i2c_bus->master.cleanup);
        bb_status_t recover_st = soft_i2c_recover(s_bound_i2c_bus, THERMAL_I2C_TIMEOUT_MS);
        if (recover_st != BB_OK)
            printf("[T3-I2C] Recovery after READ failed: bb_status=%d\n", (int)recover_st);
        delay_ms(1U << attempt);
    }
    return -1;
#else
    return i2c_read(s_i2c_bus, (uint8_t)dev_7bit, reg_addr, 2U, buf, len);
#endif
}

static int t3_i2c_write_reg16(uint16_t dev_7bit, uint16_t reg_addr, const uint8_t *buf, uint16_t len)
{
    s_i2c_diag.reg = reg_addr;
    if (!s_i2c_hw_ready) {
        if (data_rw_i2c_hw_init(s_i2c_bus) != IR_SUCCESS) {
            return -1;
        }
    }

#if defined(TINY1C_HAS_T2_SOFT_I2C) && defined(__riscv)
    if (s_bound_i2c_bus == NULL) {
        return -1;
    }

    /*
     * TUÂN THỦ NGHIÊM NGẶT HỢP ĐỒNG TẦNG 1 (hal_i2c_bitbang.c:3) & TẦNG 2 (soft_i2c.h:20):
     * - KHÔNG ĐƯỢC retry mù (blind retry) đối với giao dịch ghi kích hoạt lệnh tại 0x1D00
     *   (khi bit 15 I2C_VD_CHECK_ACCESS == 0), vì ASIC có thể đã bắt đầu thực thi lệnh trước
     *   khi bit ACK cuối bị nhiễu.
     * - Chỉ cho phép retry tối đa 3 lần khi nạp dữ liệu vào bộ đệm giữ HLD 0x9D00
     *   ((reg_addr & I2C_VD_CHECK_ACCESS) != 0U).
     */
    const uint32_t max_attempts = ((reg_addr & I2C_VD_CHECK_ACCESS) != 0U) ? 3U : 1U;

    for (uint32_t attempt = 0U; attempt < max_attempts; ++attempt) {
        bb_status_t st;
        if ((len > 0U) && (len <= 256U) && (buf != NULL)) {
            st = soft_i2c_write_reg16(s_bound_i2c_bus,
                                      (uint8_t)dev_7bit,
                                      reg_addr,
                                      buf,
                                      (size_t)len,
                                      THERMAL_I2C_TIMEOUT_MS);
        } else {
            uint8_t frame[2U + I2C_OUT_BUFFER_MAX];
            frame[0] = (uint8_t)((reg_addr >> 8U) & 0xFFU);
            frame[1] = (uint8_t)(reg_addr & 0xFFU);
            if ((len > 0U) && (buf != NULL)) {
                memcpy(&frame[2], buf, len);
            }
            bb_msg_t msg = {
                .addr = (uint8_t)dev_7bit,
                .read = false,
                .data = frame,
                .len  = (size_t)(2U + len)
            };
            st = soft_i2c_transfer(s_bound_i2c_bus, &msg, 1U, THERMAL_I2C_TIMEOUT_MS);
        }
        s_i2c_diag.bb_status = st;
        s_i2c_diag.nack_phase = (st == BB_NACK) ? s_bound_i2c_bus->master.nack_phase : 0xffU;

        if (st == BB_OK) {
            return 0;
        }
        printf("[T3-I2C] WRITE addr=0x%x reg=0x%x attempt=%d bb_status=%d cleanup=%d\n",
               (int)dev_7bit, (int)reg_addr, (int)(attempt + 1U), (int)st,
               (int)s_bound_i2c_bus->master.cleanup);
        bb_status_t recover_st = soft_i2c_recover(s_bound_i2c_bus, THERMAL_I2C_TIMEOUT_MS);
        if (recover_st != BB_OK)
            printf("[T3-I2C] Recovery after WRITE failed: bb_status=%d\n", (int)recover_st);
        if (attempt + 1U < max_attempts) {
            delay_ms(1U << attempt);
        }
    }
    return -1;
#else
    return i2c_write(s_i2c_bus, (uint8_t)dev_7bit, reg_addr, 2U, (uint8_t *)buf, len);
#endif
}

ir_error_t i2c_data_read(uint16_t byI2CSlaveID, uint16_t wI2CRegAddr, uint16_t wLen, uint8_t *pbyData)
{
    if ((pbyData == NULL) || (wLen == 0U) || (wLen > I2C_VD_BUFFER_DATA_LEN)) {
        return IR_ERROR_PARAM;
    }

    uint16_t raw_id  = (byI2CSlaveID != 0U) ? byI2CSlaveID : s_slave_id;
    uint16_t addr_7b = get_7bit_addr(raw_id);

    if (t3_i2c_read_reg16(addr_7b, wI2CRegAddr, pbyData, wLen) != 0) {
        return IR_I2C_GET_REGISTER_FAIL;
    }
    return IR_SUCCESS;
}

static ir_error_t i2c_check_access_done(uint16_t slave_id)
{
    uint8_t status = 0xFFU;
    uint16_t raw_id  = (slave_id != 0U) ? slave_id : s_slave_id;
    uint16_t addr_7b = get_7bit_addr(raw_id);
    uint64_t start_ms = data_rw_now_ms();
    uint64_t timeout_ms = (uint64_t)s_cmd_wait_timeout_ms;

    do {
        if (t3_i2c_read_reg16(addr_7b, I2C_VD_BUFFER_STATUS, &status, 1U) != 0) {
            return IR_I2C_GET_REGISTER_FAIL;
        }
        s_i2c_diag.vdcmd_status = status;

        /* Giải mã chi tiết mã lỗi tại bits [7:2] theo đúng VDCMD/data_rw.c của SDK hãng */
        uint8_t err_code = (uint8_t)(status & VCMD_ERR_STS_BIT);
        if (err_code != VCMD_ERR_STS_SUCCESS) {
            switch (err_code) {
            case VCMD_ERR_STS_LEN_ERR:
                printf("[T3-VDCMD] Status 0x0200=0x%x: LEN_ERR\n", status);
                break;
            case VCMD_ERR_STS_UNKNOWN_CMD_ERR:
                printf("[T3-VDCMD] Status 0x0200=0x%x: UNKNOWN_CMD_ERR\n", status);
                break;
            case VCMD_ERR_STS_HW_ERR:
                printf("[T3-VDCMD] Status 0x0200=0x%x: HW_ERR\n", status);
                break;
            case VCMD_ERR_STS_UNKNOWN_SUBCMD_ERR:
                printf("[T3-VDCMD] Status 0x0200=0x%x: UNKNOWN_SUBCMD_ERR\n", status);
                break;
            case VCMD_ERR_STS_PARAM_ERR:
                printf("[T3-VDCMD] Status 0x0200=0x%x: PARAM_ERR\n", status);
                break;
            default:
                printf("[T3-VDCMD] Status 0x0200=0x%x: UNKNOWN_ERR (0x%x)\n", status, err_code);
                break;
            }
            (void)t3_i2c_read_reg16(addr_7b, I2C_VD_BUFFER_RW, s_i2c_diag.cmd_echo, 8U);
            return IR_CHECK_DONE_FAIL;
        }

        /* Kiểm tra bit 1 (VCMD_RST_STS_FAIL = 0x02): nếu lệnh thất bại thì dừng ngay */
        if ((status & VCMD_RST_STS_BIT) == VCMD_RST_STS_FAIL) {
            printf("[T3-VDCMD] Status 0x0200=0x%x: VCMD_RST_STS_FAIL\n", status);
            return IR_CHECK_DONE_FAIL;
        }

        /* Nếu bit 0 (BUSY) == 0 và bit 1 (RST) == 0 -> ASIC đã thực thi lệnh thành công */
        if ((status & (VCMD_RST_STS_BIT | VCMD_BUSY_STS_BIT)) ==
            (VCMD_BUSY_STS_IDLE | VCMD_RST_STS_PASS)) {
            return IR_SUCCESS;
        }

        delay_ms(5U);
    } while ((data_rw_now_ms() - start_ms) < timeout_ms);

    printf("[T3-VDCMD] Timeout (%d ms) waiting for 0x0200 IDLE (last status=0x%x)\n",
           (int)s_cmd_wait_timeout_ms, status);
    return IR_CHECK_DONE_FAIL;
}

ir_error_t i2c_data_write(uint16_t byI2CSlaveID, uint16_t wI2CRegAddr, uint16_t wLen, uint8_t *pbyData)
{
    if ((wLen > I2C_OUT_BUFFER_MAX) || ((wLen > 0U) && (pbyData == NULL))) {
        return IR_ERROR_PARAM;
    }

    uint16_t raw_id  = (byI2CSlaveID != 0U) ? byI2CSlaveID : s_slave_id;
    uint16_t addr_7b = get_7bit_addr(raw_id);

    if (t3_i2c_write_reg16(addr_7b, wI2CRegAddr, pbyData, wLen) != 0) {
        return IR_I2C_SET_REGISTER_FAIL;
    }

    /* Nếu bit 15 (0x8000 - I2C_VD_BUFFER_HLD) bật, chỉ nạp dữ liệu giữ, chưa cần poll 0x0200 */
    if ((wI2CRegAddr & I2C_VD_CHECK_ACCESS) != 0U) {
        return IR_SUCCESS;
    }
    return i2c_check_access_done(raw_id);
}
