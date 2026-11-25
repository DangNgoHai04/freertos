/*
 * Tầng 3 (T3 - Tiny1C Vendor Command Engine): Thực thi các giao dịch Short,
 * Standard và Long VDCMD với khóa thiết bị tĩnh (StaticSemaphore_t) ở Tầng 3.
 *
 * Tuân thủ hợp đồng từ Tầng 2 (cvitek/driver/i2c/src/soft_i2c.c):
 *   "Một VDCMD gồm nhiều transfer cần mutex thiết bị ở T3 để chuỗi
 *    submit -> poll -> response không bị xen lệnh."
 */

#include <stdio.h>
#include "vdcmd.h"

#if defined(__riscv) && defined(__has_include) && __has_include("FreeRTOS.h")
#define TINY1C_VDCMD_HAS_FREERTOS 1
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#elif !defined(__riscv)
#define TINY1C_VDCMD_HAS_PTHREAD 1
#include <pthread.h>
#endif

static vdcmd_rw_handle_t vdcmd_read_handle  = { 0 };
static vdcmd_rw_handle_t vdcmd_write_handle = { 0 };

#if defined(TINY1C_VDCMD_HAS_FREERTOS)
static StaticSemaphore_t s_vdcmd_dev_mutex_buf;
static SemaphoreHandle_t s_vdcmd_dev_mutex = NULL;
#elif defined(TINY1C_VDCMD_HAS_PTHREAD)
static pthread_mutex_t s_vdcmd_dev_mutex = PTHREAD_MUTEX_INITIALIZER;
#endif

static void vdcmd_dev_lock_init(void)
{
#if defined(TINY1C_VDCMD_HAS_FREERTOS)
    if (s_vdcmd_dev_mutex == NULL) {
        s_vdcmd_dev_mutex = xSemaphoreCreateMutexStatic(&s_vdcmd_dev_mutex_buf);
    }
#endif
}

static bool vdcmd_dev_lock(void)
{
#if defined(TINY1C_VDCMD_HAS_FREERTOS)
    if (s_vdcmd_dev_mutex == NULL) {
        vdcmd_dev_lock_init();
    }
    if (s_vdcmd_dev_mutex == NULL) {
        return false;
    }
    return (xSemaphoreTake(s_vdcmd_dev_mutex,
                           pdMS_TO_TICKS(THERMAL_VDCMD_MUTEX_TIMEOUT_MS)) == pdTRUE);
#elif defined(TINY1C_VDCMD_HAS_PTHREAD)
    return (pthread_mutex_lock(&s_vdcmd_dev_mutex) == 0);
#else
    return true;
#endif
}

static void vdcmd_dev_unlock(void)
{
#if defined(TINY1C_VDCMD_HAS_FREERTOS)
    if (s_vdcmd_dev_mutex != NULL) {
        (void)xSemaphoreGive(s_vdcmd_dev_mutex);
    }
#elif defined(TINY1C_VDCMD_HAS_PTHREAD)
    (void)pthread_mutex_unlock(&s_vdcmd_dev_mutex);
#endif
}

ir_error_t vdcmd_rw_handle_register(enum product_types product_type, enum driver_types driver_type)
{
    (void)product_type;
    vdcmd_dev_lock_init();

    switch (driver_type) {
    case I2C_VDCMD:
        vdcmd_read_handle.handle    = i2c_data_read;
        vdcmd_read_handle.slave_id  = data_rw_get_slave_addr();
        vdcmd_write_handle.handle   = i2c_data_write;
        vdcmd_write_handle.slave_id = data_rw_get_slave_addr();
        return IR_SUCCESS;
    case UART_VDCMD:
    default:
        return IR_VDCMD_REGISTER_FAIL;
    }
}

ir_error_t short_cmd_read(vdcmd_short_header_t *ptVdCmdShortHeader, uint8_t byLen, uint8_t *pbyData)
{
    const uint16_t byMaxBufferLen = 64U;

    if ((ptVdCmdShortHeader == NULL) || ((byLen > 0U) && (pbyData == NULL))) {
        return IR_ERROR_PARAM;
    }
    if ((vdcmd_write_handle.handle == NULL) || (vdcmd_read_handle.handle == NULL)) {
        return IR_VDCMD_NOT_REGISTER;
    }
    if (!vdcmd_dev_lock()) {
        return IR_CONTROL_TRANSFER_FAIL;
    }

    ir_error_t rst = IR_SUCCESS;
    while (byLen > 0U) {
        uint16_t wTmpLen = (byLen >= (byMaxBufferLen - 1U)) ? (byMaxBufferLen - 1U) : byLen;

        rst = vdcmd_write_handle.handle(vdcmd_write_handle.slave_id,
                                        I2C_VD_BUFFER_RW,
                                        (uint16_t)sizeof(vdcmd_short_header_t),
                                        (uint8_t *)ptVdCmdShortHeader);
        if (rst != IR_SUCCESS) {
            break;
        }

        rst = vdcmd_read_handle.handle(vdcmd_read_handle.slave_id,
                                       (uint16_t)(I2C_VD_BUFFER_RW + sizeof(vdcmd_short_header_t)),
                                       wTmpLen,
                                       pbyData);
        if (rst != IR_SUCCESS) {
            break;
        }

        byLen   -= (uint8_t)wTmpLen;
        pbyData += wTmpLen;
    }

    vdcmd_dev_unlock();
    return rst;
}

ir_error_t short_cmd_write(vdcmd_short_header_t *ptVdCmdShortHeader, uint8_t byLen, uint8_t *pbyData)
{
    if (ptVdCmdShortHeader == NULL) {
        return IR_ERROR_PARAM;
    }
    if (vdcmd_write_handle.handle == NULL) {
        return IR_VDCMD_NOT_REGISTER;
    }
    if (!vdcmd_dev_lock()) {
        return IR_CONTROL_TRANSFER_FAIL;
    }

    ir_error_t rst;
    if ((byLen == 0U) || (pbyData == NULL)) {
        /* Đã sửa lỗi từ RTOS_V1: truyền đúng sizeof(vdcmd_short_header_t) = 4 byte */
        rst = vdcmd_write_handle.handle(vdcmd_write_handle.slave_id,
                                        I2C_VD_BUFFER_RW,
                                        (uint16_t)sizeof(vdcmd_short_header_t),
                                        (uint8_t *)ptVdCmdShortHeader);
        vdcmd_dev_unlock();
        return rst;
    }

    rst = vdcmd_write_handle.handle(vdcmd_write_handle.slave_id,
                                    I2C_VD_BUFFER_HLD,
                                    (uint16_t)sizeof(vdcmd_short_header_t),
                                    (uint8_t *)ptVdCmdShortHeader);
    if (rst != IR_SUCCESS) {
        vdcmd_dev_unlock();
        return rst;
    }

    if (byLen <= 4U) {
        rst = vdcmd_write_handle.handle(vdcmd_write_handle.slave_id,
                                        (uint16_t)(I2C_VD_BUFFER_RW + sizeof(vdcmd_short_header_t)),
                                        byLen,
                                        pbyData);
    } else {
        rst = vdcmd_write_handle.handle(vdcmd_write_handle.slave_id,
                                        (uint16_t)(I2C_VD_BUFFER_HLD + sizeof(vdcmd_short_header_t)),
                                        (uint16_t)(byLen - 4U),
                                        pbyData);
        if (rst == IR_SUCCESS) {
            rst = vdcmd_write_handle.handle(vdcmd_write_handle.slave_id,
                                            (uint16_t)(I2C_VD_BUFFER_RW + sizeof(vdcmd_short_header_t) + byLen - 4U),
                                            4U,
                                            pbyData + byLen - 4U);
        }
    }

    vdcmd_dev_unlock();
    return rst;
}

ir_error_t standard_cmd_read(vdcmd_std_header_t *ptVdCmdHeader, uint16_t wLen, uint8_t *pbyData)
{
    const uint16_t byMaxBufferLen = I2C_VD_BUFFER_DATA_LEN;

    if ((ptVdCmdHeader == NULL) || ((wLen > 0U) && (pbyData == NULL))) {
        return IR_ERROR_PARAM;
    }
    if ((vdcmd_write_handle.handle == NULL) || (vdcmd_read_handle.handle == NULL)) {
        return IR_VDCMD_NOT_REGISTER;
    }
    if (!vdcmd_dev_lock()) {
        return IR_CONTROL_TRANSFER_FAIL;
    }

    uint32_t dwAddr = ((uint32_t)ptVdCmdHeader->byPara << 24U) |
                      ((uint32_t)ptVdCmdHeader->byAddr_h << 16U) |
                      ((uint32_t)ptVdCmdHeader->byAddr_l << 8U) |
                      (uint32_t)ptVdCmdHeader->byAddr_ll;

    ir_error_t rst = IR_SUCCESS;
    while (wLen > 0U) {
        uint16_t wTmpLen = (wLen >= byMaxBufferLen) ? byMaxBufferLen : wLen;

        ptVdCmdHeader->byPara    = (uint8_t)(dwAddr >> 24U);
        ptVdCmdHeader->byAddr_h  = (uint8_t)(dwAddr >> 16U);
        ptVdCmdHeader->byAddr_l  = (uint8_t)(dwAddr >> 8U);
        ptVdCmdHeader->byAddr_ll = (uint8_t)dwAddr;
        ptVdCmdHeader->byLen_h   = (uint8_t)(wTmpLen >> 8U);
        ptVdCmdHeader->byLen_l   = (uint8_t)wTmpLen;

        rst = vdcmd_write_handle.handle(vdcmd_write_handle.slave_id,
                                        I2C_VD_BUFFER_RW,
                                        (uint16_t)sizeof(vdcmd_std_header_t),
                                        (uint8_t *)ptVdCmdHeader);
        if (rst != IR_SUCCESS) {
            break;
        }

        rst = vdcmd_read_handle.handle(vdcmd_read_handle.slave_id,
                                       (uint16_t)(I2C_VD_BUFFER_RW + sizeof(vdcmd_std_header_t)),
                                       wTmpLen,
                                       pbyData);
        if (rst != IR_SUCCESS) {
            break;
        }

        dwAddr  += wTmpLen;
        wLen    -= wTmpLen;
        pbyData += wTmpLen;
    }

    vdcmd_dev_unlock();
    return rst;
}

ir_error_t standard_cmd_write(vdcmd_std_header_t *ptVdCmdHeader, uint16_t wLen, uint8_t *pbyData)
{
    const uint16_t byMaxBufferLen = I2C_VD_BUFFER_DATA_LEN;

    if (ptVdCmdHeader == NULL) {
        return IR_ERROR_PARAM;
    }
    if ((vdcmd_write_handle.handle == NULL) || (vdcmd_read_handle.handle == NULL)) {
        return IR_VDCMD_NOT_REGISTER;
    }
    if (!vdcmd_dev_lock()) {
        return IR_CONTROL_TRANSFER_FAIL;
    }

    uint32_t dwAddr = ((uint32_t)ptVdCmdHeader->byPara << 24U) |
                      ((uint32_t)ptVdCmdHeader->byAddr_h << 16U) |
                      ((uint32_t)ptVdCmdHeader->byAddr_l << 8U) |
                      (uint32_t)ptVdCmdHeader->byAddr_ll;

    ir_error_t rst = IR_SUCCESS;
    do {
        if ((wLen == 0U) || (pbyData == NULL)) {
            rst = vdcmd_write_handle.handle(vdcmd_write_handle.slave_id,
                                            I2C_VD_BUFFER_RW,
                                            (uint16_t)sizeof(vdcmd_std_header_t),
                                            (uint8_t *)ptVdCmdHeader);
            break;
        }

        uint16_t wTmpLen = (wLen >= byMaxBufferLen) ? byMaxBufferLen : wLen;
        uint16_t wI2COneTransferLen = wTmpLen;

        ptVdCmdHeader->byPara    = (uint8_t)(dwAddr >> 24U);
        ptVdCmdHeader->byAddr_h  = (uint8_t)(dwAddr >> 16U);
        ptVdCmdHeader->byAddr_l  = (uint8_t)(dwAddr >> 8U);
        ptVdCmdHeader->byAddr_ll = (uint8_t)dwAddr;
        ptVdCmdHeader->byLen_h   = (uint8_t)(wI2COneTransferLen >> 8U);
        ptVdCmdHeader->byLen_l   = (uint8_t)wI2COneTransferLen;

        rst = vdcmd_write_handle.handle(vdcmd_write_handle.slave_id,
                                        I2C_VD_BUFFER_HLD,
                                        (uint16_t)sizeof(vdcmd_std_header_t),
                                        (uint8_t *)ptVdCmdHeader);
        if (rst != IR_SUCCESS) {
            break;
        }

        uint32_t dwDataOffset = 0U;
        while (wTmpLen > 0U) {
            uint8_t byOneLen;
            if (wTmpLen > I2C_OUT_BUFFER_MAX) {
                byOneLen = (uint8_t)I2C_OUT_BUFFER_MAX;
                rst = vdcmd_write_handle.handle(vdcmd_write_handle.slave_id,
                                                (uint16_t)(I2C_VD_BUFFER_HLD + dwDataOffset + sizeof(vdcmd_std_header_t)),
                                                I2C_OUT_BUFFER_MAX,
                                                pbyData);
                if (rst != IR_SUCCESS) {
                    break;
                }
            } else {
                byOneLen = (uint8_t)wTmpLen;
                if (wTmpLen <= 8U) {
                    rst = vdcmd_write_handle.handle(vdcmd_write_handle.slave_id,
                                                    (uint16_t)(I2C_VD_BUFFER_RW + dwDataOffset + sizeof(vdcmd_std_header_t)),
                                                    wTmpLen,
                                                    pbyData);
                } else {
                    rst = vdcmd_write_handle.handle(vdcmd_write_handle.slave_id,
                                                    (uint16_t)(I2C_VD_BUFFER_HLD + dwDataOffset + sizeof(vdcmd_std_header_t)),
                                                    (uint16_t)(wTmpLen - 8U),
                                                    pbyData);
                    if (rst == IR_SUCCESS) {
                        rst = vdcmd_write_handle.handle(vdcmd_write_handle.slave_id,
                                                        (uint16_t)(I2C_VD_BUFFER_RW + dwDataOffset + sizeof(vdcmd_std_header_t) + wTmpLen - 8U),
                                                        8U,
                                                        pbyData + wTmpLen - 8U);
                    }
                }
                if (rst != IR_SUCCESS) {
                    break;
                }
            }

            wTmpLen      -= byOneLen;
            pbyData      += byOneLen;
            dwDataOffset += byOneLen;
        }

        if (rst != IR_SUCCESS) {
            break;
        }

        wLen   -= wI2COneTransferLen;
        dwAddr += wI2COneTransferLen;
    } while (wLen > 0U);

    vdcmd_dev_unlock();
    return rst;
}

ir_error_t long_cmd_read(vdcmd_long_header_t *ptVdCmdHeader, uint16_t wParam, uint32_t dwAddr1,
                         uint32_t dwAddr2, uint32_t dwLen, uint8_t data[])
{
    if ((ptVdCmdHeader == NULL) || ((dwLen > 0U) && (data == NULL))) {
        return IR_ERROR_PARAM;
    }
    if (dwLen > (uint32_t)(256U + 8U - sizeof(vdcmd_long_header_t))) {
        return IR_VDCMD_TOO_LONG;
    }
    if ((vdcmd_write_handle.handle == NULL) || (vdcmd_read_handle.handle == NULL)) {
        return IR_VDCMD_NOT_REGISTER;
    }
    if (!vdcmd_dev_lock()) {
        return IR_CONTROL_TRANSFER_FAIL;
    }

    ptVdCmdHeader->byParam_h  = (uint8_t)(wParam >> 8U);
    ptVdCmdHeader->byParam_l  = (uint8_t)wParam;
    if (dwLen != 0U) {
        ptVdCmdHeader->byLen_hh = (uint8_t)(dwLen >> 24U);
        ptVdCmdHeader->byLen_h  = (uint8_t)(dwLen >> 16U);
        ptVdCmdHeader->byLen_l  = (uint8_t)(dwLen >> 8U);
        ptVdCmdHeader->byLen_ll = (uint8_t)dwLen;
    }
    ptVdCmdHeader->byAddr1_hh = (uint8_t)(dwAddr1 >> 24U);
    ptVdCmdHeader->byAddr1_h  = (uint8_t)(dwAddr1 >> 16U);
    ptVdCmdHeader->byAddr1_l  = (uint8_t)(dwAddr1 >> 8U);
    ptVdCmdHeader->byAddr1_ll = (uint8_t)dwAddr1;
    ptVdCmdHeader->byAddr2_hh = (uint8_t)(dwAddr2 >> 24U);
    ptVdCmdHeader->byAddr2_h  = (uint8_t)(dwAddr2 >> 16U);
    ptVdCmdHeader->byAddr2_l  = (uint8_t)(dwAddr2 >> 8U);
    ptVdCmdHeader->byAddr2_ll = (uint8_t)dwAddr2;

    ir_error_t rst = vdcmd_write_handle.handle(vdcmd_write_handle.slave_id,
                                               I2C_VD_BUFFER_HLD,
                                               8U,
                                               (uint8_t *)ptVdCmdHeader);
    if (rst == IR_SUCCESS) {
        rst = vdcmd_write_handle.handle(vdcmd_write_handle.slave_id,
                                        I2C_VD_BUFFER_RW + 8U,
                                        8U,
                                        ((uint8_t *)ptVdCmdHeader) + 8U);
    }
    if ((rst == IR_SUCCESS) && (dwLen > 0U)) {
        rst = vdcmd_read_handle.handle(vdcmd_read_handle.slave_id,
                                       (uint16_t)(I2C_VD_BUFFER_RW + sizeof(vdcmd_long_header_t)),
                                       (uint16_t)dwLen,
                                       data);
    }

    vdcmd_dev_unlock();
    return rst;
}

ir_error_t long_cmd_write(vdcmd_long_header_t *ptVdCmdHeader, uint16_t wParam, uint32_t dwAddr1,
                          uint32_t dwAddr2, uint32_t dwLen)
{
    if (ptVdCmdHeader == NULL) {
        return IR_ERROR_PARAM;
    }
    if ((vdcmd_write_handle.handle == NULL) || (vdcmd_read_handle.handle == NULL)) {
        return IR_VDCMD_NOT_REGISTER;
    }
    if (!vdcmd_dev_lock()) {
        return IR_CONTROL_TRANSFER_FAIL;
    }

    ptVdCmdHeader->byParam_h  = (uint8_t)(wParam >> 8U);
    ptVdCmdHeader->byParam_l  = (uint8_t)wParam;
    if (dwLen != 0U) {
        ptVdCmdHeader->byLen_hh = (uint8_t)(dwLen >> 24U);
        ptVdCmdHeader->byLen_h  = (uint8_t)(dwLen >> 16U);
        ptVdCmdHeader->byLen_l  = (uint8_t)(dwLen >> 8U);
        ptVdCmdHeader->byLen_ll = (uint8_t)dwLen;
    }
    ptVdCmdHeader->byAddr1_hh = (uint8_t)(dwAddr1 >> 24U);
    ptVdCmdHeader->byAddr1_h  = (uint8_t)(dwAddr1 >> 16U);
    ptVdCmdHeader->byAddr1_l  = (uint8_t)(dwAddr1 >> 8U);
    ptVdCmdHeader->byAddr1_ll = (uint8_t)dwAddr1;
    ptVdCmdHeader->byAddr2_hh = (uint8_t)(dwAddr2 >> 24U);
    ptVdCmdHeader->byAddr2_h  = (uint8_t)(dwAddr2 >> 16U);
    ptVdCmdHeader->byAddr2_l  = (uint8_t)(dwAddr2 >> 8U);
    ptVdCmdHeader->byAddr2_ll = (uint8_t)dwAddr2;

    ir_error_t rst = vdcmd_write_handle.handle(vdcmd_write_handle.slave_id,
                                               I2C_VD_BUFFER_HLD,
                                               8U,
                                               (uint8_t *)ptVdCmdHeader);
    if (rst == IR_SUCCESS) {
        rst = vdcmd_write_handle.handle(vdcmd_write_handle.slave_id,
                                        I2C_VD_BUFFER_RW + 8U,
                                        8U,
                                        ((uint8_t *)ptVdCmdHeader) + 8U);
    }

    vdcmd_dev_unlock();
    return rst;
}
