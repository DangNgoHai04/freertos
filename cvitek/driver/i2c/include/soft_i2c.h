/* T2: bus FreeRTOS có static mutex; tên riêng tránh đổi ABI i2c.h legacy.
 * Caller cấp object zero-init, giữ sống suốt vòng đời firmware, init một lần.
 * Task context duy nhất; không gọi trong ISR hoặc scheduler suspended. */
#ifndef SOFT_I2C_H
#define SOFT_I2C_H
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "hal_i2c_bitbang.h"
typedef struct {
    bb_master_t master;
    StaticSemaphore_t lock_storage;
    SemaphoreHandle_t lock;
    bool ready;
    uint32_t transfers, errors; /* Chỉ đọc khi không có worker, hoặc dưới bus lock. */
} soft_i2c_bus_t;
bb_status_t soft_i2c_init(soft_i2c_bus_t *,const bb_io_t *,uint32_t hz,uint32_t stretch_us);
/* timeout_ms gồm cả chờ mutex và giao dịch. Không tự retry NACK/lệnh camera. */
bb_status_t soft_i2c_transfer(soft_i2c_bus_t *,bb_msg_t *,size_t,uint32_t timeout_ms);
bb_status_t soft_i2c_recover(soft_i2c_bus_t *,uint32_t timeout_ms);
/* Register helper: 16 bit big-endian; repeated START giữa index và read.
 * Ghi tối đa 256 byte, buffer local nằm trên stack, không cấp heap. */
bb_status_t soft_i2c_read_reg16(soft_i2c_bus_t *,uint8_t addr,uint16_t reg,uint8_t *,size_t,uint32_t);
bb_status_t soft_i2c_write_reg16(soft_i2c_bus_t *,uint8_t addr,uint16_t reg,const uint8_t *,size_t,uint32_t);
#endif
