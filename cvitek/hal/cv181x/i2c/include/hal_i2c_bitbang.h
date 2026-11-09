/* T1: I2C master thuần phần mềm. Không phụ thuộc FreeRTOS hoặc thanh ghi SoC.
 * Hook T0 phải mô phỏng open-drain: low=true kéo thấp, false nhả (Hi-Z).
 * Caller giữ khóa bus suốt transfer/recover. Không gọi từ ISR. */
#ifndef HAL_I2C_BITBANG_H
#define HAL_I2C_BITBANG_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
typedef enum { BB_OK=0, BB_INVALID=-1, BB_TIMEOUT=-2, BB_NACK=-3,
    BB_BUSY=-4, BB_ARBITRATION=-5, BB_NOT_READY=-6 } bb_status_t;
typedef struct {
    void *ctx;
    void (*drive)(void *, bool scl, bool low);
    bool (*sample)(void *, bool scl);
    uint64_t (*now_us)(void *);
    void (*delay_us)(void *, uint32_t);
} bb_io_t;
/* Mỗi message bắt đầu bằng START/repeated START và địa chỉ + R/W.
 * STOP chỉ ở cuối mảng. Địa chỉ 7 bit, không truyền 0x78 cho Tiny1C. */
typedef struct { uint8_t addr; bool read; uint8_t *data; size_t len; } bb_msg_t;
typedef struct {
    bb_io_t io;
    uint32_t half_us, stretch_us;
    uint64_t deadline;
    bb_status_t cleanup; /* Lỗi STOP giữ riêng, không che lỗi gốc. */
} bb_master_t;
bb_status_t bb_init(bb_master_t *, const bb_io_t *, uint32_t hz, uint32_t stretch_us);
bb_status_t bb_transfer(bb_master_t *, bb_msg_t *, size_t count, uint32_t timeout_us);
/* Recovery chủ động: tối đa 9 clock rồi STOP. Chỉ dùng bus độc quyền. */
bb_status_t bb_recover(bb_master_t *, uint32_t timeout_us);
#endif
