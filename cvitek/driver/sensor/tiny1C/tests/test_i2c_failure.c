/*
 * Kịch bản kiểm thử lỗi đường truyền I2C (test_i2c_failure.c):
 *   - Ghi đè các hàm `i2c_read` / `i2c_write` trả về lỗi (-1) để mô phỏng trường hợp
 *     cảm biến Tiny1C bị ngắt kết nối hoặc đứt cáp I2C5.
 *   - Xác minh `core_init` phát hiện lỗi và trả về `IR_NOT_GET_DEVICE_INFO`, cờ `probed == false`,
 *     đồng thời chặn không cho `core_start` khởi chạy luồng khi chưa nhận diện được thiết bị.
 */
#include "core.h"
#include "data_rw.h"
#include <assert.h>
#include <stdio.h>
int i2c_read(uint8_t a,uint8_t b,uint16_t c,uint16_t d,uint8_t *e,uint16_t f)
{(void)a;(void)b;(void)c;(void)d;(void)e;(void)f;return -1;}
int i2c_write(uint8_t a,uint8_t b,uint16_t c,uint16_t d,uint8_t *e,uint16_t f)
{(void)a;(void)b;(void)c;(void)d;(void)e;(void)f;return -1;}
int main(void) {
    data_rw_set_gpioa_ownership(true);
    assert(core_init(5,0x78) == IR_NOT_GET_DEVICE_INFO);
    assert(!core_get_device_info()->probed);
    assert(core_start(MODE_TEMP,25) == IR_NOT_GET_DEVICE_INFO);
    puts("disconnected camera rejected PASS");
}
