/* T0 board SG2002: A15=SCL, A27=SDA. Không dùng D-PHY hoặc IIC4. */
#ifndef HAL_I2C_GPIO_PLATFORM_H
#define HAL_I2C_GPIO_PLATFORM_H
#include "hal_i2c_bitbang.h"
/* Chỉ gọi một lần lúc boot, trước worker GPIO. Caller xác nhận quyền truy cập
 * GPIOA DR/DDR liên core: tắt i2c5 Linux chưa đủ nếu Linux đổi chân khác cùng bank.
 * timer_hz là tần số rdtime thật của firmware (không phải CPU frequency). */
bb_status_t bb_gpio_platform_init(bb_io_t *, uint32_t timer_hz, bool ownership_confirmed);
#endif
