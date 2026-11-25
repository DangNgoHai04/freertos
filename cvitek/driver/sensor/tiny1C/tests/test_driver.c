/*
 * Kịch bản kiểm thử tích hợp Tầng 3 (test_driver.c):
 *   1. Kiểm tra khóa an toàn tài nguyên: từ chối `core_init` khi chưa xác nhận quyền sở hữu GPIOA.
 *   2. Kiểm tra khởi tạo I2C5 + đọc thông tin thiết bị Tiny1C (`core_init`, `core_get_device_info`).
 *   3. Kiểm tra khởi tạo VoSPI SPI0 (`spi_stream_init`) và bật luồng `MODE_TEMP` (25 FPS).
 *   4. Kiểm tra từ chối các tham số sai (`mode` ngoài phạm vi, `fps = 0`, con trỏ đệm `NULL`, bảng màu số 2 Reserved).
 *   5. Kiểm tra thu khung đơn 96 KB (`MODE_TEMP`) và khung kép 192 KB (`MODE_DUAL`, tự động nâng SCK lên 46.875 MHz).
 *   6. Kiểm tra dừng luồng (`core_stop`).
 */
#include "core.h"
#include "data_rw.h"
#include "vospi.h"
#include <assert.h>
#include <stdio.h>
static uint8_t pixels[DUAL_FRAME_BYTES] __attribute__((aligned(64)));
int main(void) {
    assert(core_init(5,0x78) != IR_SUCCESS); /* Chưa xác nhận quyền sở hữu GPIOA -> từ chối */
    data_rw_set_gpioa_ownership(true);
    assert(core_init(5,0x78) == IR_SUCCESS);
    assert(core_get_device_info()->probed);
    assert(spi_stream_init(0,SPI0_DSI_CS_PIN_DEFAULT) == 0);
    assert(core_start(MODE_TEMP,25) == IR_SUCCESS);
    assert(core_start((camera_mode_t)99,25) == IR_ERROR_PARAM);
    assert(core_start(MODE_IMAGE,0) == IR_ERROR_PARAM);
    assert(spi_get_frame(pixels,FRAME_BYTES,VOSPI_HEADER_IMAGE) == 0);
    assert(spi_get_frame(NULL,FRAME_BYTES,VOSPI_HEADER_IMAGE) != 0);
    assert(set_palette(2) != IR_SUCCESS);
    assert(core_start(MODE_DUAL,25) == IR_SUCCESS);
    assert(hal_spi_get_freq_hz() == THERMAL_SPI_DUAL_SCK_HZ);
    assert(spi_get_frame(pixels,DUAL_FRAME_BYTES,VOSPI_HEADER_IMAGE) == 0);
    assert(core_stop() == IR_SUCCESS);
    puts("driver success/validation tests PASS");
}
