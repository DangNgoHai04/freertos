/*
 * Kịch bản kiểm thử giao thức VoSPI & Tiêm lỗi phần cứng (test_vospi.c):
 *   1. inject = 1: Giả lập lỗi đường truyền SPI khi đọc gói Header 512-byte -> kiểm tra
 *      tăng bộ đếm `header_err` và tự động gọi `hal_spi_recover_bus()`.
 *   2. inject = 2: Giả lập gói Header 512-byte có bit `frame_valid == 0` -> kiểm tra
 *      tăng bộ đếm `frames_invalid` và dừng ngay không đọc gói Payload 96 KB.
 *   3. inject = 3: Giả lập lỗi đường truyền SPI khi đọc gói Payload 96 KB -> kiểm tra
 *      tăng bộ đếm `payload_err` và tự động gọi `hal_spi_recover_bus()`.
 *   4. inject = 0 (MODE_DUAL 192 KB): Kiểm tra thu liên tiếp 4 giao dịch SPI (Header 0xAA ->
 *      Payload 96KB ảnh -> Header 0xCC -> Payload 96KB nhiệt) với byte điền `0x55` chuẩn xác.
 */
#include "vospi.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static int call, inject, recovered;
static uint8_t bytes[DUAL_FRAME_BYTES];
void spi0_dsi_hw_init(void) {}
void spi0_dsi_configure_vospi(uint16_t a,bool b) {(void)a;(void)b;}
void spi0_dsi_cs_set(int a,bool b) {(void)a;(void)b;}
int spi0_dsi_transfer_burst(const uint8_t*a,uint8_t*b,uint32_t c) {(void)a;(void)b;(void)c;return 0;}
int hal_spi_set_freq_hz(uint32_t hz) {(void)hz;return 0;}
uint32_t hal_spi_get_freq_hz(void) {return 23437500;}
void delay_ms(uint32_t ms) { assert(ms == 1); }
int hal_spi_recover_bus(void) {++recovered;return 0;}
int hal_spi_transfer_frame(const void *tx,void *rx,size_t n,uint16_t fill,uint32_t timeout) {
    (void)timeout; ++call;
    if (n == 512) {
        assert(tx && fill == 0);
        assert(((const uint8_t*)tx)[0] == (call == 3 ? 0xcc : 0xaa));
        if (inject == 1) return -1;
        memset(rx,inject == 5 ? 0xff : 0,n);
        if (inject != 5) ((uint8_t*)rx)[64] = inject == 2 ? 0 : 1;
    } else {
        assert(n == FRAME_BYTES && tx == NULL && fill == 0x55);
        if (inject == 3) return -1;
        memset(rx,inject == 4 ? 0xff : 0x12,n);
    }
    return 0;
}
int main(void) {
    assert(spi_stream_init(0,0) == 0);
    for (inject=1; inject<=3; ++inject) {
        call=0;
        assert(spi_get_frame(bytes,FRAME_BYTES,0xaa) < 0);
        assert(call == (inject == 3 ? 2 : 1));
    }
    assert(recovered == 2);
    inject=5; call=0;
    assert(spi_get_frame(bytes,FRAME_BYTES,0xaa) == -2 && call == 1);
    inject=4; call=0;
    assert(spi_get_frame(bytes,FRAME_BYTES,0xaa) == -3 && call == 2);
    inject=0; call=0;
    assert(spi_get_frame(bytes,DUAL_FRAME_BYTES,0xaa) == 0 && call == 4);
    vospi_stats_t stats; spi_get_vospi_stats(&stats);
    assert(stats.frames_invalid == 1 && stats.header_err == 2 && stats.payload_err == 2);
    puts("VoSPI AA/CC + invalid/header/payload error tests PASS");
}
