/* T0: giả lập open-drain bằng DDR. DR của A15/A27 luôn bằng 0.
 * Runtime chỉ đổi DDR; không ghi mức 1 ra chân. Pull-up ngoài tạo mức cao.
 * Critical section bảo vệ RMW giữa task/ISR cùng core, KHÔNG khóa Linux. */
#include "hal_i2c_gpio_platform.h"
#include "FreeRTOS.h"
#include "task.h"
#include "mmio.h"
#define GPIOA_DR 0x03020000UL
#define GPIOA_DDR 0x03020004UL
#define GPIOA_EXT 0x03020050UL
#define MASK ((1U<<15)|(1U<<27))
static uint32_t tick_hz;
static bool claimed;
static uint64_t ticks(void) { uint64_t t; __asm__ volatile("rdtime %0":"=r"(t)); return t; }
static uint64_t time_us(void *p)
{
    (void)p; uint64_t t=ticks();
    return (t/tick_hz)*1000000ULL+(t%tick_hz)*1000000ULL/tick_hz;
}
static void delay_us(void *p,uint32_t us)
{
    uint64_t begin=time_us(p); while(time_us(p)-begin<us) { __asm__ volatile("nop"); }
}
static void drive(void *p,bool scl,bool low)
{
    (void)p; uint32_t bit=1U<<(scl?15:27);
    taskENTER_CRITICAL();
    uint32_t d=mmio_read_32(GPIOA_DDR);
    mmio_write_32(GPIOA_DDR,low ? d|bit : d&~bit);
    taskEXIT_CRITICAL();
}
static bool sample(void *p,bool scl)
{ (void)p; return (mmio_read_32(GPIOA_EXT)&(1U<<(scl?15:27)))!=0; }
bb_status_t bb_gpio_platform_init(bb_io_t *io,uint32_t hz,bool owned)
{
    if(!io||!hz)return BB_INVALID;
    if(!owned||claimed)return BB_NOT_READY;
    tick_hz=hz;
    taskENTER_CRITICAL();
    mmio_write_32(0x03002000UL,mmio_read_32(0x03002000UL)|(1U<<29));
    /* Nhả hướng trước, preload latch=0 rồi mới mux sang GPIO: tránh xung cao chủ động. */
    mmio_write_32(GPIOA_DDR,mmio_read_32(GPIOA_DDR)&~MASK);
    mmio_write_32(GPIOA_DR,mmio_read_32(GPIOA_DR)&~MASK);
    /* SPK_EN/A15 và EMMC_DAT3/A27 đều có function GPIO=3. */
    mmio_write_32(0x0300103CUL,(mmio_read_32(0x0300103CUL)&~7U)|3U);
    mmio_write_32(0x03001058UL,(mmio_read_32(0x03001058UL)&~7U)|3U);
    taskEXIT_CRITICAL();
    if((mmio_read_32(0x0300103CUL)&7U)!=3U ||
       (mmio_read_32(0x03001058UL)&7U)!=3U ||
       (mmio_read_32(GPIOA_DDR)&MASK)) return BB_NOT_READY;
    claimed=true;
    *io=(bb_io_t){.drive=drive,.sample=sample,.now_us=time_us,.delay_us=delay_us};
    return BB_OK;
}
