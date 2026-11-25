/* Test hành vi dây qua hook T0 giả: không cần phần cứng hoặc scheduler. */
#include "soft_i2c.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
typedef struct { bool scl_low,sda_low,stuck; bool reads[1024]; size_t n,pos;
    uint64_t us; unsigned starts,stops,stretch_reads; } wire_t;
static void drive(void *p,bool scl,bool low) {
    wire_t *w=p;
    if(scl)w->scl_low=low;
    else { if(!w->scl_low&&low!=w->sda_low) { if(low)w->starts++;else w->stops++; } w->sda_low=low; }
}
static bool sample(void *p,bool scl) {
    wire_t *w=p; if(scl){if(w->stretch_reads){w->stretch_reads--;return false;}return !w->scl_low&&!w->stuck;}
    if(w->sda_low)return false;
    assert(w->pos<w->n); return w->reads[w->pos++];
}
static uint64_t now(void *p){return ((wire_t*)p)->us;}
static void delay(void *p,uint32_t us){((wire_t*)p)->us+=us;}
static void push(wire_t *w,bool bit){w->reads[w->n++]=bit;}
static void sent(wire_t *w,uint8_t v,bool nack){for(int i=7;i>=0;i--)if(v&(1<<i))push(w,true);push(w,nack);}
static void received(wire_t *w,uint8_t v,bool last){for(int i=7;i>=0;i--)push(w,(v&(1<<i))!=0);if(last)push(w,true);}
static void init(wire_t *w,soft_i2c_bus_t *b){memset(w,0,sizeof *w);memset(b,0,sizeof *b);
    bb_io_t io={w,drive,sample,now,delay};assert(soft_i2c_init(b,&io,100000,50)==BB_OK);}
SemaphoreHandle_t xSemaphoreCreateMutexStatic(StaticSemaphore_t *s){s->locked=0;return s;}
int xSemaphoreTake(SemaphoreHandle_t s,TickType_t t){(void)t;if(s->locked)return 0;s->locked=1;return 1;}
int xSemaphoreGive(SemaphoreHandle_t s){s->locked=0;return 1;}
int main(void){wire_t w;soft_i2c_bus_t b;uint8_t data[2];
    init(&w,&b);
    push(&w,true);sent(&w,0x78,false);sent(&w,2,false);sent(&w,0,false);
    push(&w,true);sent(&w,0x79,false);received(&w,0xa5,false);received(&w,0x5a,true);push(&w,true);
    assert(soft_i2c_read_reg16(&b,0x3c,0x0200,data,2,20)==BB_OK);
    assert(data[0]==0xa5&&data[1]==0x5a&&w.starts==2&&w.stops==1&&w.pos==w.n);
    init(&w,&b);push(&w,true);sent(&w,0x78,false);push(&w,true);
    bb_msg_t address_only={0x3c,false,NULL,0};
    assert(soft_i2c_transfer(&b,&address_only,1,20)==BB_OK&&w.stops==1);
    init(&w,&b);push(&w,true);sent(&w,0x78,true);push(&w,true);
    assert(soft_i2c_write_reg16(&b,0x3c,0x0200,data,2,20)==BB_NACK);
    assert(b.master.nack_phase==0); /* NACK ngay byte địa chỉ. */
    assert(w.stops==1&&!w.scl_low&&!w.sda_low&&!b.lock->locked);
    init(&w,&b);w.stretch_reads=5;push(&w,true);sent(&w,0x78,false);
    sent(&w,2,false);sent(&w,0,false);sent(&w,0xa5,false);sent(&w,0x5a,false);push(&w,true);
    assert(soft_i2c_write_reg16(&b,0x3c,0x0200,data,2,20)==BB_OK&&w.pos==w.n);
    init(&w,&b);push(&w,true);sent(&w,0x78,false);sent(&w,2,true);push(&w,true);
    assert(soft_i2c_write_reg16(&b,0x3c,0x0200,data,2,20)==BB_NACK&&w.stops==1);
    assert(b.master.nack_phase==1); /* NACK tại byte địa chỉ thanh ghi cao. */
    init(&w,&b);b.master.half_us=500;push(&w,true);
    assert(soft_i2c_read_reg16(&b,0x3c,0,data,2,1)==BB_TIMEOUT);
    init(&w,&b);w.stuck=true;
    assert(soft_i2c_read_reg16(&b,0x3c,0x0200,data,2,20)==BB_TIMEOUT);
    assert(w.us<100&&!w.scl_low&&!w.sda_low);
    init(&w,&b);push(&w,true);push(&w,false); /* Mất arbitration ở bit 1 đầu địa chỉ. */
    assert(soft_i2c_read_reg16(&b,0x3c,0x0200,data,2,20)==BB_ARBITRATION);
    assert(!w.scl_low&&!w.sda_low);
    init(&w,&b);push(&w,false);assert(soft_i2c_read_reg16(&b,0x3c,0,data,2,20)==BB_BUSY);
    init(&w,&b);b.lock->locked=1;assert(soft_i2c_read_reg16(&b,0x3c,0,data,2,20)==BB_BUSY);
    init(&w,&b);for(int i=0;i<9;i++)push(&w,false);push(&w,true);
    assert(soft_i2c_recover(&b,20)==BB_OK);
    init(&w,&b);for(int i=0;i<10;i++)push(&w,false);
    assert(soft_i2c_recover(&b,20)==BB_BUSY);
    init(&w,&b);bb_msg_t invalid={0x80,false,data,2};
    assert(soft_i2c_transfer(&b,&invalid,1,20)==BB_INVALID&&w.starts==0);
    assert(soft_i2c_write_reg16(&b,0x3c,0,data,257,20)==BB_INVALID);
    puts("I2C: combined read, final NACK, address NACK, stretching timeout, arbitration, busy, mutex, recovery, validation PASS");
}
