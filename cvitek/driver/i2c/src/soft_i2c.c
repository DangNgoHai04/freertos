/* T2 quản lý khóa; T1 lo wire protocol. Một VDCMD gồm nhiều transfer cần
 * mutex thiết bị ở T3 để chuỗi submit -> poll -> response không bị xen lệnh. */
#include "soft_i2c.h"
#include "task.h"
#include <string.h>
bb_status_t soft_i2c_init(soft_i2c_bus_t *b,const bb_io_t *io,uint32_t hz,uint32_t stretch)
{
    if(!b||b->ready)return BB_INVALID;
    bb_status_t r=bb_init(&b->master,io,hz,stretch); if(r)return r;
    b->lock=xSemaphoreCreateMutexStatic(&b->lock_storage);
    if(!b->lock)return BB_NOT_READY;
    b->transfers=b->errors=0; b->ready=true; return BB_OK;
}
static bb_status_t run(soft_i2c_bus_t *b,bb_msg_t *m,size_t n,uint32_t ms,bool recover)
{
    if(!b||!b->ready)return BB_NOT_READY;
    if(!ms||ms>60000U)return BB_INVALID;
    uint64_t begin=b->master.io.now_us(b->master.io.ctx), budget=(uint64_t)ms*1000;
    /* Làm tròn tick lên, sau đó kiểm tra deadline thực trước khi chạm dây. */
    TickType_t wait=(TickType_t)(((uint64_t)ms*configTICK_RATE_HZ+999)/1000);
    if(xSemaphoreTake(b->lock,wait)!=pdTRUE)return BB_BUSY;
    uint64_t elapsed=b->master.io.now_us(b->master.io.ctx)-begin;
    bb_status_t r=BB_TIMEOUT;
    if(elapsed<budget) r=recover ? bb_recover(&b->master,(uint32_t)(budget-elapsed)) :
        bb_transfer(&b->master,m,n,(uint32_t)(budget-elapsed));
    b->transfers++; if(r)b->errors++;
    xSemaphoreGive(b->lock); return r;
}
bb_status_t soft_i2c_transfer(soft_i2c_bus_t *b,bb_msg_t *m,size_t n,uint32_t ms)
{ return run(b,m,n,ms,false); }
bb_status_t soft_i2c_recover(soft_i2c_bus_t *b,uint32_t ms)
{ return run(b,NULL,0,ms,true); }
bb_status_t soft_i2c_read_reg16(soft_i2c_bus_t *b,uint8_t addr,uint16_t reg,uint8_t *data,size_t len,uint32_t ms)
{
    if(!data||!len)return BB_INVALID;
    uint8_t index[2]={(uint8_t)(reg>>8),(uint8_t)reg};
    bb_msg_t m[2]={{addr,false,index,2},{addr,true,data,len}};
    return soft_i2c_transfer(b,m,2,ms);
}
bb_status_t soft_i2c_write_reg16(soft_i2c_bus_t *b,uint8_t addr,uint16_t reg,const uint8_t *data,size_t len,uint32_t ms)
{
    if(!data||!len||len>256)return BB_INVALID;
    uint8_t buffer[258]={(uint8_t)(reg>>8),(uint8_t)reg};
    memcpy(buffer+2,data,len); bb_msg_t m={addr,false,buffer,len+2};
    return soft_i2c_transfer(b,&m,1,ms);
}
