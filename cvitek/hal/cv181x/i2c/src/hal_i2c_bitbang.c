/* T1 tạo cạnh SCL/SDA; T0 quyết định cách truy cập GPIO và timebase.
 * Deadline tính bằng thời gian thực, gồm clock stretching và bị preempt.
 * Không retry tự động: lệnh camera có thể đã thực thi trước khi mất ACK. */
#include "hal_i2c_bitbang.h"
static void line(bb_master_t *b, bool scl, bool low) { b->io.drive(b->io.ctx,scl,low); }
static bool level(bb_master_t *b, bool scl) { return b->io.sample(b->io.ctx,scl); }
static uint64_t now(bb_master_t *b) { return b->io.now_us(b->io.ctx); }
static void pause_bit(bb_master_t *b) { b->io.delay_us(b->io.ctx,b->half_us); }
static bb_status_t high(bb_master_t *b)
{
    uint64_t start=now(b);
    line(b,true,false); /* Nhả SCL; slave có thể tiếp tục kéo thấp. */
    while (!level(b,true)) {
        if (now(b)>=b->deadline || now(b)-start>=b->stretch_us) return BB_TIMEOUT;
        b->io.delay_us(b->io.ctx,1);
    }
    if (now(b)>=b->deadline) return BB_TIMEOUT;
    pause_bit(b);
    return now(b)>=b->deadline ? BB_TIMEOUT : BB_OK;
}
static bb_status_t start(bb_master_t *b)
{
    line(b,false,false); pause_bit(b);
    bb_status_t r=high(b); if(r) return r;
    if(!level(b,false)) return BB_BUSY;
    line(b,false,true); pause_bit(b); line(b,true,true);
    return BB_OK;
}
static bb_status_t stop(bb_master_t *b)
{
    line(b,true,true); line(b,false,true); pause_bit(b);
    bb_status_t r=high(b);
    line(b,false,false); pause_bit(b);
    if(r) return r;
    return level(b,false) ? BB_OK : BB_BUSY;
}
static bb_status_t send_bit(bb_master_t *b, bool bit)
{
    line(b,false,!bit); pause_bit(b);
    bb_status_t r=high(b); if(r) return r;
    /* Phát 1 nhưng đọc 0: mất quyền bus. Nhả dây, không phát STOP. */
    if(bit && !level(b,false)) return BB_ARBITRATION;
    line(b,true,true); return BB_OK;
}
static bb_status_t recv_bit(bb_master_t *b, bool *bit)
{
    line(b,false,false); pause_bit(b);
    bb_status_t r=high(b); if(r) return r;
    *bit=level(b,false); line(b,true,true); return BB_OK;
}
static bb_status_t write_byte(bb_master_t *b, uint8_t v)
{
    for(unsigned i=0;i<8;i++) { bb_status_t r=send_bit(b,(v&0x80)!=0); if(r)return r; v<<=1; }
    bool nack; bb_status_t r=recv_bit(b,&nack);
    return r ? r : nack ? BB_NACK : BB_OK;
}
static bb_status_t read_byte(bb_master_t *b, uint8_t *v, bool last)
{
    *v=0;
    for(unsigned i=0;i<8;i++) { bool bit; bb_status_t r=recv_bit(b,&bit); if(r)return r; *v=(uint8_t)((*v<<1)|bit); }
    return send_bit(b,last); /* ACK=0; byte cuối NACK=1. */
}
bb_status_t bb_init(bb_master_t *b,const bb_io_t *io,uint32_t hz,uint32_t stretch)
{
    if(!b||!io||!io->drive||!io->sample||!io->now_us||!io->delay_us||
       hz<1000||hz>100000||!stretch) return BB_INVALID;
    /* Bản đầu giới hạn Standard-mode <=100kHz. Làm tròn lên để không vượt yêu cầu. */
    *b=(bb_master_t){.io=*io,.half_us=(500000U+hz-1)/hz,.stretch_us=stretch};
    line(b,true,false); line(b,false,false); return BB_OK;
}
bb_status_t bb_transfer(bb_master_t *b,bb_msg_t *m,size_t count,uint32_t timeout)
{
    if(!b||!b->io.drive||!m||!count||!timeout) return BB_INVALID;
    for(size_t i=0;i<count;i++)
        if(m[i].addr>0x7f || (m[i].len&&!m[i].data) || (m[i].read&&!m[i].len)) return BB_INVALID;
    b->deadline=now(b)+timeout; b->cleanup=BB_OK;
    bb_status_t r=BB_OK;
    for(size_t i=0;i<count&&!r;i++) {
        r=start(b); if(r)break;
        r=write_byte(b,(uint8_t)((m[i].addr<<1)|m[i].read));
        for(size_t j=0;j<m[i].len&&!r;j++)
            r=m[i].read ? read_byte(b,&m[i].data[j],j+1==m[i].len) : write_byte(b,m[i].data[j]);
    }
    /* Không cướp bus đang bị giữ hoặc sau arbitration loss. Timeout nhả dây;
     * recovery là quyết định riêng của tầng trên, không kéo dài deadline. */
    if(r==BB_OK||r==BB_NACK) { b->cleanup=stop(b); if(!r)r=b->cleanup; }
    line(b,true,false); line(b,false,false);
    return r;
}
bb_status_t bb_recover(bb_master_t *b,uint32_t timeout)
{
    if(!b||!b->io.drive||!timeout)return BB_INVALID;
    b->deadline=now(b)+timeout; line(b,false,false);
    bb_status_t r=BB_OK;
    for(unsigned i=0;i<9&&!level(b,false);i++) {
        line(b,true,true); pause_bit(b); r=high(b); if(r)break;
    }
    if(!r)r=stop(b);
    line(b,true,false); line(b,false,false); b->cleanup=r; return r;
}
