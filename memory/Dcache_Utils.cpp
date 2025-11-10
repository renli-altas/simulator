#include "Dcache_Utils.h"
#include <cstdio>
uint32_t dcache_data[DCACHE_LINE_NUM][DCACHE_WAY_NUM][DCACHE_OFFSET_NUM] = {0};
uint32_t dcache_lru[DCACHE_LINE_NUM][DCACHE_WAY_NUM] = {0};
uint32_t dcache_tag[DCACHE_LINE_NUM][DCACHE_WAY_NUM] = {0};
bool dcache_valid[DCACHE_LINE_NUM][DCACHE_WAY_NUM] = {0};
bool dcache_dirty[DCACHE_LINE_NUM][DCACHE_WAY_NUM] = {0};

void updatelru(int linenum,int way)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           
{
    for (int i = 0; i < DCACHE_WAY_NUM; i++)
    {
        dcache_lru[linenum][i]++;
    }
    dcache_lru[linenum][way] = 0;
}
int getlru(int linenum)
{ 
    int imax = -1;
    int way = -1;
    for (int i = 0; i < DCACHE_WAY_NUM; i++)
    {
        if ((int)dcache_lru[linenum][i] > imax)
        {
            imax = dcache_lru[linenum][i];
            way = i;
        }
    }
    dcache_lru[linenum][way] = 0;
    return way;
}
uint32_t read_cache_data(uint32_t index, uint32_t way, uint32_t offset)
{
    return dcache_data[index][way][offset];
}

void write_cache_data(uint32_t index, uint32_t way, uint32_t offset, uint32_t wdata, uint8_t wstrb)
{
    uint32_t old_data = dcache_data[index][way][offset];
    uint32_t mask = 0;
    if (wstrb & 0b1)
        mask |= 0xFF;
    if (wstrb & 0b10)
        mask |= 0xFF00;
    if (wstrb & 0b100)
        mask |= 0xFF0000;
    if (wstrb & 0b1000)
        mask |= 0xFF000000;
    dcache_data[index][way][offset] = (mask & wdata) | (~mask & old_data);
    dcache_dirty[index][way] = 1;
}

bool hit_check(uint32_t index, uint32_t tag, uint32_t &hit_way)
{
    bool hit = false;
    hit_way = -1;
    for (int i = 0; i < DCACHE_WAY_NUM; i++)
    {
        if (dcache_tag[index][i] == tag && dcache_valid[index][i])
        {
            hit = true;
            hit_way = i;
        }
    }
    return hit;
}

void get_addr_info(uint32_t addr, uint32_t &tag, uint32_t &index, uint32_t &offset)
{
    tag = (addr >> 2) >> (DCACHE_INDEX_BITS + DCACHE_OFFSET_BITS);
    index = ((addr >> 2) >> DCACHE_OFFSET_BITS) & ((1 << DCACHE_INDEX_BITS) - 1);
    offset = (addr >> 2) & ((1 << DCACHE_OFFSET_BITS) - 1);
}
uint32_t get_addr(uint32_t tag, uint32_t index, uint32_t offset)
{
    return (tag << (DCACHE_INDEX_BITS + DCACHE_OFFSET_BITS) | index << DCACHE_OFFSET_BITS | offset) << 2;
}
void change_state(Dcache_State &state,bool io_req,bool hit,bool done,bool flush)
{
    if(state==DCACHE_IDLE){
        if(io_req==true&&!hit){
            state = DCACHE_WAIT;
        }
    }else if(state==DCACHE_WAIT){
        if(flush==true||done==true){
            state = DCACHE_IDLE; 
        }
    }
}
void write_cache_line(uint32_t index, uint32_t way, uint32_t& offset,uint32_t& data,bool&wdone, bool done,bool last)
{

    if(offset==0){
        wdone=true;
    }
    else if(done){
        wdone=false;
    }
    else {
        wdone=true;
    }
    if(done){
        offset++;
    }
    if(last){
        offset=0;  
        wdone=false;
    }
    data = dcache_data[index][way][offset];
}
void read_cache_line(uint32_t index, uint32_t way, uint32_t& offset,uint32_t data, bool done,bool last)
{

    if(done){
        dcache_data[index][way][offset++] = data;
    }
    if(last){
        offset=0;
    }
}
void transfer_zero(MSHR_INFO* &mshrio)
{
    mshrio->valid = false;
    mshrio->addr = 0;
    mshrio->wdata = 0;
    mshrio->wr = false;
    mshrio->wstrb = 0;
    mshrio->tag = 0;
    mshrio->index = 0;
    mshrio->offset = 0;
}
void transfer_data(MSHR_INFO* &mshrio,Mem_IO* cpu,uint32_t tag,uint32_t offset,uint32_t index,uint32_t way,bool dirty,uint32_t paddr,bool ready)
{
    mshrio->valid = ready;
    mshrio->addr = cpu->addr;
    mshrio->wdata = cpu->wdata;
    mshrio->wr = cpu->wr;
    mshrio->wstrb = cpu->wstrb;
    mshrio->tag = tag;
    mshrio->index = index;
    mshrio->offset = offset;
    mshrio->dirty=dirty;
    mshrio->way=way;
    mshrio->paddr = paddr;
}
void read_data(EXMem_IO* &mem,uint32_t addr,uint32_t offset)
{
    mem->control.en = true;
    mem->control.wen = false;
    mem->control.addr = addr;
    mem->control.wdata = 0;
    mem->control.sel = 0;
    mem->control.len = DCACHE_OFFSET_NUM - 1;
    mem->control.size = 0b10;
    mem->control.last = offset == DCACHE_OFFSET_NUM - 1;
}
void miss_deal(uint32_t index, uint32_t& hit_way, uint32_t tag,bool &dirty_writeback,uint32_t&paddr)
{
    hit_way = getlru(index);
    if(dcache_dirty[index][hit_way]){
        dirty_writeback=true;
        paddr = get_addr(dcache_tag[index][hit_way], index, 0);
    }
}