#include "Dcache_Utils.h"
#include <cstdio>
uint32_t dcache_data[DCACHE_LINE_NUM][DCACHE_WAY_NUM][DCACHE_OFFSET_NUM] = {0};
uint32_t dcache_lru[DCACHE_LINE_NUM][DCACHE_WAY_NUM] = {0};
uint32_t dcache_tag[DCACHE_LINE_NUM][DCACHE_WAY_NUM] = {0};
bool dcache_valid[DCACHE_LINE_NUM][DCACHE_WAY_NUM] = {0};
bool dcache_dirty[DCACHE_LINE_NUM][DCACHE_WAY_NUM] = {0};

void updatelru(int linenum)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           
{
    for (int i = 0; i < DCACHE_WAY_NUM; i++)
    {
        dcache_lru[linenum][i]++;
    }
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
void uselru(int linenum, int way)
{
    dcache_lru[linenum][way] = 0;
}

uint32_t read_cache_line(uint32_t index, uint32_t way, uint32_t offset)
{
    return dcache_data[index][way][offset];
}

void write_cache_line(uint32_t index, uint32_t way, uint32_t offset, uint32_t wdata, uint8_t wstrb)
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

void change_state(Dcache_State &state,bool io_req,bool hit,bool io_last,bool dirty,bool flush)
{
    if(state==DCACHE_IDLE){
        if(io_req==true&&!hit&&dirty==true){
            state = DCACHE_WRITE;
        }
        else if(io_req==true&&!hit&&dirty==false){
            state = DCACHE_READ;
        }
    }else if(state==DCACHE_WRITE){
        if(flush==true&&io_last==true){
            state = DCACHE_IDLE; 
        }
        else if(io_last==true){
            state = DCACHE_READ; 
        }
    }
    else if(state==DCACHE_READ){
        if(io_last==true){
            state = DCACHE_IDLE; 
        }
    }
}
void write_cache_line(uint32_t index, uint32_t way, uint32_t& offset,uint32_t& data, bool done,bool last)
{
    if(done){
        data = dcache_data[index][way][offset++];
    }
    if(last){
        offset=0;   
    }
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
void transfer_zero(EXMem_IO* &mem)
{
    mem->control.en = false;
    mem->control.wen = false;
    mem->control.addr = 0;
    mem->control.wdata = 0;
    mem->control.sel = 0;
    mem->control.len = 0;
    mem->control.size = 0;
    mem->control.last = true;
}
void write_data(EXMem_IO* &mem,uint32_t data,uint32_t addr,uint32_t offset)
{
    mem->control.en = true;
    mem->control.wen = true;
    mem->control.addr = addr;
    mem->control.wdata = data;
    mem->control.sel = 0b1111;
    mem->control.len = DCACHE_OFFSET_NUM - 1;
    mem->control.size = 0b10;
    mem->control.last = offset == DCACHE_OFFSET_NUM;
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
    mem->control.last = offset == DCACHE_OFFSET_NUM;
}
void miss_deal(uint32_t index, uint32_t way, uint32_t tag,uint32_t &hit_way,bool &dirty_writeback)
{
    hit_way = getlru(index);
    if(dcache_dirty[index][hit_way]){
        dirty_writeback=true;
    }
}