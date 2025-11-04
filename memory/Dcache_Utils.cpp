#include "Dcache_Utils.h"

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

void update_cache_line(uint32_t index, uint32_t way, uint32_t tag)
{
    dcache_tag[index][way] = tag;
    dcache_valid[index][way] = true;
    dcache_dirty[index][way] = false;
}
void update_cache_offset(uint32_t index, uint32_t way, uint32_t offset, uint32_t data)
{
    dcache_data[index][way][offset] = data;
}
void get_addr_info(uint32_t addr, uint32_t &tag, uint32_t &index, uint32_t &offset)
{
    tag = (addr >> 2) >> (DCACHE_INDEX_BITS + DCACHE_OFFSET_BITS);
    index = ((addr >> 2) >> DCACHE_OFFSET_BITS) & ((1 << DCACHE_INDEX_BITS) - 1);
    offset = (addr >> 2) & ((1 << DCACHE_OFFSET_BITS) - 1);
}

void change_state(Dcache_State &state,bool io_req,bool hit,bool io_last,bool dirty)
{
    if(state==DCACHE_IDLE){
        if(io_req==true&&!hit&&dirty==true){
            state = DCACHE_WRITE;
        }
        else if(io_req==true&&!hit&&dirty==false){
            state = DCACHE_READ;
        }
    }else if(state==DCACHE_WRITE){
        if(io_last==true){
            state = DCACHE_READ; 
        }
    }
    else if(state==DCACHE_READ){
        if(io_last==true){
            state = DCACHE_IDLE; 
        }
    }
}
void transfer_cache_line(uint32_t index, uint32_t way, uint32_t offset,uint32_t data, bool done,bool last)
{
    if(done){
        dcache_data[index][way][offset] = data;
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
void write_data(Mem_IO* &cpu, EXMem_IO* &mem)
{
    mem->control.en = cpu->req;
    mem->control.wen = cpu->wr;
    mem->control.addr = cpu->addr;
    mem->control.wdata = cpu->wdata;
    mem->control.sel = cpu->wstrb;
    mem->control.len = DCACHE_OFFSET_NUM - 1;
    mem->control.size = 0b10;
    mem->control.last = true;
}
void read_data(Mem_IO* &cpu, EXMem_IO* &mem)
{
    cpu->req = mem->control.en;
    cpu->wr = mem->control.wen;
    cpu->addr = mem->control.addr;
    cpu->wdata = mem->control.wdata;
    cpu->wstrb = mem->control.sel;
}
void miss_deal(uint32_t index, uint32_t way, uint32_t tag,uint32_t &hit_way,bool &dirty_writeback)
{
    hit_way = getlru(index);
    if(dcache_dirty[index][hit_way]){
        dirty_writeback=true;
    }
    update_cache_line(index, hit_way, tag);
}