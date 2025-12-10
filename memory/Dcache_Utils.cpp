#include "Dcache_Utils.h"
#include <cstdio>
#include "MSHR.h"
extern long long sim_time;
uint32_t dcache_data[DCACHE_LINE_NUM][DCACHE_WAY_NUM][DCACHE_OFFSET_NUM] = {0};
uint32_t dcache_lru[DCACHE_LINE_NUM][DCACHE_WAY_NUM] = {0};
uint32_t dcache_tag[DCACHE_LINE_NUM][DCACHE_WAY_NUM] = {0};
bool dcache_valid[DCACHE_LINE_NUM][DCACHE_WAY_NUM] = {0};
bool dcache_dirty[DCACHE_LINE_NUM][DCACHE_WAY_NUM] = {0};
bool dcache_issued[DCACHE_LINE_NUM][DCACHE_WAY_NUM] = {0};

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

uint32_t read_cache_data_pipeline(uint32_t way, uint32_t old_dcache_data[DCACHE_WAY_NUM])
{
    return old_dcache_data[way];
}
void write_cache_data_pipeline(uint32_t index, uint32_t way, uint32_t offset,uint32_t old_dcache_data[DCACHE_WAY_NUM], uint32_t wdata, uint8_t wstrb)
{
    uint32_t old_data = old_dcache_data[way];
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
    if(DCACHE_LOG){
        printf("write cache data addr:0x%08x wdata:0x%08x wdatadone:0x%08x old_data:0x%08x wstrb:%02x index:%d offset:%d way:%d\n", get_addr(dcache_tag[index][way], index, offset),wdata,dcache_data[index][way][offset], old_data, wstrb, index, offset, way);
    }
}
void write_cache_data_load(uint32_t way,uint32_t store_data[DCACHE_WAY_NUM],uint32_t old_dcache_data[DCACHE_WAY_NUM], uint32_t wdata, uint8_t wstrb)
{
    uint32_t old_data = store_data[way];
    uint32_t mask = 0;
    if (wstrb & 0b1)
        mask |= 0xFF;
    if (wstrb & 0b10)
        mask |= 0xFF00;
    if (wstrb & 0b100)
        mask |= 0xFF0000;
    if (wstrb & 0b1000)
        mask |= 0xFF000000;
    old_dcache_data[way] = (mask & wdata) | (~mask & old_data);
    if(DCACHE_LOG){
        printf("write load data wdata:0x%08x wdatadone:0x%08x old_data:0x%08x wstrb:%02x way:%d\n",wdata,old_dcache_data[way], old_data, wstrb, way);
    }
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
void hit_check(uint32_t index, uint32_t tag, uint32_t tag_way[DCACHE_WAY_NUM], bool &hit,uint32_t &hit_way)
{
    for (int i = 0; i < DCACHE_WAY_NUM; i++)
    {
        if (tag_way[i] == tag && dcache_valid[index][i])
        {
            hit = true;
            hit_way = i;
        }
    }
    if(hit)return ;
    for(int i = 0; i < DCACHE_WAY_NUM; i++) //修改同一个
    {
        if (dcache_tag[index][i] == tag && dcache_issued[index][i])
        {
            hit_way = i;
        }
    }
    hit = false;
    return ;
}
bool hit_check_mmu(uint32_t index, uint32_t tag, uint32_t &hit_way)
{
    bool hit = false;
    hit_way = -1;
    for (int i = 0; i < DCACHE_WAY_NUM; i++)
    {
        if (dcache_tag[index][i] == tag && (dcache_valid[index][i]||dcache_issued[index][i]))
        {
            hit = true;
            hit_way = i;
        }
    }
    if(!hit){
        hit_way = getlru(index);
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
    mshrio->dirty=false;
    mshrio->way=0;
    mshrio->paddr = 0;
    mshrio->mispred = 0;
}
void transfer_data(MSHR_INFO* &mshrio,Mem_IN cpu,uint32_t tag,uint32_t offset,uint32_t index,uint32_t way,bool dirty,uint32_t paddr,bool ready,bool mispred)
{
    mshrio->valid = ready;
    mshrio->addr = cpu.addr;
    mshrio->wdata = cpu.wdata;
    mshrio->wr = cpu.wr;
    mshrio->wstrb = cpu.wstrb;
    mshrio->tag = tag;
    mshrio->index = index;
    mshrio->offset = offset;
    mshrio->dirty=dirty;
    mshrio->way=way;
    mshrio->paddr = paddr;
    mshrio->uop = cpu.uop;
    mshrio->mispred = mispred;
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
void miss_deal(uint32_t index, uint32_t& hit_way, uint32_t tag,bool &dirty_writeback,uint32_t&paddr,uint32_t tag_deal[DCACHE_WAY_NUM])
{
    dirty_writeback=dcache_dirty[index][hit_way];

    dcache_tag[index][hit_way]=tag;
    // printf("Dcache miss_deal writeback tag_deal:0x%08x index:%d offset:%d\n", tag_deal[hit_way], index, 0);
    paddr = get_addr(tag_deal[hit_way], index, 0);
    // dcache_dirty[index][hit_way]=0;
    dcache_valid[index][hit_way]=0;
    dcache_issued[index][hit_way]=1;
}
bool dcache_read(uint32_t addr, uint32_t &data)
{
    uint32_t tag, index, offset;
    get_addr_info(addr, tag, index, offset);
    // if(DCACHE_LOG){
    //     printf("MMU Dcache read addr:0x%08x tag:0x%08x index:%d offset:%d\n", addr, tag, index, offset);
    // }
    uint32_t hit_way;
    if (hit_check_mmu(index, tag, hit_way) || find_mshr_table(get_addr(tag, index, 0),hit_way))
    {
        data = read_cache_data(index, hit_way, offset);
        // if(DCACHE_LOG){
        //     printf("MMU Dcache read hit addr:0x%08x tag:0x%08x index:%d offset:%d way:%d data:%08x\n", addr, tag, index, offset, hit_way, data);
        // }
        return true;
    }
    return false;
}
void tag_and_data_read(uint32_t index,uint32_t offset,uint32_t tag[DCACHE_WAY_NUM], uint32_t data[DCACHE_WAY_NUM])
{
    for(int i=0;i<DCACHE_WAY_NUM;i++){
        tag[i] = dcache_tag[index][i];
        data[i] = dcache_data[index][i][offset];
    }
}