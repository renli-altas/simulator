#pragma once

#include "IO.h"
#include <config.h>
#include <cmath>

const int DCACHE_SIZE = 4096;                                                // 4KB
const int DCACHE_TAG_BITS = 22;                                              // sv32
const int DCACHE_WAY_NUM = 4;                                                // 4-way set associative
const int DCACHE_LINE_SIZE = 16;                                             // 16B - 4
const int DCACHE_OFFSET_NUM = DCACHE_LINE_SIZE / 4;                          // 4
const int DCACHE_OFFSET_BITS = log2(DCACHE_OFFSET_NUM);                      // 2
const int DCACHE_INDEX_BITS = 32 -2 - DCACHE_TAG_BITS - DCACHE_OFFSET_BITS;  // 6
const int DCACHE_LINE_NUM = DCACHE_SIZE / DCACHE_LINE_SIZE / DCACHE_WAY_NUM; // 64 lines

extern uint32_t dcache_data[DCACHE_LINE_NUM][DCACHE_WAY_NUM][DCACHE_OFFSET_NUM];
extern uint32_t dcache_lru[DCACHE_LINE_NUM][DCACHE_WAY_NUM];
extern uint32_t dcache_tag[DCACHE_LINE_NUM][DCACHE_WAY_NUM];// sv3
extern bool dcache_valid[DCACHE_LINE_NUM][DCACHE_WAY_NUM];
extern bool dcache_dirty[DCACHE_LINE_NUM][DCACHE_WAY_NUM];

enum Dcache_State
{
    DCACHE_IDLE = 0,
    DCACHE_WRITE = 1,
    DCACHE_READ = 2
};

void updatelru(int linenum);
int getlru(int linenum);
void uselru(int linenum,int way);

void write_cache_data(uint32_t index,uint32_t way,uint32_t offset,uint32_t wdata,uint8_t wstrb);
uint32_t read_cache_data(uint32_t index,uint32_t way,uint32_t offset);
bool hit_check(uint32_t index,uint32_t tag,uint32_t &hit_way);

void get_addr_info(uint32_t addr,uint32_t &tag,uint32_t& index,uint32_t &offset);
void change_state(Dcache_State &state,bool io_req,bool hit,bool io_last,bool dirty,bool flush);

void read_cache_line(uint32_t index, uint32_t way, uint32_t& offset,uint32_t data, bool done,bool last);
void write_cache_line(uint32_t index, uint32_t way, uint32_t& offset,uint32_t& data, bool done,bool last);

void transfer_zero(EXMem_IO* &mem);
void read_data(EXMem_IO* &mem,uint32_t addr,uint32_t offset);
void write_data(EXMem_IO* &mem,uint32_t data,uint32_t addr,uint32_t offset);
void miss_deal(uint32_t index, uint32_t& hit_way, uint32_t tag,bool &dirty_writeback);