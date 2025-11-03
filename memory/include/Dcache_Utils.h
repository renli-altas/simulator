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

void updatelru(int linenum);
int getlru(int linenum);
void uselru(int linenum,int way);

void write_cache_line(uint32_t index,uint32_t way,uint32_t offset,uint32_t wdata,uint8_t wstrb);
uint32_t read_cache_line(uint32_t index,uint32_t way,uint32_t offset);
bool hit_check(uint32_t index,uint32_t tag,uint32_t &hit_way);

void get_addr_info(uint32_t addr,uint32_t &tag,uint32_t& index,uint32_t &offset);
void update_cache_line(uint32_t index, uint32_t way, uint32_t tag);