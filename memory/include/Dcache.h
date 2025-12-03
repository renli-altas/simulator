#pragma once

#include "IO.h"
#include <config.h>
#include <cmath>
#include "Dcache_Utils.h"



class Dcache_IO
{
public:
  Mem_IN *cpu_ld_in;
  Mem_IN *cpu_st_in;
  Mem_OUT *cpu_ld_out;
  Mem_OUT *cpu_st_out;
  MSHR_INFO* mshr_ld;
  MSHR_INFO* mshr_st;
  Cache_Mshr *mshr_control;
  Exe_Cache *control;
  Mem_OUT *mshr_out;
};

class Dcache
{
public:

  void comb_in();
  void comb_out();
  void seq();

  void init();
  void output(Mem_OUT*out,bool valid,bool wr,uint32_t data,uint32_t addr,Inst_uop uop);
  uint32_t tag_ld_way[DCACHE_WAY_NUM];
  uint32_t tag_st_way[DCACHE_WAY_NUM];
  uint32_t data_ld_way[DCACHE_WAY_NUM][DCACHE_OFFSET_NUM];
  uint32_t data_st_way[DCACHE_WAY_NUM][DCACHE_OFFSET_NUM];

  Dcache_IO io;
  Mem_IN cpu_ld_in;
  Mem_IN cpu_st_in;

  uint32_t data_ld;
  uint32_t data_st;

  
  bool hit_ld;
  bool hit_st;
  uint32_t hit_way_ld;
  uint32_t hit_way_st;

  uint32_t tag_st;
  uint32_t index_st;
  uint32_t offset_st;
  
  uint32_t tag_ld;
  uint32_t index_ld;
  uint32_t offset_ld;

  uint32_t hit_num;
  uint32_t miss_num;

  uint32_t write_data_st;
  uint32_t write_data_ld;

  bool dirty_writeback_ld;
  bool dirty_writeback_st;

  uint32_t paddr_ld;
  uint32_t paddr_st;
};
