#pragma once

#include "IO.h"
#include <config.h>
#include <cmath>
#include "Dcache_Utils.h"

class Dcache_IO
{
public:
  Mem_IO *cpu_ld;
  Mem_IO *cpu_st;
  MSHR_INFO* mshr_ld;
  MSHR_INFO* mshr_st;
  Cache_Mshr *mshr_control;
  Exe_Cache *control;
};

class Dcache
{
public:

  void comb_in();
  void comb_out();
  void seq();

  void init();


  uint32_t tag_ld;
  uint32_t index_ld;
  uint32_t offset_ld;

  uint32_t tag_st;
  uint32_t index_st;
  uint32_t offset_st;
  
  bool hit_ld;
  uint32_t hit_way_ld;
  uint32_t rdata;

  bool hit_st;
  uint32_t hit_way_st;

  Dcache_IO io;
  Dcache_State state_ld;
  Dcache_State state_st;

  uint32_t hit_num;
  uint32_t miss_num;

  uint32_t write_data_st;
  uint32_t write_data_ld;

  bool dirty_writeback_ld;
  bool dirty_writeback_st;

  uint32_t paddr_ld;
  uint32_t paddr_st;

  bool flush_flag;

  bool req_ld_reg;

  bool type;
};
