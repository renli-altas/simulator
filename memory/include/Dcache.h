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


  uint32_t tag_ld1;
  uint32_t index_ld1;
  uint32_t offset_ld1;

  uint32_t tag_ld2;
  uint32_t index_ld2;
  uint32_t offset_ld2;

  uint32_t tag_st1;
  uint32_t index_st1;
  uint32_t offset_st1;
  
  uint32_t tag_st2;
  uint32_t index_st2;
  uint32_t offset_st2;

  bool hit_ld1;
  bool hit_ld2;
  uint32_t hit_way_ld1;
  uint32_t hit_way_ld2;
  uint32_t rdata;

  bool hit_st1;
  bool hit_st2;
  uint32_t hit_way_st1;
  uint32_t hit_way_st2;


  Dcache_IO io1;
  Dcache_IO io2;

  uint32_t hit_num;
  uint32_t miss_num;

  uint32_t write_data_st;
  uint32_t write_data_ld;

  bool dirty_writeback_ld;
  bool dirty_writeback_st;

  uint32_t paddr_ld;
  uint32_t paddr_st;
};
