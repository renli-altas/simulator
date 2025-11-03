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
  bool flush;
};

enum Dcache_State_LD
{
    DCACHE_IDLE_LD = 0,
    DCACHE_WAIT_LD = 1,
};
enum Dcache_State_ST
{
    DCACHE_IDLE_ST = 0,
    DCACHE_WAIT_ST = 1,
};

class Dcache_Check
{
public:

  void comb();
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

  bool hit_st;
  uint32_t hit_way_st;

  Dcache_IO io;

  uint32_t old_tag_ld;
  uint32_t old_tag_st;

  uint32_t hit_num;
  uint32_t miss_num;

  Dcache_State_LD state_ld;
  Dcache_State_ST state_st;
};
