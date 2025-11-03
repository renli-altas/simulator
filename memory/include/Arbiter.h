#pragma once

#include "IO.h"
#include <config.h>

class ARBITER_IO
{
public:
  Mem_IO *cpu_ld;
  Mem_IO *cpu_st;

  Mem_IO *cache_ld;
  Mem_IO *cache_st;

  EXMem_IO *mem;
};

class Arbiter
{
public:
  void comb_in();
  void comb_out();
  void seq();

  reg2_t state;

  void init();

  ARBITER_IO io;
};
