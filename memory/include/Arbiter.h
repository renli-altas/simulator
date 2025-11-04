#pragma once

#include "IO.h"
#include <config.h>

class ARBITER_IO
{
public:
  EXMem_IO *cpu_ld;
  EXMem_IO *cpu_st;

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
