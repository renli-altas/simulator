#pragma once

#include "AXI.h"
#include "IO.h"
#include <config.h>
class ADAPTOR_IO
{
public:
  EXMem_IO *cpu;
  union axi_ar_channel *axi_ar;
  union axi_r_channel *axi_r;
  union axi_aw_channel *axi_aw;
  union axi_w_channel *axi_w;
  union axi_b_channel *axi_b;
};
enum ADAPTOR_STATE{
    IDLE,
    WRITE_WAIT,
    ARREADY_WAIT
};
class ADAPTOR
{
public:
  void comb();
  void seq();

  enum ADAPTOR_STATE state;
  bool wflag;
  bool rflag;

  bool rlast;
  bool wlast;
  
  bool rdone;
  bool wdone;
  bool wvalid;
  
  bool awready;
  bool wready;



  void init();

  ADAPTOR_IO io;
};