#pragma once

#include "config.h"

class FU {
public:
  void exec(Inst_uop &inst,Mem_IN* &io,bool mispred);
  int latency = 0;
  int cycle = 0;
  bool complete = false;
};
