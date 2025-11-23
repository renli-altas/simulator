#pragma once
#include "IO.h"
#include <config.h>
#include <cstdint>

class LDQ_IN {
public:
  Dis_Ldq *dis2ldq;
  Exe_Ldq *exe2ldq;
  // Rob_Commit *rob_commit;
  Dec_Broadcast *dec_bcast;
  Rob_Broadcast *rob_bcast;

  Mem_Data * cache2ldq;
};

class LDQ_OUT {
public:
  Ldq_Dis *ldq2dis;
  Ldq_Prf *ldq2prf;
  Mem_Control * ldq2cache;
};

// typedef struct {
//   bool ready[FETCH_WIDTH];

//   // 内存写端口
//   bool wen;
//   uint32_t wdata;
//   uint32_t waddr;
//   uint32_t wstrb;
// } LDQ_out;

class LDQ {
public:
  LDQ_IN in;
  LDQ_OUT out;
  void comb();
  void seq();

  void init();
  int enq_ptr;
  int deq_ptr;

  LDQ_entry entry[LDQ_NUM];
  int commit_ptr = 0;
  int count = 0;
  int commit_count = 0;
};
