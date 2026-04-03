// RealLsu class fragment: STQ state and helpers.
#pragma once
#include "IO.h"
#include "config.h"

enum class StoreForwardState : uint8_t {
  NoHit = 0,
  Hit = 1,
  Retry = 2,
};

struct StoreForwardResult {
  StoreForwardState state = StoreForwardState::NoHit;
  uint32_t data = 0;
};

class StqState {

private:
  StqEntry stq[STQ_SIZE]{};
  int stq_head = 0;   // deq 指针
  int stq_commit = 0; // commit 指针
  int stq_tail = 0;   // enq 指针
  int stq_count = 0;
  int mshr_replay_count_stq = 0;
  bool stq_head_flag = false; // 用于区分环形缓冲区中的两轮
  bool pending_sta_addr_valid[STQ_SIZE]{};
  MicroOp pending_sta_addr_uops[STQ_SIZE]{};
  int pending_sta_addr_count = 0;
};
