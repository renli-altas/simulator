#pragma once

#include "../include/AbstractLsu.h"
#include "../include/AbstractMmu.h"
#include <cstdint>
#include <cstring>
#include <functional>

#define STQ_WIDTH (__builtin_ctz(STQ_SIZE))
#define LDQ_WIDTH (__builtin_ctz(LDQ_SIZE))

class PtwMemPort;
class PtwWalkPort;

namespace RealLsuBsd {

template <int Capacity> struct MicroOpArrayQueue {
  MicroOp entries[Capacity];
  int head = 0;
  int tail = 0;
  int count = 0;

  void clear() {
    head = 0;
    tail = 0;
    count = 0;
    for (int i = 0; i < Capacity; i++) {
      entries[i] = {};
    }
  }

  bool empty() const { return count == 0; }

  bool full() const { return count == Capacity; }

  int size() const { return count; }

  bool push_back(const MicroOp &uop) {
    if (full()) {
      return false;
    }
    entries[tail] = uop;
    tail = (tail + 1) % Capacity;
    count++;
    return true;
  }

  MicroOp front() const { return entries[head]; }

  void pop_front() {
    if (empty()) {
      return;
    }
    entries[head] = {};
    head = (head + 1) % Capacity;
    count--;
  }

  template <typename Fn> void for_each(Fn fn) {
    for (int i = 0; i < count; i++) {
      fn(entries[(head + i) % Capacity]);
    }
  }

  template <typename Pred> void remove_if(Pred pred) {
    MicroOp kept[Capacity];
    int kept_count = 0;
    for (int i = 0; i < count; i++) {
      const MicroOp &entry = entries[(head + i) % Capacity];
      if (!pred(entry)) {
        kept[kept_count++] = entry;
      }
    }
    clear();
    for (int i = 0; i < kept_count; i++) {
      entries[i] = kept[i];
    }
    count = kept_count;
    tail = (kept_count == Capacity) ? 0 : kept_count;
  }
};

using FinishedLoadQueue = MicroOpArrayQueue<ROB_NUM>;
using FinishedStaQueue = MicroOpArrayQueue<STQ_SIZE>;
using PendingStaAddrQueue = MicroOpArrayQueue<STQ_SIZE>;

struct LdqEntry {
  reg<1> valid;
  reg<1> killed;
  reg<1> sent;
  reg<1> waiting_resp;
  reg<64> wait_resp_since;
  reg<1> tlb_retry;
  reg<1> is_mmio_wait;
  reg<3> replay_priority;
  MicroOp uop;
};

struct CombLsu2DisInfoInterface {
  int stq_head;
  int stq_tail;
  int stq_count;
  bool stq_head_flag;
  int ldq_count;
  int ldq_alloc_tail;
  LdqEntry (&ldq)[LDQ_SIZE];
  LsuOut &out;
  bool committed_store_pending;
};

struct CombRecvInterface {
  AbstractMmu &mmu;
  PtwMemPort *ptw_mem_port;
  PtwWalkPort *ptw_walk_port;
  LsuIn &in;
  LsuOut &out;
  PeripheralIO &peripheral_io;
  SimContext *ctx;
  LdqEntry (&ldq)[LDQ_SIZE];
  StqEntry (&stq)[STQ_SIZE];
  int &stq_head;
  int &stq_count;
  bool &pending_mmio_valid;
  PeripheralInIO &pending_mmio_req;
  int &replay_count_ldq;
  int &replay_count_stq;
  int &mshr_replay_count_ldq;
  int &mshr_replay_count_stq;
  bool &replay_type;
  uint32_t (&issued_stq_addr_nxt)[LSU_STA_COUNT];
  bool (&issued_stq_addr_valid_nxt)[LSU_STA_COUNT];
  std::function<void()> progress_pending_sta_addr;
  std::function<void(const MicroOp &)> handle_store_data;
  std::function<void(const MicroOp &)> handle_store_addr;
  std::function<void(const MicroOp &)> handle_load_req;
  std::function<bool(uint32_t)> is_mmio_addr;
  std::function<bool(const MicroOp &)> has_older_store_pending;
  std::function<void(StqEntry &, int, int)> change_store_info;
};

struct CombLoadResInterface {
  LsuIn &in;
  LsuOut &out;
  PeripheralIO &peripheral_io;
  SimContext *ctx;
  LdqEntry (&ldq)[LDQ_SIZE];
  StqEntry (&stq)[STQ_SIZE];
  FinishedLoadQueue &finished_loads;
  FinishedStaQueue &finished_sta_reqs;
  bool &reserve_valid;
  uint32_t &reserve_addr;
  std::function<void(int)> free_ldq_entry;
  std::function<uint32_t(uint32_t, uint32_t, int)> extract_data;
};

struct CombFlushInterface {
  LsuIn &in;
  LdqEntry (&ldq)[LDQ_SIZE];
  FinishedLoadQueue &finished_loads;
  FinishedStaQueue &finished_sta_reqs;
  PendingStaAddrQueue &pending_sta_addr_reqs;
  std::function<void(int)> free_ldq_entry;
};

} // namespace RealLsuBsd
