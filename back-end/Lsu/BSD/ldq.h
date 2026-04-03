// RealLsu class fragment: LDQ state and helpers.

class RealLsu;

struct LdqEntry {
  bool valid;
  bool killed;
  bool sent;
  bool waiting_resp;
  uint64_t wait_resp_since;
  bool tlb_retry;
  bool is_mmio_wait; // 地址已翻译为 MMIO，等待到达 ROB 队头后再发送
  uint8_t replay_priority;
  MicroOp uop;
};

class LdqState {

private:
  LdqEntry ldq[LDQ_SIZE]{};
  int ldq_count = 0;
  int ldq_alloc_tail = 0;
  int mshr_replay_count_ldq = 0;
};
