#pragma once
#include "IO.h"
#include "SimpleMmu.h"
#include "config.h"
#include <array>
#include <cstdint>
#include <memory>

#include "../BSD/finished.h"
#include "../BSD/ldq.h"
#include "../BSD/stq.h"

class SimContext;
class PtwMemPort;
class PtwWalkPort;

typedef struct {
  RobCommitIO *rob_commit;
  RobBroadcastIO *rob_bcast;
  DecBroadcastIO *dec_bcast;
  CsrStatusIO *csr_status;
  DisLsuIO *dis2lsu;
  ExeLsuIO *exe2lsu;
  DcacheLsuIO *dcache2lsu;
  PeripheralRespIO *peripheral_resp;
} LsuIn;

typedef struct {
  LsuDisIO *lsu2dis;
  LsuRobIO *lsu2rob;
  LsuExeIO *lsu2exe;
  LsuDcacheIO *lsu2dcache;
  PeripheralReqIO *peripheral_req;
} LsuOut;

class RealLsu {
private:
  static constexpr int64_t REQ_WAIT_RETRY = 0x7FFFFFFFFFFFFFFF;
  static constexpr int64_t REQ_WAIT_RESP = 0x7FFFFFFFFFFFFFFE;
  static constexpr int64_t REQ_WAIT_SEND = 0x7FFFFFFFFFFFFFFD;
  static constexpr int64_t REQ_WAIT_EXEC = 0x7FFFFFFFFFFFFFFC;

  // MMU Instance (Composition)
  std::unique_ptr<AbstractMmu> mmu;

  // === 内部状态寄存器 (对应 seq 更新) ===

  LdqState ldq_state{};
  StqState stq_state{};
  FinishedState finished_state{};

  bool reserve_valid;
  uint32_t reserve_addr;

  bool replay_type; // 0 = LDQ, 1 = STQ
  bool pending_mmio_valid = false;
  PeripheralReqIO pending_mmio_req{};

public:
  explicit RealLsu(SimContext *ctx);
  ~RealLsu() = default;

  LsuIn in{};
  LsuOut out{};
  SimContext *ctx;
  PtwMemPort *ptw_mem_port = nullptr;
  PtwWalkPort *ptw_walk_port = nullptr;

  // 组合逻辑实现
  void init();
  void comb_lsu2dis_info();
  void comb_recv();
  void comb_load_res();
  void comb_flush();

  // 时序逻辑实现
  void seq();

  StqEntry get_stq_entry(int stq_idx);

  void set_ptw_mem_port(PtwMemPort *port) {
    mmu->set_ptw_mem_port(port);
  }
  void set_ptw_walk_port(PtwWalkPort *port) {
    mmu->set_ptw_walk_port(port);
  }
  void restore_reservation(bool valid, uint32_t addr) {
    reserve_valid = valid;
    reserve_addr = addr;
  }

  // 一致性访存接口 (供 MMU 使用)
  uint32_t coherent_read(uint32_t p_addr);
  bool has_committed_store_pending() const {
    int ptr = stq_head;
    int remain = stq_count;
    while (remain > 0) {
      const StqEntry &e = stq[ptr];
      if (e.valid && e.committed && !e.done) {
        return true;
      }
      ptr = (ptr + 1) % STQ_SIZE;
      remain--;
    }
    return false;
  }

private:
  // 内部辅助函数
  void init_ldq_state();
  void handle_load_req(const MicroOp &uop);
  bool reserve_ldq_entry(int idx, mask_t br_mask, uint32_t rob_idx,
                         uint32_t rob_flag);
  void consume_ldq_alloc_reqs();
  void progress_ldq_entries();
  void update_load_ready_state(LdqEntry &entry, uint32_t p_addr,
                               int64_t ready_cycle);
  void free_ldq_entry(int idx);

  void init_stq_state();
  void handle_store_addr(const MicroOp &uop);
  void handle_store_data(const MicroOp &uop);
  int find_recovery_tail(mask_t br_mask);
  bool reserve_stq_entry(mask_t br_mask, uint32_t rob_idx, uint32_t rob_flag,
                         uint32_t func3);
  void consume_stq_alloc_reqs(int &push_count);
  int count_active_stq_entries() const;
  int count_committed_stq_prefix() const;
  int count_stq_entries_until(int stop_idx) const;
  void clear_stq_entries(int start_idx, int count);
  void change_store_info(StqEntry &entry, int port_idx, int stq_idx);
  void handle_global_flush();
  void handle_mispred(mask_t mask);
  void retire_stq_head_if_ready(int &pop_count);
  void commit_stores_from_rob();
  void progress_pending_sta_addr();
  void clear_pending_sta_addr();
  void enqueue_pending_sta_addr(const MicroOp &uop);
  bool finish_store_addr_once(const MicroOp &inst);
  bool has_older_store_pending(const MicroOp &load_uop) const;
  StoreForwardResult check_store_forward(uint32_t p_addr,
                                         const MicroOp &load_uop);

  void init_finished_state();

  int get_mem_width(int func3) const {
    switch (func3 & 0b11) {
    case 0b00:
      return 1;
    case 0b01:
      return 2;
    case 0b10:
      return 4;
    default:
      return 4;
    }
  }
  uint32_t extract_data(uint32_t raw_mem_val, uint32_t addr, int func3) const {
    int bit_offset = (addr & 0x3) * 8;
    uint32_t result = 0;
    uint32_t shifted = raw_mem_val >> bit_offset;

    switch (func3) {
    case 0b000:
      result = shifted & 0xFF;
      if (result & 0x80)
        result |= 0xFFFFFF00;
      break;
    case 0b001:
      result = shifted & 0xFFFF;
      if (result & 0x8000)
        result |= 0xFFFF0000;
      break;
    case 0b010:
      result = shifted;
      break;
    case 0b100:
      result = shifted & 0xFF;
      break;
    case 0b101:
      result = shifted & 0xFFFF;
      break;
    default:
      result = shifted;
      break;
    }
    return result;
  }
  uint32_t merge_data_to_word(uint32_t old_word, uint32_t new_data,
                              uint32_t addr, int func3) const {
    int bit_offset = (addr & 0x3) * 8;
    uint32_t mask = 0;

    switch (func3 & 0b11) {
    case 0b00:
      mask = 0xFF;
      break;
    case 0b01:
      mask = 0xFFFF;
      break;
    default:
      mask = 0xFFFFFFFF;
      break;
    }

    uint32_t clear_mask = ~(mask << bit_offset);
    uint32_t result = old_word & clear_mask;
    result |= ((new_data & mask) << bit_offset);
    return result;
  }
  bool is_mmio_addr(uint32_t paddr) const;
};
