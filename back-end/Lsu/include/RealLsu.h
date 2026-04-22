#pragma once

#include "LsuCommon.h"
#include "TlbMmu.h"
#include "config.h"
#include <cstdio>
#include <cstdint>
#include <memory>

class Csr;
class SimContext;
class PtwMemPort;
class PtwWalkPort;

class RealLsu {
private:
  static constexpr int FINISHED_LOADS_QUEUE_SIZE = ROB_NUM;
  static constexpr int FINISHED_STA_QUEUE_SIZE = ROB_NUM;
  static constexpr int PENDING_STA_ADDR_QUEUE_SIZE = STQ_SIZE;
  static constexpr int LOAD_RESP_WAIT_CYCLES_LIMIT = 150;
  static constexpr int STQ_COUNT_WIDTH = bit_width_for_count(STQ_SIZE + 1);
  static constexpr int LDQ_COUNT_WIDTH = bit_width_for_count(LDQ_SIZE + 1);
  static constexpr int FINISHED_LOADS_QUEUE_IDX_WIDTH =
      bit_width_for_count(FINISHED_LOADS_QUEUE_SIZE);
  static constexpr int FINISHED_LOADS_QUEUE_COUNT_WIDTH =
      bit_width_for_count(FINISHED_LOADS_QUEUE_SIZE + 1);
  static constexpr int FINISHED_STA_QUEUE_IDX_WIDTH =
      bit_width_for_count(FINISHED_STA_QUEUE_SIZE);
  static constexpr int FINISHED_STA_QUEUE_COUNT_WIDTH =
      bit_width_for_count(FINISHED_STA_QUEUE_SIZE + 1);
  static constexpr int PENDING_STA_ADDR_QUEUE_IDX_WIDTH =
      bit_width_for_count(PENDING_STA_ADDR_QUEUE_SIZE);
  static constexpr int PENDING_STA_ADDR_QUEUE_COUNT_WIDTH =
      bit_width_for_count(PENDING_STA_ADDR_QUEUE_SIZE + 1);
  static constexpr int LOAD_REPLAY_PRIORITY_WIDTH = bit_width_for_count(6);
  static constexpr int LOAD_RESP_WAIT_CYCLES_WIDTH =
      bit_width_for_count(LOAD_RESP_WAIT_CYCLES_LIMIT + 1);

  enum class LoadState : uint8_t {
    WaitExec = 0,
    WaitSend = 1,
    WaitResp = 2,
    WaitRetry = 3,
    Ready = 4,
  };

  struct LsuUop {
    wire<32> diag_val;
    wire<32> result;
    wire<PRF_IDX_WIDTH> dest_preg;
    wire<3> func3;
    wire<7> func7;
    wire<BR_MASK_WIDTH> br_mask;
    wire<ROB_IDX_WIDTH> rob_idx;
    wire<STQ_IDX_WIDTH> stq_idx;
    wire<1> stq_flag;
    wire<LDQ_IDX_WIDTH> ldq_idx;
    wire<1> rob_flag;
    wire<1> dest_en;
    wire<1> page_fault_inst;
    wire<1> page_fault_load;
    wire<1> page_fault_store;
    UopType op;
    DebugMeta dbg;
    wire<1> flush_pipe;
    bool is_cache_miss;

    static LsuUop from_micro_op(const MicroOp &src) {
      LsuUop dst{};
      dst.diag_val = src.diag_val;
      dst.result = src.result;
      dst.dest_preg = src.dest_preg;
      dst.func3 = src.func3;
      dst.func7 = src.func7;
      dst.br_mask = src.br_mask;
      dst.rob_idx = src.rob_idx;
      dst.stq_idx = src.stq_idx;
      dst.stq_flag = src.stq_flag;
      dst.ldq_idx = src.ldq_idx;
      dst.rob_flag = src.rob_flag;
      dst.dest_en = src.dest_en;
      dst.page_fault_inst = src.page_fault_inst;
      dst.page_fault_load = src.page_fault_load;
      dst.page_fault_store = src.page_fault_store;
      dst.op = src.op;
      dst.dbg = src.dbg;
      dst.flush_pipe = src.flush_pipe;
      dst.is_cache_miss = src.tma.is_cache_miss;
      return dst;
    }

    MicroOp to_micro_op() const {
      MicroOp dst;
      dst.diag_val = diag_val;
      dst.result = result;
      dst.dest_preg = dest_preg;
      dst.func3 = func3;
      dst.func7 = func7;
      dst.br_mask = br_mask;
      dst.rob_idx = rob_idx;
      dst.stq_idx = stq_idx;
      dst.stq_flag = stq_flag;
      dst.ldq_idx = ldq_idx;
      dst.rob_flag = rob_flag;
      dst.dest_en = dest_en;
      dst.page_fault_inst = page_fault_inst;
      dst.page_fault_load = page_fault_load;
      dst.page_fault_store = page_fault_store;
      dst.op = op;
      dst.dbg = dbg;
      dst.flush_pipe = flush_pipe;
      dst.tma.is_cache_miss = is_cache_miss;
      return dst;
    }
  };

  struct LdqEntry {
    reg<1> valid;
    reg<1> killed;
    reg<1> sent;
    reg<1> waiting_resp;
    LoadState load_state;
    reg<1> ready_delay;
    reg<LOAD_RESP_WAIT_CYCLES_WIDTH> resp_wait_cycles;
    reg<1> tlb_retry;
    reg<1> is_mmio_wait;
    reg<LOAD_REPLAY_PRIORITY_WIDTH> replay_priority;
    LsuUop uop;
  };

  enum class StoreForwardState : uint8_t {
    NoHit = 0,
    Hit = 1,
    Retry = 2,
  };

  struct StoreForwardResult {
    StoreForwardState state = StoreForwardState::NoHit;
    uint32_t data = 0;
  };

  struct StoreTag {
    reg<STQ_IDX_WIDTH> idx = 0;
    reg<1> flag = false;
  };

  struct StoreNode {
    StoreTag tag;
    StqEntry entry;
  };

  struct LsuState {
    StoreTag empty_stq_tag{};

    StoreNode committed_stq[STQ_SIZE];
    reg<STQ_IDX_WIDTH> committed_stq_head = 0;
    reg<STQ_COUNT_WIDTH> committed_stq_count = 0;
    StoreNode speculative_stq[STQ_SIZE];
    reg<STQ_IDX_WIDTH> speculative_stq_head = 0;
    reg<STQ_COUNT_WIDTH> speculative_stq_count = 0;

    LdqEntry ldq[LDQ_SIZE];
    reg<1> reserve_valid = false;
    reg<32> reserve_addr = 0;

    reg<1> replay_type = false;

    LsuUop finished_loads[FINISHED_LOADS_QUEUE_SIZE];
    reg<FINISHED_LOADS_QUEUE_IDX_WIDTH> finished_loads_head = 0;
    reg<FINISHED_LOADS_QUEUE_COUNT_WIDTH> finished_loads_count = 0;
    LsuUop finished_sta_reqs[FINISHED_STA_QUEUE_SIZE];
    reg<FINISHED_STA_QUEUE_IDX_WIDTH> finished_sta_reqs_head = 0;
    reg<FINISHED_STA_QUEUE_COUNT_WIDTH> finished_sta_reqs_count = 0;
    LsuUop pending_sta_addr_reqs[PENDING_STA_ADDR_QUEUE_SIZE];
    reg<PENDING_STA_ADDR_QUEUE_IDX_WIDTH> pending_sta_addr_reqs_head = 0;
    reg<PENDING_STA_ADDR_QUEUE_COUNT_WIDTH> pending_sta_addr_reqs_count = 0;
    reg<1> pending_mmio_valid = false;
    PeripheralReqIO pending_mmio_req{};
  };

  std::unique_ptr<TlbMmu> mmu;
  LsuState cur;
  LsuState nxt;

public:
  LsuIn in;
  LsuOut out;
  SimContext *ctx;

  RealLsu(SimContext *ctx);

  void init();
  void comb_cal();
  void comb_lsu2dis_info();
  void comb_recv();
  void comb_load_res();
  void comb_flush();
  void seq();

  StqEntry get_stq_entry(int stq_idx, bool stq_flag);
  void dump_debug_state() const {}
  void dump_mmu_debug(FILE *out) const { (void)out; }
  void set_csr(Csr *csr) { (void)csr; }
  void restore_reservation(bool valid, uint32_t addr) {
    (void)valid;
    (void)addr;
  }

  void set_ptw_mem_port(PtwMemPort *port) {
    mmu->set_ptw_mem_port(port);
  }
  void set_ptw_walk_port(PtwWalkPort *port) {
    mmu->set_ptw_walk_port(port);
  }

  uint32_t coherent_read(uint32_t p_addr);
  void overlay_committed_store_word(uint32_t p_addr,
                                    uint32_t &data);
  bool has_translation_store_conflict(uint32_t p_addr) const;
  bool has_committed_store_pending() const;

private:
  int get_mem_width(int func3) const { return lsu_get_mem_width(func3); }
  uint32_t extract_data(uint32_t raw_mem_val, uint32_t addr, int func3) const {
    return lsu_extract_data(raw_mem_val, addr, func3);
  }
  uint32_t merge_data_to_word(uint32_t old_word, uint32_t new_data,
                              uint32_t addr, int func3) const {
    return lsu_merge_data_to_word(old_word, new_data, addr, func3);
  }

  void handle_load_req(const MicroOp &uop);
  void handle_store_addr(const MicroOp &uop);
  void handle_store_data(const MicroOp &uop);
  StoreTag make_store_tag(int idx, bool flag) const;
  StoreTag next_store_tag(const StoreTag &tag) const;
  StoreTag current_stq_head_tag(const LsuState &state) const;
  StoreTag current_stq_tail_tag(const LsuState &state) const;
  int total_stq_count(const LsuState &state) const;
  int encode_store_req_id(const StoreTag &tag) const;
  StoreTag decode_store_req_id(int req_id) const;
  StoreNode &committed_stq_at(LsuState &state, int offset);
  const StoreNode &committed_stq_at(const LsuState &state, int offset) const;
  StoreNode &speculative_stq_at(LsuState &state, int offset);
  const StoreNode &speculative_stq_at(const LsuState &state, int offset) const;
  StoreNode *find_store_node(LsuState &state, const StoreTag &tag);
  const StoreNode *find_store_node(const LsuState &state,
                                   const StoreTag &tag) const;
  StqEntry *find_store_entry(LsuState &state, const StoreTag &tag);
  const StqEntry *find_store_entry(const LsuState &state,
                                   const StoreTag &tag) const;
  bool is_store_older(int s_idx, int s_flag, int l_idx, int l_flag) const;
  void clear_store_node(StoreNode &node);
  void committed_stq_push(LsuState &state, const StoreNode &node);
  StoreNode &committed_stq_front(LsuState &state);
  const StoreNode &committed_stq_front(const LsuState &state) const;
  void committed_stq_pop(LsuState &state);
  void speculative_stq_push(LsuState &state, const StoreNode &node);
  StoreNode &speculative_stq_front(LsuState &state);
  const StoreNode &speculative_stq_front(const LsuState &state) const;
  void speculative_stq_pop(LsuState &state);
  void move_speculative_front_to_committed(LsuState &state);
  bool reserve_stq_entry(LsuState &state, mask_t br_mask, uint32_t rob_idx,
                         uint32_t rob_flag, uint32_t func3, bool stq_flag);
  void consume_stq_alloc_reqs(LsuState &state);
  bool reserve_ldq_entry(LsuState &state, int idx, mask_t br_mask,
                         uint32_t rob_idx, uint32_t rob_flag);
  void consume_ldq_alloc_reqs(LsuState &state);
  void free_ldq_entry(LsuState &state, int idx);
  bool is_mmio_addr(uint32_t paddr) const;
  void change_store_info(const StoreNode &node, int port_idx);
  void handle_global_flush(LsuState &state);
  void handle_mispred(LsuState &state, mask_t mask);
  void retire_stq_head_if_ready(LsuState &state, int &pop_count);
  void commit_stores_from_rob(LsuState &state);
  void progress_ldq_entries(LsuState &state);
  void progress_pending_sta_addr(LsuState &state);
  bool finish_store_addr_once(LsuState &state, const LsuUop &inst);
  bool committed_store_conflicts_word(const LsuState &state,
                                      uint32_t word_addr) const;
  void set_load_state(LdqEntry &entry, LoadState state);
  void set_load_ready(LdqEntry &entry, uint8_t delay);
  void begin_load_response_wait(LdqEntry &entry);
  void prepare_runtime_state(LsuState &state);

  bool has_older_store_pending(const LsuState &state,
                               const LsuUop &load_uop) const;
  StoreForwardResult check_store_forward(const LsuState &state, uint32_t p_addr,
                                         const LsuUop &load_uop);
};
