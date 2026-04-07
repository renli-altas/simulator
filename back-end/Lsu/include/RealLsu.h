#pragma once
#include "FIFO.h"
#include "IO.h"
#include "SimpleMmu.h"
#include "config.h"
#include <cstdint>
#include <memory>

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

enum class StoreForwardState : wire<2> {
  NoHit = 0,
  Hit = 1,
  Retry = 2,
  NONE = 3,
};

struct StoreForwardResult {
  StoreForwardState state = StoreForwardState::NoHit;
  uint32_t data = 0;
};

typedef struct{
  // Store Queue (同一套环形年龄顺序，按 commit 前/后拆成两份存储)
  FIFO_FLAG<StqCommitEntry, STQ_SIZE> commit_stq;
  FIFO_FLAG<StqDoneEntry, STQ_SIZE> done_stq;
  LdqEntry ldq[LDQ_SIZE];

  FIFO<MicroOp, LDQ_SIZE> finished_loads;
  // 4. 完成的 STA 队列 (等待访存流水线对齐写回)
  FIFO<MicroOp, STQ_SIZE> finished_sta_reqs;
  // 5. STA 地址翻译重试队列 (DTLB/PTW miss -> RETRY)
  FIFO<MicroOp, STQ_SIZE> pending_sta_addr_reqs;

  wire<1> reserve_valid;
  wire<32> reserve_addr;
  
  wire<1> replay_type; // 0 = LDQ, 1 = STQ
  // 3. 完成的 Load 队列 (等待写回)
  wire<1> pending_mmio_valid = false;
  PeripheralReqIO pending_mmio_req{};
}LSUState;

class RealLsu {
private:
  // MMU Instance (Composition)
  std::unique_ptr<AbstractMmu> mmu;
  LSUState cur;
  LSUState nxt;

  
public:
  LsuIn in;
  LsuOut out;
  SimContext *ctx;

  RealLsu(SimContext *ctx);

  // 组合逻辑实现
  void init();
  void comb_lsu2dis_info();
  void comb_recv();
  void comb_load_res();
  void comb_flush();
  void comb_pipeline();

  // 时序逻辑实现
  void seq();
  void set_ptw_mem_port(PtwMemPort *port) {
    mmu->set_ptw_mem_port(port);
  }
  void set_ptw_walk_port(PtwWalkPort *port) {
    mmu->set_ptw_walk_port(port);
  }
  void dump_debug_state() const;
  StqDoneEntry get_done_stq_entry(int idx, uint32_t rob_idx,
                                  uint32_t rob_flag) const;
  bool has_committed_store_pending() const {
    return nxt.done_stq.size()>0;
  }
  uint32_t coherent_read(uint32_t p_addr) const{
    assert("coherent_read should only be called for SimpleMMU"); 
    return 0;
  }


private:
  // 内部辅助函数
  wire<LDQ_IDX_WIDTH> LDQ_Count();

  void handle_load_req(const MicroOp &uop);
  void handle_store_addr(const MicroOp &uop);
  void handle_store_data(const MicroOp &inst);
  bool reserve_stq_entry(mask_t br_mask, uint32_t rob_idx, uint32_t rob_flag,
                         uint32_t stq_idx, uint32_t stq_flag,
                         uint32_t func3);
  bool reserve_ldq_entry(int idx, mask_t br_mask, uint32_t rob_idx,
                                uint32_t rob_flag); 
  void consume_stq_alloc_reqs();
  void consume_ldq_alloc_reqs();
  void free_ldq_entry(int idx);
  bool is_mmio_addr(uint32_t paddr) const;
  void change_store_info(const StqDoneEntry &entry, int port_idx, int stq_idx);
  void handle_global_flush();
  void handle_mispred(mask_t mask);
  void retire_stq_head_if_ready();
  void commit_stores_from_rob();
  void progress_ldq_entries();
  void progress_pending_sta_addr();
  bool finish_store_addr_once(const MicroOp &inst);
  StqCommitEntry *find_commit_stq_entry_by_rob(uint32_t rob_idx,
                                               uint32_t rob_flag);
  const StqCommitEntry *find_commit_stq_entry_by_rob(uint32_t rob_idx,
                                                     uint32_t rob_flag) const;
  void update_load_ready_state(LdqEntry &entry, uint32_t p_addr,
                               int64_t ready_cycle);

  bool has_older_store_pending(const MicroOp &load_uop) const;
  StoreForwardResult check_store_forward(uint32_t p_addr,
                                         const MicroOp &load_uop);
};
