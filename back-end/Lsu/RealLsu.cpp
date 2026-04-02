#include "RealLsu.h"
#include "AbstractLsu.h"
#include "DcacheConfig.h"
#include "PhysMemory.h"
#include "TlbMmu.h"
#include "config.h"
#include "util.h"
#include <cstdint>
#include <cstring>
#include <memory>
static constexpr int64_t REQ_WAIT_RETRY = 0x7FFFFFFFFFFFFFFF;
static constexpr int64_t REQ_WAIT_SEND = 0x7FFFFFFFFFFFFFFD;
static constexpr int64_t REQ_WAIT_RESP = 0x7FFFFFFFFFFFFFFE;
static constexpr int64_t REQ_WAIT_EXEC = 0x7FFFFFFFFFFFFFFC;
static constexpr uint64_t LD_RESP_STUCK_RETRY_CYCLES = 150;
static constexpr uint64_t LD_KILLED_GC_CYCLES = 2;
static inline bool is_amo_lr_uop(const MicroOp &uop) {
  return ((uop.dbg.instruction & 0x7Fu) == 0x2Fu) &&
         ((uop.func7 >> 2) == AmoOp::LR);
}
namespace {
inline bool stq_entry_matches_uop(const StqEntry &entry, const MicroOp &uop) {
  return entry.valid && entry.rob_idx == uop.rob_idx &&
         entry.rob_flag == uop.rob_flag;
}
} // namespace

RealLsu::RealLsu(SimContext *ctx) : AbstractLsu(ctx) {
  // Initialize MMU
#ifdef CONFIG_TLB_MMU
  mmu = std::make_unique<TlbMmu>(ctx, nullptr, nullptr, DTLB_ENTRIES);
#else
  mmu = std::make_unique<SimpleMmu>(ctx, this);
#endif
  init();
}

void RealLsu::init() {
  stq_head = 0;
  stq_tail = 0;
  stq_commit = 0;
  stq_count = 0;
  ldq_count = 0;
  ldq_alloc_tail = 0;
  stq_head_flag = false;
  finished_loads.clear();
  finished_sta_reqs.clear();
  pending_sta_addr_reqs.clear();
  pending_mmio_valid = false;
  pending_mmio_req = {};
  mmu->flush();

  reserve_valid = false;
  reserve_addr = 0;

  replay_type = 0;
  replay_count_ldq = 0;
  replay_count_stq = 0;

  mshr_replay_count_ldq = 0;
  mshr_replay_count_stq = 0;

  memset(issued_stq_addr, 0, sizeof(issued_stq_addr));
  memset(issued_stq_addr_nxt, 0, sizeof(issued_stq_addr));
  memset(issued_stq_addr_valid, 0, sizeof(issued_stq_addr_valid));
  memset(issued_stq_addr_valid_nxt, 0, sizeof(issued_stq_addr_valid_nxt));

  // 初始化所有 STQ LDQ 条目，防止未初始化内存导致的破坏
  for (int i = 0; i < STQ_SIZE; i++) {
    stq[i].valid = false;
    stq[i].addr_valid = false;
    stq[i].data_valid = false;
    stq[i].committed = false;
    stq[i].done = false;
    stq[i].is_mmio = false;
    stq[i].send = false;
    stq[i].replay = 0;
    stq[i].addr = 0;
    stq[i].data = 0;
    stq[i].br_mask = 0;
    stq[i].rob_idx = 0;
    stq[i].rob_flag = 0;
    stq[i].func3 = 0;
    stq[i].is_mmio = false;
  }

  for (int i = 0; i < LDQ_SIZE; i++) {
    ldq[i].valid = false;
    ldq[i].killed = false;
    ldq[i].sent = false;
    ldq[i].waiting_resp = false;
    ldq[i].wait_resp_since = 0;
    ldq[i].tlb_retry = false;
    ldq[i].is_mmio_wait = false;
    ldq[i].uop = {};
    ldq[i].replay_priority = 0;
  }
}

RealLsuBsd::CombLsu2DisInfoInterface RealLsu::make_comb_lsu2dis_info_ifc() {
  return {
      stq_head,
      stq_tail,
      stq_count,
      stq_head_flag,
      ldq_count,
      ldq_alloc_tail,
      ldq,
      out,
      has_committed_store_pending(),
  };
}

RealLsuBsd::CombRecvInterface RealLsu::make_comb_recv_ifc() {
  return {
      *mmu,
      ptw_mem_port,
      ptw_walk_port,
      in,
      out,
      peripheral_io,
      ctx,
      ldq,
      stq,
      stq_head,
      stq_count,
      pending_mmio_valid,
      pending_mmio_req,
      replay_count_ldq,
      replay_count_stq,
      mshr_replay_count_ldq,
      mshr_replay_count_stq,
      replay_type,
      issued_stq_addr_nxt,
      issued_stq_addr_valid_nxt,
      [this]() { progress_pending_sta_addr(); },
      [this](const MicroOp &uop) { handle_store_data(uop); },
      [this](const MicroOp &uop) { handle_store_addr(uop); },
      [this](const MicroOp &uop) { handle_load_req(uop); },
      [this](uint32_t paddr) { return is_mmio_addr(paddr); },
      [this](const MicroOp &uop) { return has_older_store_pending(uop); },
      [this](StqEntry &entry, int port_idx, int stq_idx) {
        change_store_info(entry, port_idx, stq_idx);
      },
  };
}

RealLsuBsd::CombLoadResInterface RealLsu::make_comb_load_res_ifc() {
  return {
      in,
      out,
      peripheral_io,
      ctx,
      ldq,
      stq,
      finished_loads,
      finished_sta_reqs,
      reserve_valid,
      reserve_addr,
      [this](int idx) { free_ldq_entry(idx); },
      [this](uint32_t raw_mem_val, uint32_t addr, int func3) {
        return extract_data(raw_mem_val, addr, func3);
      },
  };
}

RealLsuBsd::CombFlushInterface RealLsu::make_comb_flush_ifc() {
  return {
      in,
      ldq,
      finished_loads,
      finished_sta_reqs,
      pending_sta_addr_reqs,
      [this](int idx) { free_ldq_entry(idx); },
  };
}

// =========================================================
// 1. Dispatch 阶段: STQ 分配反馈
// =========================================================

void RealLsu::comb_lsu2dis_info() {
  RealLsuBsd::comb_lsu2dis_info(make_comb_lsu2dis_info_ifc());
}

// =========================================================
// 2. Execute 阶段: 接收 AGU/SDU 请求 (多端口轮询)
// =========================================================
void RealLsu::comb_recv() {
  RealLsuBsd::comb_recv(make_comb_recv_ifc());
}

// =========================================================
// 3. Writeback 阶段: 输出 Load 结果 (多端口写回)
// =========================================================
void RealLsu::comb_load_res() {
  RealLsuBsd::comb_load_res(make_comb_load_res_ifc());
}

void RealLsu::handle_load_req(const MicroOp &inst) {
  int ldq_idx = inst.ldq_idx;
  Assert(ldq_idx >= 0 && ldq_idx < LDQ_SIZE);
  if (!ldq[ldq_idx].valid || ldq[ldq_idx].killed) {
    return;
  }

  MicroOp task = inst;
  task.tma.is_cache_miss = false; // Initialize to false
  uint32_t p_addr;
  auto mmu_ret = mmu->translate(p_addr, task.result, 1, in.csr_status);

  if (mmu_ret == AbstractMmu::Result::RETRY) {
    task.cplt_time = REQ_WAIT_EXEC;
    ldq[ldq_idx].tlb_retry = true;
    ldq[ldq_idx].uop = task;
    return;
  }

  if (mmu_ret == AbstractMmu::Result::FAULT) {
    task.page_fault_load = true;
    task.diag_val = task.result; // Store faulting virtual address
    task.cplt_time = sim_time + 1;
  } else {
    task.diag_val = p_addr;

    // [Fix] Disable Store-to-Load Forwarding for MMIO ranges
    // These addresses involve side effects and must read from consistent memory
    bool is_mmio = is_mmio_addr(p_addr);
    if (is_mmio && ctx != nullptr) {
      ctx->perf.mmio_inst_count++;
      ctx->perf.mmio_load_count++;
    }
    // task.flush_pipe = is_mmio;
    ldq[ldq_idx].is_mmio_wait = is_mmio; // 延迟发送：等待到达 ROB 队头后再发出
    auto fwd_res =
        is_mmio ? StoreForwardResult{} : check_store_forward(p_addr, inst);

    if (fwd_res.state == StoreForwardState::Hit) {
      task.result = fwd_res.data;
      task.cplt_time = sim_time + 0; // 这一拍直接完成！
    } else if (fwd_res.state == StoreForwardState::NoHit) {
      task.cplt_time = REQ_WAIT_SEND;
    } else {
      task.cplt_time = REQ_WAIT_RETRY;
    }
  }

  ldq[ldq_idx].tlb_retry = false;
  ldq[ldq_idx].uop = task;
}

void RealLsu::handle_store_addr(const MicroOp &inst) {
  Assert(inst.stq_idx >= 0 && inst.stq_idx < STQ_SIZE);
  if (!finish_store_addr_once(inst)) {
    const bool queued = pending_sta_addr_reqs.push_back(inst);
    Assert(queued && "pending_sta_addr_reqs overflow");
  }
}

void RealLsu::handle_store_data(const MicroOp &inst) {
  Assert(inst.stq_idx >= 0 && inst.stq_idx < STQ_SIZE);
  if (!stq_entry_matches_uop(stq[inst.stq_idx], inst)) {
    return;
  }
  stq[inst.stq_idx].data = inst.result;
  stq[inst.stq_idx].data_valid = true;
}

bool RealLsu::reserve_stq_entry(mask_t br_mask, uint32_t rob_idx,
                                uint32_t rob_flag, uint32_t func3) {
  if (stq_count >= STQ_SIZE) {
    return false;
  }
  const int alloc_idx = stq_tail;
  stq[alloc_idx].valid = true;
  stq[alloc_idx].addr_valid = false;
  stq[alloc_idx].data_valid = false;
  stq[alloc_idx].committed = false;
  stq[alloc_idx].done = false;
  stq[alloc_idx].is_mmio = false;
  stq[alloc_idx].send = false;
  stq[alloc_idx].replay = 0;
  stq[alloc_idx].addr = 0;
  stq[alloc_idx].data = 0;
  stq[alloc_idx].suppress_write = 0;
  stq[alloc_idx].br_mask = br_mask;
  stq[alloc_idx].rob_idx = rob_idx;
  stq[alloc_idx].rob_flag = rob_flag;
  stq[alloc_idx].func3 = func3;
  stq_tail = (stq_tail + 1) % STQ_SIZE;
  return true;
}

void RealLsu::consume_stq_alloc_reqs(int &push_count) {
  for (int i = 0; i < MAX_STQ_DISPATCH_WIDTH; i++) {
    if (!in.dis2lsu->alloc_req[i]) {
      continue;
    }
    bool ok = reserve_stq_entry(in.dis2lsu->br_mask[i], in.dis2lsu->rob_idx[i],
                                in.dis2lsu->rob_flag[i], in.dis2lsu->func3[i]);
    Assert(ok && "STQ allocate overflow");
    push_count++;
  }
}

bool RealLsu::reserve_ldq_entry(int idx, mask_t br_mask, uint32_t rob_idx,
                                uint32_t rob_flag) {
  Assert(idx >= 0 && idx < LDQ_SIZE);
  if (ldq[idx].valid) {
    return false;
  }
  ldq[idx].valid = true;
  ldq[idx].killed = false;
  ldq[idx].sent = false;
  ldq[idx].waiting_resp = false;
  ldq[idx].wait_resp_since = 0;
  ldq[idx].tlb_retry = false;
  ldq[idx].is_mmio_wait = false;
  ldq[idx].replay_priority = 0;
  ldq[idx].uop = {};
  ldq[idx].uop.br_mask = br_mask;
  ldq[idx].uop.rob_idx = rob_idx;
  ldq[idx].uop.rob_flag = rob_flag;
  ldq[idx].uop.ldq_idx = idx;
  ldq[idx].uop.cplt_time = REQ_WAIT_EXEC;
  ldq_count++;
  ldq_alloc_tail = (idx + 1) % LDQ_SIZE;
  return true;
}

void RealLsu::consume_ldq_alloc_reqs() {
  for (int i = 0; i < MAX_LDQ_DISPATCH_WIDTH; i++) {
    if (!in.dis2lsu->ldq_alloc_req[i]) {
      continue;
    }
    bool ok = reserve_ldq_entry(
        in.dis2lsu->ldq_idx[i], in.dis2lsu->ldq_br_mask[i],
        in.dis2lsu->ldq_rob_idx[i], in.dis2lsu->ldq_rob_flag[i]);
    Assert(ok && "LDQ allocate collision");
  }
}

bool RealLsu::is_mmio_addr(uint32_t paddr) const {
  return ((paddr & UART_ADDR_MASK) == UART_ADDR_BASE) ||
         ((paddr & PLIC_ADDR_MASK) == PLIC_ADDR_BASE) ||
         (paddr == OPENSBI_TIMER_LOW_ADDR) ||
         (paddr == OPENSBI_TIMER_HIGH_ADDR);
}
void RealLsu::change_store_info(StqEntry &head, int port, int store_index) {

  uint32_t alignment_mask = (head.func3 & 0x3) == 0   ? 0
                            : (head.func3 & 0x3) == 1 ? 1
                                                      : 3;
  Assert((head.p_addr & alignment_mask) == 0 &&
         "DUT: Store address misaligned at commit!");

  uint32_t byte_off = head.p_addr & 0x3;
  uint32_t wstrb = 0;
  uint32_t wdata = 0;
  switch (head.func3 & 0x3) {
  case 0:
    wstrb = (1u << byte_off);
    wdata = (head.data & 0xFFu) << (byte_off * 8);
    break;
  case 1:
    wstrb = (0x3u << byte_off);
    wdata = (head.data & 0xFFFFu) << (byte_off * 8);
    break;
  default:
    wstrb = 0xFu;
    wdata = head.data;
    break;
  }

  out.lsu2dcache->req_ports.store_ports[port].valid = true;
  out.lsu2dcache->req_ports.store_ports[port].addr = head.p_addr;
  out.lsu2dcache->req_ports.store_ports[port].strb = wstrb;
  out.lsu2dcache->req_ports.store_ports[port].data = wdata;
  out.lsu2dcache->req_ports.store_ports[port].uop = head;
  out.lsu2dcache->req_ports.store_ports[port].req_id = store_index;
}

void RealLsu::handle_global_flush() {
  const int active_count = count_active_stq_entries();
  const int committed_count = count_committed_stq_prefix();
  const int discard_count = active_count - committed_count;

  stq_tail = stq_commit;
  stq_count = committed_count;
  clear_stq_entries(stq_tail, discard_count);
  pending_sta_addr_reqs.clear();
  pending_mmio_valid = false;
  pending_mmio_req = {};
  reserve_addr = 0;
  reserve_valid = false;
}

void RealLsu::handle_mispred(mask_t mask) {
  auto is_killed = [&](const MicroOp &u) { return (u.br_mask & mask) != 0; };

  for (int i = 0; i < LDQ_SIZE; i++) {
    if (!ldq[i].valid) {
      continue;
    }
    if (is_killed(ldq[i].uop)) {
      if (ldq[i].sent) {

        ldq[i].killed = true;
      } else {

        free_ldq_entry(i);
      }
    }
  }

  finished_sta_reqs.remove_if(is_killed);
  finished_loads.remove_if(is_killed);
  pending_sta_addr_reqs.remove_if(is_killed);

  if (pending_mmio_valid && (pending_mmio_req.uop.br_mask & mask) != 0) {
    pending_mmio_valid = false;
    pending_mmio_req = {};
  }

  int recovery_tail = find_recovery_tail(mask);
  if (recovery_tail == -1) {
    return;
  }

  const int active_count = count_active_stq_entries();
  stq_tail = recovery_tail;
  stq_count = count_stq_entries_until(recovery_tail);
  clear_stq_entries(stq_tail, active_count - stq_count);
}
void RealLsu::retire_stq_head_if_ready(int &pop_count) {
  const int retire_idx = stq_head;
  StqEntry &head = stq[retire_idx];
  const uint32_t retired_suppress_write = head.suppress_write;

  if (!head.valid) {
    return;
  }

  if (!head.suppress_write) {
    if (!(head.valid && head.addr_valid && head.data_valid && head.committed)) {
      return;
    }
    if (!head.done) {
      return;
    }
  }

  // Normal store: comb 阶段已完成写握手
  // Suppressed store: 跳过写握手直接 retire
  head.valid = false;
  head.committed = false;
  head.addr_valid = false;
  head.data_valid = false;
  head.done = false;
  head.addr = 0;
  head.data = 0;
  head.br_mask = 0;
  head.send = false;
  head.replay = 0;
  // Keep the suppression tag on an invalid entry until the slot is reused so
  // commit-time difftest can still recognize a failed SC that intentionally
  // does not perform a memory write.
  head.suppress_write = retired_suppress_write;

  stq_head++;
  if (stq_head == STQ_SIZE) {
    stq_head = 0;
    stq_head_flag = !stq_head_flag;
  }
  pop_count++;
}

void RealLsu::commit_stores_from_rob() {
  for (int i = 0; i < COMMIT_WIDTH; i++) {
    if (!in.rob_commit->commit_entry[i].valid) {
      continue;
    }
    const auto &commit_uop = in.rob_commit->commit_entry[i].uop;
    if (!is_store(commit_uop)) {
      continue;
    }
    int idx = commit_uop.stq_idx;
    Assert(idx >= 0 && idx < STQ_SIZE);
    if (idx == stq_commit) {
      stq[idx].committed = true;
      stq_commit = (stq_commit + 1) % STQ_SIZE;
    } else {
      Assert(0 && "Store commit out of order?");
    }
  }
}

void RealLsu::progress_ldq_entries() {
  for (int i = 0; i < LDQ_SIZE; i++) {
    auto &entry = ldq[i];
    if (!entry.valid) {
      continue;
    }
    if (entry.killed && !entry.sent) {

      free_ldq_entry(i);
      continue;
    }

    if (entry.waiting_resp || entry.uop.cplt_time == REQ_WAIT_EXEC) {
      if (!entry.tlb_retry) {
        continue;
      }
      uint32_t p_addr = 0;
      auto mmu_ret = mmu->translate(p_addr, entry.uop.result, 1, in.csr_status);
      if (mmu_ret == AbstractMmu::Result::RETRY) {
        continue;
      }
      entry.tlb_retry = false;
      if (mmu_ret == AbstractMmu::Result::FAULT) {
        entry.uop.page_fault_load = true;
        entry.uop.diag_val = entry.uop.result;
        entry.uop.cplt_time = sim_time + 1;
      } else {
        entry.uop.diag_val = p_addr;
        bool is_mmio = is_mmio_addr(p_addr);
        if (is_mmio && ctx != nullptr) {
          ctx->perf.mmio_inst_count++;
          ctx->perf.mmio_load_count++;
        }
        entry.uop.flush_pipe =
            false; // MMIO no longer triggers flush (non-speculative is enough)
        entry.is_mmio_wait = is_mmio; // 延迟发送：等待到达 ROB 队头后再发出
        auto fwd_res = is_mmio ? StoreForwardResult{}
                               : check_store_forward(p_addr, entry.uop);
        if (fwd_res.state == StoreForwardState::Hit) {
          entry.uop.result = fwd_res.data;
          entry.uop.cplt_time = sim_time;
        } else if (fwd_res.state == StoreForwardState::NoHit) {
          entry.uop.cplt_time = REQ_WAIT_SEND;
        } else {
          entry.uop.cplt_time = REQ_WAIT_RETRY;
        }
      }
      continue;
    }

    if (entry.uop.cplt_time == REQ_WAIT_RETRY) {
      auto fwd_res = check_store_forward(entry.uop.diag_val, entry.uop);
      if (fwd_res.state == StoreForwardState::Hit) {
        entry.uop.result = fwd_res.data;
        entry.uop.cplt_time = sim_time;
      } else if (fwd_res.state == StoreForwardState::NoHit) {
        entry.uop.cplt_time = REQ_WAIT_SEND;
      }
    }

    if (entry.uop.cplt_time <= sim_time) {
      if (!entry.killed) {
        if (is_amo_lr_uop(entry.uop)) {
          reserve_valid = true;
          reserve_addr = entry.uop.diag_val;
        }
        const bool queued = finished_loads.push_back(entry.uop);
        Assert(queued && "finished_loads overflow");
      }
      free_ldq_entry(i);
    }
  }
}

bool RealLsu::finish_store_addr_once(const MicroOp &inst) {
  int idx = inst.stq_idx;
  Assert(idx >= 0 && idx < STQ_SIZE);
  if (!stq_entry_matches_uop(stq[idx], inst)) {
    return true;
  }
  stq[idx].addr = inst.result; // VA

  uint32_t pa = inst.result;
  auto mmu_ret = mmu->translate(pa, inst.result, 2, in.csr_status);
  if (mmu_ret == AbstractMmu::Result::RETRY) {
    return false;
  }

  if (mmu_ret == AbstractMmu::Result::FAULT) {
    MicroOp fault_op = inst;
    fault_op.page_fault_store = true;
    fault_op.cplt_time = sim_time;
    if (is_amo_sc_uop(inst)) {
      reserve_valid = false;
    }
    const bool queued = finished_sta_reqs.push_back(fault_op);
    Assert(queued && "finished_sta_reqs overflow");
    stq[idx].p_addr = pa;
    stq[idx].addr_valid = false;
    return true;
  }

  MicroOp success_op = inst;
  success_op.cplt_time = sim_time;
  if (is_amo_sc_uop(inst)) {
    bool sc_success = reserve_valid && (reserve_addr == pa);
    // SC clears reservation regardless of success/failure.
    reserve_valid = false;
    success_op.result = sc_success ? 0 : 1;
    success_op.dest_en = true;
    success_op.op =
        UOP_LOAD; // Reuse existing LSU load wb/awake path for SC result
    stq[idx].suppress_write = !sc_success;
    const bool queued = finished_loads.push_back(success_op);
    Assert(queued && "finished_loads overflow");
    stq[idx].is_mmio = false; // SC 结果不区分 MMIO，始终走正常内存路径
    stq[idx].p_addr = pa;
    stq[idx].addr_valid = true;
    return true;
  }
  bool is_mmio = is_mmio_addr(pa);
  if (is_mmio && ctx != nullptr) {
    ctx->perf.mmio_inst_count++;
    ctx->perf.mmio_store_count++;
  }
  // MMIO store must not trigger ROB flush at STA writeback. Otherwise ROB may
  // flush globally before LSU consumes rob_commit, dropping the STQ commit.
  success_op.flush_pipe = false;
  stq[idx].is_mmio = is_mmio;
  const bool queued = finished_sta_reqs.push_back(success_op);
  Assert(queued && "finished_sta_reqs overflow");
  stq[idx].p_addr = pa;
  stq[idx].addr_valid = true;
  return true;
}

void RealLsu::progress_pending_sta_addr() {
  if (pending_sta_addr_reqs.empty()) {
    return;
  }
  const int n = pending_sta_addr_reqs.size();
  for (int i = 0; i < n; i++) {
    MicroOp op = pending_sta_addr_reqs.front();
    pending_sta_addr_reqs.pop_front();
    if (!finish_store_addr_once(op)) {
      const bool queued = pending_sta_addr_reqs.push_back(op);
      Assert(queued && "pending_sta_addr_reqs overflow");
    }
  }
}

void RealLsu::free_ldq_entry(int idx) {
  Assert(idx >= 0 && idx < LDQ_SIZE);
  if (ldq[idx].valid) {
    ldq[idx].valid = false;
    ldq[idx].killed = false;
    ldq[idx].sent = false;
    ldq[idx].waiting_resp = false;
    ldq[idx].wait_resp_since = 0;
    ldq[idx].tlb_retry = false;
    ldq[idx].is_mmio_wait = false;
    ldq[idx].uop = {};
    ldq_count--;
    Assert(ldq_count >= 0);
  }
}

// =========================================================
// 5. Exception: Flush 处理
// =========================================================

void RealLsu::comb_flush() {
  RealLsuBsd::comb_flush(make_comb_flush_ifc());
}

// =========================================================
// 6. Sequential Logic: 状态更新与时序模拟
// =========================================================
void RealLsu::seq() {
  bool is_flush = in.rob_bcast->flush;
  bool is_mispred = in.dec_bcast->mispred;
  int push_count = 0;
  int pop_count = 0;

  if (is_flush) {
    mmu->flush();
    handle_global_flush();
    return;
  }

  if (is_mispred) {
    mmu->flush();
    handle_mispred(in.dec_bcast->br_mask);
  }

  // 清除已解析分支的 br_mask bit（在 flush 之后，只影响存活条目）
  mask_t clear = in.dec_bcast->clear_mask;
  if (clear) {
    for (int i = 0; i < LDQ_SIZE; i++) {
      if (ldq[i].valid)
        ldq[i].uop.br_mask &= ~clear;
    }
    for (int i = 0; i < STQ_SIZE; i++) {
      if (stq[i].valid)
        stq[i].br_mask &= ~clear;
    }
    finished_sta_reqs.for_each(
        [&](MicroOp &e) { e.br_mask &= static_cast<mask_t>(~clear); });
    finished_loads.for_each(
        [&](MicroOp &e) { e.br_mask &= static_cast<mask_t>(~clear); });
    pending_sta_addr_reqs.for_each(
        [&](MicroOp &e) { e.br_mask &= static_cast<mask_t>(~clear); });
  }

  if (is_mispred) {
    return;
  }

  if (in.rob_bcast->fence) {
    mmu->flush();
  }

  consume_stq_alloc_reqs(push_count);
  consume_ldq_alloc_reqs();
  commit_stores_from_rob();

  // Make newly allocated stores visible before forwarding checks.
  stq_count = stq_count + push_count;
  if (stq_count > STQ_SIZE) {
    Assert(0 && "STQ Count Overflow! logic bug!");
  }
  progress_ldq_entries();

  // Retire after load progress so same-cycle completed stores can still
  // participate in store-to-load forwarding.
  retire_stq_head_if_ready(pop_count);
  stq_count = stq_count - pop_count;
  if (stq_count < 0) {
    Assert(0 && "STQ Count Underflow! logic bug!");
  }

  memcpy(issued_stq_addr, issued_stq_addr_nxt, sizeof(issued_stq_addr));
  memcpy(issued_stq_addr_valid, issued_stq_addr_valid_nxt,
         sizeof(issued_stq_addr_valid));

#if LSU_LIGHT_ASSERT
  if (pop_count == 0) {
    const StqEntry &head = stq[stq_head];
    const bool head_ready_to_retire = head.valid && head.addr_valid &&
                                      head.data_valid && head.committed &&
                                      head.done;
    Assert(!head_ready_to_retire &&
           "STQ invariant: retire-ready head was not popped");
  }
  // Lightweight O(1) ring invariants for STQ pointers/count.
  const int head_to_tail = (stq_tail - stq_head + STQ_SIZE) % STQ_SIZE;
  const int head_to_commit = (stq_commit - stq_head + STQ_SIZE) % STQ_SIZE;
  if (stq_count == 0) {
    Assert(stq_head == stq_tail && stq_tail == stq_commit &&
           "STQ invariant: empty queue pointer mismatch");
  } else if (stq_count == STQ_SIZE) {
    Assert(stq_head == stq_tail &&
           "STQ invariant: full queue requires head == tail");
  } else {
    Assert(head_to_tail == stq_count &&
           "STQ invariant: count != distance(head, tail)");
  }
  Assert(head_to_commit <= stq_count &&
         "STQ invariant: commit pointer is outside active window");
#endif
}

// =========================================================
// 辅助：基于 Tag 查找新的 Tail
// =========================================================
int RealLsu::find_recovery_tail(mask_t br_mask) {
  // 从 Commit 指针（安全点）开始，向 Tail 扫描
  // 我们要找的是“第一个”被误预测影响的指令
  // 因为是顺序分配，一旦找到一个，后面（更年轻）的肯定也都要丢弃

  int ptr = stq_commit;

  // 修正：正确计算未提交指令数，处理队列已满的情况 (Tail == Commit)
  // stq_count 追踪总有效条目 (Head -> Tail)。
  // Head -> Commit 之间的条目已提交。
  // Commit -> Tail 之间的条目未提交。
  int committed_count = count_committed_stq_prefix();
  int active_count = count_active_stq_entries();
  int uncommitted_count = active_count - committed_count;

  // 安全检查
  if (uncommitted_count < 0)
    uncommitted_count = 0; // 不应该发生
  int count = uncommitted_count;

  for (int i = 0; i < count; i++) {
    // 检查当前条目是否依赖于被误预测的分支
    if (stq[ptr].valid && (stq[ptr].br_mask & br_mask)) {
      // 找到了！这个位置就是错误路径的开始
      // 新的 Tail 应该回滚到这里
      return ptr;
    }
    ptr = (ptr + 1) % STQ_SIZE;
  }

  // 扫描完所有未提交指令都没找到相关依赖 -> 不需要回滚
  return -1;
}

int RealLsu::count_active_stq_entries() const {
  int count = 0;
  int ptr = stq_head;
  while (count < STQ_SIZE && stq[ptr].valid) {
    count++;
    ptr = (ptr + 1) % STQ_SIZE;
  }
  return count;
}

int RealLsu::count_committed_stq_prefix() const {
  int count = 0;
  int ptr = stq_head;
  while (count < STQ_SIZE && stq[ptr].valid && stq[ptr].committed) {
    count++;
    ptr = (ptr + 1) % STQ_SIZE;
  }
  return count;
}

int RealLsu::count_stq_entries_until(int stop_idx) const {
  int count = 0;
  int ptr = stq_head;
  while (count < STQ_SIZE && ptr != stop_idx && stq[ptr].valid) {
    count++;
    ptr = (ptr + 1) % STQ_SIZE;
  }
  return count;
}

void RealLsu::clear_stq_entries(int start_idx, int count) {
  int ptr = start_idx;
  for (int i = 0; i < count; i++) {
    stq[ptr].valid = false;
    stq[ptr].addr_valid = false;
    stq[ptr].data_valid = false;
    stq[ptr].committed = false;
    stq[ptr].done = false;
    stq[ptr].is_mmio = false;
    stq[ptr].send = false;
    stq[ptr].replay = 0;
    stq[ptr].addr = 0;
    stq[ptr].data = 0;
    stq[ptr].suppress_write = 0;
    stq[ptr].br_mask = 0;
    stq[ptr].rob_idx = 0;
    stq[ptr].rob_flag = 0;
    stq[ptr].func3 = 0;
    ptr = (ptr + 1) % STQ_SIZE;
  }
}

bool RealLsu::is_store_older(int s_idx, int s_flag, int l_idx, int l_flag) {
  if (s_flag == l_flag) {
    return s_idx < l_idx;
  } else {
    return s_idx > l_idx;
  }
}

bool RealLsu::has_older_store_pending(const MicroOp &load_uop) const {
  int ptr_idx = stq_head;
  bool ptr_flag = stq_head_flag;
  const int stop_idx = load_uop.stq_idx;
  const bool stop_flag = load_uop.stq_flag;
  int guard = 0;

  while (!(ptr_idx == stop_idx && ptr_flag == stop_flag)) {
    guard++;
    Assert(guard <= STQ_SIZE + 1);

    const StqEntry &entry = stq[ptr_idx];
    if (entry.valid && !entry.suppress_write) {
      return true;
    }

    ptr_idx++;
    if (ptr_idx == STQ_SIZE) {
      ptr_idx = 0;
      ptr_flag = !ptr_flag;
    }
  }

  return false;
}

// =========================================================
// 🛡️ [Nanako Implementation] 完整的 STLF 模拟逻辑
// =========================================================

RealLsu::StoreForwardResult
RealLsu::check_store_forward(uint32_t p_addr, const MicroOp &load_uop) {
  uint32_t current_word = 0;
  bool hit_any = false;
  int ptr_idx = stq_head;
  bool ptr_flag = stq_head_flag;
  const int stop_idx = load_uop.stq_idx;
  const bool stop_flag = load_uop.stq_flag;
  int guard = 0;

  // Scan [head, load.stop) in ring age order, where stop is (idx,flag).
  while (!(ptr_idx == stop_idx && ptr_flag == stop_flag)) {
    guard++;
    Assert(guard <= STQ_SIZE + 1);

    StqEntry &entry = stq[ptr_idx];
    if (entry.valid && !entry.suppress_write) {
      if (!entry.addr_valid) {
        return {StoreForwardState::Retry, 0};
      }

      int store_width = get_mem_width(entry.func3);
      int load_width = get_mem_width(load_uop.func3);
      uint32_t s_start = entry.p_addr;
      uint32_t s_end = s_start + store_width;
      uint32_t l_start = p_addr;
      uint32_t l_end = l_start + load_width;
      uint32_t overlap_start = std::max(s_start, l_start);
      uint32_t overlap_end = std::min(s_end, l_end);

      if (s_start <= l_start && s_end >= l_end) {
        // Store fully covers load bytes; merge by byte-lane so
        // sb/sh at non-zero byte offsets can still forward correctly.
        hit_any = true;
        if (!entry.data_valid) {
          return {StoreForwardState::Retry, 0};
        }
        current_word = merge_data_to_word(current_word, entry.data,
                                          entry.p_addr, entry.func3);
      } else if (overlap_start < overlap_end) {
        hit_any = true;
        // Partial overlap is intentionally conservative: keep the load in
        // retry until the older store fully retires from STQ.
        return {StoreForwardState::Retry, 0};
      }
    }

    ptr_idx++;
    if (ptr_idx == STQ_SIZE) {
      ptr_idx = 0;
      ptr_flag = !ptr_flag;
    }
  }

  if (!hit_any) {
    return {StoreForwardState::NoHit, 0};
  }
  return {StoreForwardState::Hit,
          extract_data(current_word, p_addr, load_uop.func3)};
}
StqEntry RealLsu::get_stq_entry(int stq_idx) {
  Assert(stq_idx >= 0 && stq_idx < STQ_SIZE);
  return stq[stq_idx];
}

uint32_t RealLsu::coherent_read(uint32_t p_addr) {
  // 1. 基准值：读物理内存 (假设 p_addr 已对齐到 4)
  Assert(0 && "coherent_read should not be called in current design!");
  uint32_t data = pmem_read(p_addr);

  // 2. 遍历 STQ 进行覆盖 (Coherent Check)
  int ptr = stq_head;
  int count = stq_count;
  for (int i = 0; i < count; i++) {
    const auto &entry = stq[ptr];
    if (entry.valid && entry.addr_valid && !entry.suppress_write) {
      // 只要 Store 的 Word 地址匹配，就进行 merge (假设 aligned Store 不跨
      // Word)
      if ((entry.p_addr >> 2) == (p_addr >> 2)) {
        data = merge_data_to_word(data, entry.data, entry.p_addr, entry.func3);
      }
    }
    ptr = (ptr + 1) % STQ_SIZE;
  }

  return data;
}

uint32_t extract_data(uint32_t raw_mem_val, uint32_t addr, int func3) {
  int bit_offset = (addr & 0x3) * 8; // 0, 8, 16, 24
  uint32_t result = 0;

  // 先移位，把目标数据移到最低位
  uint32_t shifted = raw_mem_val >> bit_offset;

  switch (func3) {
  case 0b000: // LB (Sign Ext)
    result = shifted & 0xFF;
    if (result & 0x80)
      result |= 0xFFFFFF00;
    break;
  case 0b001: // LH (Sign Ext)
    result = shifted & 0xFFFF;
    if (result & 0x8000)
      result |= 0xFFFF0000;
    break;
  case 0b010:         // LW
    result = shifted; // 取全部 32位
    break;
  case 0b100: // LBU (Zero Ext)
    result = shifted & 0xFF;
    break;
  case 0b101: // LHU (Zero Ext)
    result = shifted & 0xFFFF;
    break;
  default:
    result = shifted;
    break;
  }
  return result;
}
