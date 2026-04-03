#include "RealLsu.h"
#include "DcacheConfig.h"
#include "TlbMmu.h"
#include "config.h"
#include "util.h"
#include <cstdint>
#include <memory>
static constexpr uint64_t LD_RESP_STUCK_RETRY_CYCLES = 150;
static constexpr uint64_t LD_KILLED_GC_CYCLES = 2;
static inline bool is_amo_lr_uop(const MicroOp &uop) {
  return ((uop.dbg.instruction & 0x7Fu) == 0x2Fu) &&
         ((uop.func7 >> 2) == AmoOp::LR);
}

RealLsu::RealLsu(SimContext *ctx) : ctx(ctx) {
  // Initialize MMU
#ifdef CONFIG_TLB_MMU
  mmu = std::make_unique<TlbMmu>(ctx, nullptr, nullptr, DTLB_ENTRIES);
#else
  mmu = std::make_unique<SimpleMmu>(ctx, this);
#endif
  init();
}

void RealLsu::init() {
  init_stq_state();
  init_ldq_state();
  init_finished_state();
  pending_mmio_valid = false;
  pending_mmio_req = {};
  mmu->flush();

  reserve_valid = false;
  reserve_addr = 0;

  replay_type = 0;
}

// =========================================================
// 1. Dispatch 阶段: STQ 分配反馈
// =========================================================

void RealLsu::comb_lsu2dis_info() {
  const bool stq_tail_flag =
      (stq_count == STQ_SIZE || (stq_count > 0 && stq_tail < stq_head))
          ? !stq_head_flag
          : stq_head_flag;
  out.lsu2dis->stq_tail = stq_tail;
  out.lsu2dis->stq_tail_flag = stq_tail_flag;
  // Leave one full commit row of visible headroom. Without this slack, oracle
  // mode can overfill store-side speculation, which hurts Dhrystone IPC even
  // though the total STQ capacity is technically not exhausted.
  const int visible_stq_free_raw = STQ_SIZE - stq_count - COMMIT_WIDTH;
  out.lsu2dis->stq_free = visible_stq_free_raw > 0 ? visible_stq_free_raw : 0;
  out.lsu2dis->ldq_free = LDQ_SIZE - ldq_count;

  for (auto &v : out.lsu2dis->ldq_alloc_idx) {
    v = -1;
  }
  for (auto &v : out.lsu2dis->ldq_alloc_valid) {
    v = false;
  }
  int scan_pos = ldq_alloc_tail;
  int produced = 0;
  for (int n = 0; n < LDQ_SIZE && produced < MAX_LDQ_DISPATCH_WIDTH; n++) {
    if (!ldq[scan_pos].valid) {
      out.lsu2dis->ldq_alloc_idx[produced] = scan_pos;
      out.lsu2dis->ldq_alloc_valid[produced] = true;
      produced++;
    }
    scan_pos = (scan_pos + 1) % LDQ_SIZE;
  }

  // Populate miss_mask (Phase 4)
  uint64_t mask = 0;
  for (int i = 0; i < LDQ_SIZE; i++) {
    const auto &entry = ldq[i];
    if (entry.valid && !entry.killed && entry.uop.tma.is_cache_miss) {
      mask |= (1ULL << entry.uop.rob_idx);
    }
  }
  out.lsu2rob->tma.miss_mask = mask;
  out.lsu2rob->committed_store_pending = has_committed_store_pending();
}

// =========================================================
// 2. Execute 阶段: 接收 AGU/SDU 请求 (多端口轮询)
// =========================================================
void RealLsu::comb_recv() {
  auto &peripheral_req = *out.peripheral_req;
  const auto &peripheral_resp = *in.peripheral_resp;

  peripheral_req = {};
  PeripheralReqIO mmio_req = {};
  bool mmio_req_used = false;

  if (pending_mmio_valid) {
    if (peripheral_resp.ready) {
      peripheral_req = pending_mmio_req;
      // MMIO bridge is ready: hand off exactly once, then let inflight
      // tracking rely on LDQ/STQ waiting_resp/send state instead of
      // re-driving the same pending request forever.
      pending_mmio_valid = false;
      pending_mmio_req = {};
    } else {
      // Keep the request pending until the bridge becomes ready.
    }
    mmio_req_used = true;
  } else if (!peripheral_resp.ready) {
    // Bridge is still busy with a previously accepted MMIO transaction.
    mmio_req_used = true;
  }

  // Retry STA address translations that previously returned MMU::RETRY.
  progress_pending_sta_addr();

  // 1. 优先级：Store Data (来自 SDU)
  // 确保在消费者检查之前数据就绪
  for (int i = 0; i < LSU_SDU_COUNT; i++) {
    if (in.exe2lsu->sdu_req[i].valid) {
      handle_store_data(in.exe2lsu->sdu_req[i].uop.to_micro_op());
    }
  }

  // 2. 优先级：Store Addr (来自 AGU)
  // 确保地址对于别名检查有效
  for (int i = 0; i < LSU_AGU_COUNT; i++) {
    if (in.exe2lsu->agu_req[i].valid) {
      const auto &uop = in.exe2lsu->agu_req[i].uop;
      if (uop.op == UOP_STA) {
        handle_store_addr(uop.to_micro_op());
      }
    }
  }

  // 3. 优先级：Loads (来自 AGU)
  // 最后处理 Load，使其能看到本周期最新的 Store (STLF)
  for (int i = 0; i < LSU_AGU_COUNT; i++) {
    if (in.exe2lsu->agu_req[i].valid) {
      const auto &uop = in.exe2lsu->agu_req[i].uop;
      if (uop.op == UOP_LOAD) {
        handle_load_req(uop.to_micro_op());
      }
    }
  }

  for (int i = 0; i < LSU_LDU_COUNT; i++) {
    out.lsu2dcache->req_ports.load_ports[i].valid = false;
  }
  for (int i = 0; i < LSU_STA_COUNT; i++) {
    out.lsu2dcache->req_ports.store_ports[i].valid = false;
  }

  // Lost-response recovery:
  // 1) non-killed load waits too long -> retry
  // 2) killed load waits too long -> drop the slot to avoid LDQ leak/deadlock
  for (int i = 0; i < LDQ_SIZE; i++) {
    auto &entry = ldq[i];
    if (!entry.valid || !entry.sent || !entry.waiting_resp) {
      continue;
    }
    if (entry.uop.cplt_time != REQ_WAIT_RESP) {
      continue;
    }
    if (sim_time < 0) {
      continue;
    }
    const uint64_t sim_time_u64 = static_cast<uint64_t>(sim_time);
    if (sim_time_u64 < entry.wait_resp_since) {
      continue;
    }
    const uint64_t wait_cycles = sim_time_u64 - entry.wait_resp_since;
    if (entry.killed) {
      if (wait_cycles >= LD_KILLED_GC_CYCLES) {
        entry.sent = false;
        entry.waiting_resp = false;
        entry.wait_resp_since = 0;
        entry.uop.cplt_time = sim_time;
        entry.replay_priority = 0;
      }
      continue;
    }
    if (is_mmio_addr(entry.uop.diag_val)) {
      continue;
    }
    if (wait_cycles >= LD_RESP_STUCK_RETRY_CYCLES) {
      entry.sent = false;
      entry.waiting_resp = false;
      entry.wait_resp_since = 0;
      entry.uop.cplt_time = REQ_WAIT_SEND;
      entry.replay_priority = 3;
    }
  }

  // Rebuild MSHR replay pressure from queue state every cycle.
  // This avoids stale counters after flush/mispredict/recovery paths.
  mshr_replay_count_ldq = 0;
  mshr_replay_count_stq = 0;
  for (int i = 0; i < LDQ_SIZE; i++) {
    const auto &e = ldq[i];
    if (!e.valid || e.killed || e.sent || e.waiting_resp) {
      continue;
    }
    if (e.replay_priority == 1) {
      mshr_replay_count_ldq++;
    }
  }
  for (int i = 0; i < STQ_SIZE; i++) {
    const auto &e = stq[i];
    if (!e.valid || !e.addr_valid || !e.data_valid || !e.committed || e.done ||
        e.send) {
      continue;
    }
    if (e.replay == 1) {
      mshr_replay_count_stq++;
    }
  }

  if (mshr_replay_count_stq > REPLAY_STORE_COUNT_UPPER_BOUND &&
      replay_type == 0) {
    replay_type = 1;
  } else if (mshr_replay_count_stq < REPLAY_STORE_COUNT_LOWER_BOUND &&
             replay_type == 1) {
    replay_type = 0;
  }

  if (mshr_replay_count_ldq == 0)
    replay_type = 1;

  bool has_replay = false;
  auto has_mmio_inflight = [&]() {
    if (pending_mmio_valid) {
      return true;
    }
    for (int idx = 0; idx < LDQ_SIZE; idx++) {
      const auto &e = ldq[idx];
      if (e.valid && e.waiting_resp && !e.killed && !e.is_mmio_wait &&
          is_mmio_addr(e.uop.diag_val)) {
        return true;
      }
    }
    for (int idx = 0; idx < STQ_SIZE; idx++) {
      const auto &e = stq[idx];
      if (e.valid && e.is_mmio && e.send && !e.done) {
        return true;
      }
    }
    return false;
  };

  const bool fill_wakeup = in.dcache2lsu->resp_ports.replay_resp.replay;
  const bool mshr_has_free =
      (in.dcache2lsu->resp_ports.replay_resp.free_slots > 0);

  if (fill_wakeup || mshr_has_free) {
    for (int i = 0; i < LDQ_SIZE; i++) {
      auto &entry = ldq[i];
      if (!entry.valid || entry.killed || entry.sent || entry.waiting_resp) {
        continue;
      }
      if (fill_wakeup && entry.replay_priority == 2 &&
          cache_line_match(entry.uop.diag_val,
                           in.dcache2lsu->resp_ports.replay_resp.replay_addr)) {
        entry.replay_priority = 5;
      }
      if (mshr_has_free && entry.replay_priority == 1 && replay_type == 0 &&
          !has_replay) {
        entry.replay_priority = 4;
        has_replay = true;
      }
    }
    for (int i = 0; i < STQ_SIZE; i++) {
      auto &entry = stq[(stq_head + i) % STQ_SIZE];
      if (!entry.valid || !entry.addr_valid || !entry.data_valid ||
          !entry.committed || entry.done || entry.send) {
        continue;
      }
      if (fill_wakeup && entry.replay == 2 &&
          cache_line_match(entry.p_addr,
                           in.dcache2lsu->resp_ports.replay_resp.replay_addr)) {
        entry.replay = 0;
      }
      if (mshr_has_free && entry.replay == 1 && replay_type == 1 &&
          !has_replay) {
        entry.replay = 0;
        has_replay = true;
      }
    }
  }

  for (int i = 0; i < LSU_LDU_COUNT; i++) {
    int max_idx = -1;
    int max_priority = -1;
    int best_age = ROB_NUM + 1;
    for (int j = 0; j < LDQ_SIZE; j++) {
      auto &entry = ldq[j];
      if (!entry.valid || entry.killed || entry.sent || entry.waiting_resp) {
        continue;
      }
      // Only issue loads whose address translation / forwarding stage has
      // finished and explicitly marked them ready to send.
      if (entry.uop.cplt_time != REQ_WAIT_SEND) {
        continue;
      }
      // replay=1(mshr_full) and replay=2(mshr_hit) both wait for explicit
      // wakeup from MSHR fill events.
      if (entry.replay_priority == 1 || entry.replay_priority == 2) {
        continue;
      }
      // MMIO load 必须等到成为 ROB 当前最老的未提交指令后才发送，
      // 并且这条 load 之前的 store 已经从 STQ 排空，这样前面更老指令的
      // 提交副作用（尤其是 MMIO store）一定已经生效。
      if (entry.is_mmio_wait) {
        const bool mmio_can_issue =
            !mmio_req_used && !has_mmio_inflight() && in.rob_bcast->head_valid &&
            !has_older_store_pending(entry.uop) &&
            entry.uop.rob_idx == (uint32_t)in.rob_bcast->head_rob_idx &&
            peripheral_resp.ready;
        if (!mmio_can_issue) {
          continue;
        }
        // MMIO response path expects uop.rob_idx to carry an LDQ-local
        // token, not architectural ROB index.
        MicroOp mmio_uop = entry.uop;
        mmio_uop.rob_idx = j;
        mmio_req.is_mmio = 1;
        mmio_req.wen = 0; // Load 没有写使能
        mmio_req.mmio_addr = entry.uop.diag_val;
        mmio_req.mmio_wdata = 0; // Load 没有写数据
        mmio_req.uop = mmio_uop;
        mmio_req_used = true;
        pending_mmio_valid = true;
        pending_mmio_req = mmio_req;
        entry.is_mmio_wait = false; // 已发出请求，重置等待标志
        entry.sent = true;
        entry.waiting_resp = true;
        entry.wait_resp_since = sim_time;
        entry.uop.cplt_time = REQ_WAIT_RESP;
        break;
      }
      int rob_age = 0;
      if (in.rob_bcast->head_valid) {
        rob_age = (static_cast<int>(entry.uop.rob_idx) -
                   static_cast<int>(in.rob_bcast->head_rob_idx) + ROB_NUM) %
                  ROB_NUM;
      }
      if (entry.replay_priority > max_priority ||
          (entry.replay_priority == max_priority && rob_age < best_age)) {
        max_priority = entry.replay_priority;
        max_idx = j;
        best_age = rob_age;
      }
    }
    if (max_idx != -1) {
      MicroOp req_uop = ldq[max_idx].uop;
      req_uop.rob_idx = max_idx; // Local token: LDQ index
      out.lsu2dcache->req_ports.load_ports[i].valid = true;
      out.lsu2dcache->req_ports.load_ports[i].addr = ldq[max_idx].uop.diag_val;
      out.lsu2dcache->req_ports.load_ports[i].req_id = max_idx;
      out.lsu2dcache->req_ports.load_ports[i].uop = req_uop;
      ldq[max_idx].sent = true;
      ldq[max_idx].waiting_resp = true;
      ldq[max_idx].wait_resp_since = sim_time;
      ldq[max_idx].uop.cplt_time = REQ_WAIT_RESP;
      if (ldq[max_idx].replay_priority >= 4) {
        // replay_priority=4: replay=1(mshr_full) wakeup by free-slot.
        // replay_priority=5: replay=2(mshr_hit) wakeup by fill-match.
        ldq[max_idx].replay_priority = 0;
      }
    }
  }

  // Per cycle, each STA port can issue at most one real store request.
  // Scan STQ from head and pick the oldest issuable entries.
  int issued_sta = 0;
  for (int i = 0; i < stq_count && issued_sta < LSU_STA_COUNT; i++) {
    int stq_idx = (stq_head + i) % STQ_SIZE;
    auto &entry = stq[stq_idx];

    // Respect store ordering: younger stores cannot bypass an older
    // store whose addr/data/commit are not ready yet.
    if (!entry.valid || !entry.addr_valid || !entry.data_valid ||
        !entry.committed) {
      break;
    }

    if (entry.suppress_write) {
      continue;
    }
    if (entry.done || entry.send || entry.replay) {
      continue;
    }
    bool continue_flag = false;
    for (int j = 0; j < i; j++) {
      int older_stq_idx = (stq_head + j) % STQ_SIZE;
      auto &older_entry = stq[older_stq_idx];
      if (!older_entry.valid || !older_entry.addr_valid ||
          !older_entry.data_valid || !older_entry.committed ||
          older_entry.done || older_entry.suppress_write) {
        continue;
      }
      // Preserve program order for same-address stores until the older
      // store is fully acknowledged. Otherwise a bank-conflict replay can
      // let an older store reissue after a younger one and overwrite the
      // newer value.
      if (older_entry.p_addr == entry.p_addr) {
        continue_flag = true;
        break;
      }
    }
    if (continue_flag) {
      continue;
    }
    if (entry.is_mmio) {
      if (mmio_req_used) {
        continue;
      }
      if (has_mmio_inflight()) {
        continue;
      }
      // MMIO store only needs STQ ordering. Once it is the oldest
      // committed/ready store reachable from stq_head, it can issue
      // even if the ROB head has already advanced past it.
      if (!peripheral_resp.ready) {
        continue;
      }
      mmio_req.is_mmio = 1;
      mmio_req.wen = 1; // Store 有写使能
      mmio_req.mmio_addr = entry.p_addr;
      mmio_req.mmio_wdata = entry.data;
      mmio_req.uop = {};
      mmio_req.uop.op = UOP_STA;
      // MMIO response path uses uop.rob_idx as STQ slot token.
      mmio_req.uop.rob_idx = stq_idx;
      mmio_req.uop.func3 = entry.func3;
      mmio_req_used = true;
      pending_mmio_valid = true;
      pending_mmio_req = mmio_req;
      entry.send = true;
      issued_sta++;
      continue;
    }
    change_store_info(entry, issued_sta, stq_idx);
    entry.send = true; // Mark only when the request is truly driven.
    issued_sta++;
  }

  // MMIO request is sent only in the cycle-begin pending path above.
  // Do not re-drive here, otherwise a newly enqueued pending request can be
  // sent twice (once at end of this cycle, once again next cycle).
}

// =========================================================
// 3. Writeback 阶段: 输出 Load 结果 (多端口写回)
// =========================================================
void RealLsu::comb_load_res() {
  const auto &peripheral_resp = *in.peripheral_resp;

  // 1. 先清空所有写回端口
  for (int i = 0; i < LSU_LOAD_WB_WIDTH; i++) {
    out.lsu2exe->wb_req[i].valid = false;
  }

  for (int i = 0; i < LSU_LDU_COUNT; i++) {
    if (in.dcache2lsu->resp_ports.load_resps[i].valid) {
      int idx =
          static_cast<int>(in.dcache2lsu->resp_ports.load_resps[i].req_id);
      if (idx >= 0 && idx < LDQ_SIZE) {
        auto &entry = ldq[idx];
        if (entry.valid && entry.sent && entry.waiting_resp) {
          const auto &resp_uop = in.dcache2lsu->resp_ports.load_resps[i].uop;
          const bool same_token =
              (entry.uop.dbg.inst_idx == resp_uop.dbg.inst_idx) &&
              (entry.uop.rob_flag == resp_uop.rob_flag);
          if (!same_token) {
            continue;
          }
          if (!entry.killed) {
            if (in.dcache2lsu->resp_ports.load_resps[i].replay == 0) {
              uint32_t raw_data = in.dcache2lsu->resp_ports.load_resps[i].data;
              uint32_t extracted =
                  extract_data(raw_data, entry.uop.diag_val, entry.uop.func3);
              if (is_amo_lr_uop(entry.uop)) {
                reserve_addr = entry.uop.diag_val;
                reserve_valid = true;
              }
              entry.uop.result = extracted;
              entry.uop.dbg.difftest_skip =
                  in.dcache2lsu->resp_ports.load_resps[i].uop.dbg.difftest_skip;
              entry.uop.cplt_time = sim_time;
              entry.uop.tma.is_cache_miss =
                  !in.dcache2lsu->resp_ports.load_resps[i]
                       .uop.tma.is_cache_miss;
              entry.replay_priority = 0;
              const bool queued = finished_loads.push_back(entry.uop);
              Assert(queued && "finished_loads overflow");
              free_ldq_entry(idx);
            } else {
              // Handle load replay if needed (e.g., due to MSHR eviction)
              entry.replay_priority =
                  in.dcache2lsu->resp_ports.load_resps[i].replay;
              // replay=1(resource full) waits for a free-slot wakeup.
              // replay=2(mshr_hit) waits for matching line fill wakeup.
              entry.sent = false;
              entry.waiting_resp = false;
              entry.wait_resp_since = 0;
              entry.uop.cplt_time = REQ_WAIT_SEND;
            }
          } else {
            free_ldq_entry(idx);
          }
        }
      } else {
        Assert(false && "Invalid LDQ index in load response");
      }
    }
  }
  if (peripheral_resp.is_mmio && peripheral_resp.uop.op == UOP_LOAD) {
    int idx = peripheral_resp.uop.rob_idx;
    if (idx >= 0 && idx < LDQ_SIZE) {
      auto &entry = ldq[idx];
      if (entry.valid && entry.sent && entry.waiting_resp) {
        if (!entry.killed) {
          entry.uop.result = peripheral_resp.mmio_rdata;
          entry.uop.dbg.difftest_skip = peripheral_resp.uop.dbg.difftest_skip;
          entry.uop.cplt_time = sim_time;
          entry.uop.tma.is_cache_miss = false; // MMIO 访问不算 Cache Miss
          const bool queued = finished_loads.push_back(entry.uop);
          Assert(queued && "finished_loads overflow");
        }
      }
      free_ldq_entry(idx);
    } else {
      Assert(false && "Invalid LDQ index in MMIO load response");
    }
  }

  for (int i = 0; i < LSU_STA_COUNT; i++) {
    if (in.dcache2lsu->resp_ports.store_resps[i].valid) {
      int stq_idx = in.dcache2lsu->resp_ports.store_resps[i].req_id;
      if (stq_idx >= 0 && stq_idx < STQ_SIZE) {
        auto &entry = stq[stq_idx];
        if (entry.valid && !entry.done && entry.send) {
          if (in.dcache2lsu->resp_ports.store_resps[i].replay == 0) {
            entry.done = true;
            entry.replay = 0;
            entry.send = false;
          } else {
            // Handle store replay if needed (e.g., due to MSHR eviction)
            uint8_t replay_code =
                in.dcache2lsu->resp_ports.store_resps[i].replay;
            // replay=3 is bank-conflict: it should be retried directly
            // on the next cycle and must not freeze the STQ head.
            entry.replay = (replay_code == 3) ? 0 : replay_code;
            entry.send = false; // 重置发送标志，等待下次发送
          }
        }
      } else {
        Assert(false && "Invalid STQ index in store response");
      }
    }
  }

  if (peripheral_resp.is_mmio && peripheral_resp.uop.op == UOP_STA) {
    int stq_idx = peripheral_resp.uop.rob_idx;
    if (stq_idx >= 0 && stq_idx < STQ_SIZE) {
      auto &entry = stq[stq_idx];
      if (entry.valid && !entry.done && entry.send) {
        entry.done = true;
        entry.send = false;
      }
    } else {
      Assert(false && "Invalid STQ index in MMIO store response");
    }
  }

  // 2. 从完成队列填充端口 (Load)
  for (int i = 0; i < LSU_LOAD_WB_WIDTH; i++) {
    MicroOp wb_uop;
    if (finished_loads.pop_front(wb_uop)) {
      out.lsu2exe->wb_req[i].valid = true;
      out.lsu2exe->wb_req[i].uop =
          LsuExeIO::LsuExeRespUop::from_micro_op(wb_uop);
    } else {
      break;
    }
  }

  // 3. 从完成队列填充端口 (STA)
  for (int i = 0; i < LSU_STA_COUNT; i++) {
    MicroOp wb_uop;
    if (finished_sta_reqs.pop_front(wb_uop)) {
      out.lsu2exe->sta_wb_req[i].valid = true;
      out.lsu2exe->sta_wb_req[i].uop =
          LsuExeIO::LsuExeRespUop::from_micro_op(wb_uop);
    } else {
      out.lsu2exe->sta_wb_req[i].valid = false;
    }
  }
}

bool RealLsu::is_mmio_addr(uint32_t paddr) const {
  return ((paddr & UART_ADDR_MASK) == UART_ADDR_BASE) ||
         ((paddr & PLIC_ADDR_MASK) == PLIC_ADDR_BASE) ||
         (paddr == OPENSBI_TIMER_LOW_ADDR) ||
         (paddr == OPENSBI_TIMER_HIGH_ADDR);
}

// =========================================================
// 5. Exception: Flush 处理
// =========================================================

void RealLsu::comb_flush() {
  if (in.rob_bcast->flush) {
    // 1. LDQ: 已发请求项标记 killed，未发请求项直接释放
    for (int i = 0; i < LDQ_SIZE; i++) {
      if (!ldq[i].valid) {
        continue;
      }
      if (ldq[i].sent) {

        ldq[i].killed = true;
      } else {

        free_ldq_entry(i);
      }
    }
    finished_loads.clear();
    finished_sta_reqs.clear();
    clear_pending_sta_addr();
  }
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
    finished_sta_reqs.for_each_mut([&](MicroOp &e) { e.br_mask &= ~clear; });
    finished_loads.for_each_mut([&](MicroOp &e) { e.br_mask &= ~clear; });
    for (int i = 0; i < STQ_SIZE; i++) {
      if (pending_sta_addr_valid[i]) {
        pending_sta_addr_uops[i].br_mask &= ~clear;
      }
    }
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

}
