#include "comb_recv.h"
#include "DcacheConfig.h"
#include "util.h"
#include <cstdint>
#include <cstring>

namespace {

constexpr int64_t REQ_WAIT_SEND = 0x7FFFFFFFFFFFFFFD;
constexpr int64_t REQ_WAIT_RESP = 0x7FFFFFFFFFFFFFFE;
constexpr uint64_t LD_RESP_STUCK_RETRY_CYCLES = 150;
constexpr uint64_t LD_KILLED_GC_CYCLES = 2;

} // namespace

namespace RealLsuBsd {

void comb_recv(CombRecvInterface ifc) {
  ifc.mmu.set_ptw_mem_port(ifc.ptw_mem_port);
  ifc.mmu.set_ptw_walk_port(ifc.ptw_walk_port);
  ifc.peripheral_io.in = {};
  PeripheralInIO mmio_req = {};
  bool mmio_req_used = false;

  if (ifc.pending_mmio_valid) {
    if (ifc.peripheral_io.out.ready) {
      ifc.peripheral_io.in = ifc.pending_mmio_req;
      ifc.pending_mmio_valid = false;
      ifc.pending_mmio_req = {};
    }
    mmio_req_used = true;
  } else if (!ifc.peripheral_io.out.ready) {
    mmio_req_used = true;
  }

  ifc.progress_pending_sta_addr();

  for (int i = 0; i < LSU_SDU_COUNT; i++) {
    if (ifc.in.exe2lsu->sdu_req[i].valid) {
      ifc.handle_store_data(ifc.in.exe2lsu->sdu_req[i].uop.to_micro_op());
    }
  }

  for (int i = 0; i < LSU_AGU_COUNT; i++) {
    if (ifc.in.exe2lsu->agu_req[i].valid) {
      const auto &uop = ifc.in.exe2lsu->agu_req[i].uop;
      if (uop.op == UOP_STA) {
        ifc.handle_store_addr(uop.to_micro_op());
      }
    }
  }

  for (int i = 0; i < LSU_AGU_COUNT; i++) {
    if (ifc.in.exe2lsu->agu_req[i].valid) {
      const auto &uop = ifc.in.exe2lsu->agu_req[i].uop;
      if (uop.op == UOP_LOAD) {
        ifc.handle_load_req(uop.to_micro_op());
      }
    }
  }

  for (int i = 0; i < LSU_LDU_COUNT; i++) {
    ifc.out.lsu2dcache->req_ports.load_ports[i].valid = false;
  }
  for (int i = 0; i < LSU_STA_COUNT; i++) {
    ifc.out.lsu2dcache->req_ports.store_ports[i].valid = false;
  }

  for (int i = 0; i < LDQ_SIZE; i++) {
    auto &entry = ifc.ldq[i];
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
    if (ifc.is_mmio_addr(entry.uop.diag_val)) {
      continue;
    }
    if (wait_cycles >= LD_RESP_STUCK_RETRY_CYCLES) {
      if (ifc.ctx != nullptr) {
        ifc.ctx->perf.ld_resp_timeout_retry_count++;
      }
      entry.sent = false;
      entry.waiting_resp = false;
      entry.wait_resp_since = 0;
      entry.uop.cplt_time = REQ_WAIT_SEND;
      entry.replay_priority = 3;
    }
  }

  ifc.replay_count_ldq = 0;
  ifc.replay_count_stq = 0;

  ifc.mshr_replay_count_ldq = 0;
  ifc.mshr_replay_count_stq = 0;
  for (int i = 0; i < LDQ_SIZE; i++) {
    const auto &e = ifc.ldq[i];
    if (!e.valid || e.killed || e.sent || e.waiting_resp) {
      continue;
    }
    if (e.replay_priority == 1) {
      ifc.mshr_replay_count_ldq++;
    }
  }
  for (int i = 0; i < STQ_SIZE; i++) {
    const auto &e = ifc.stq[i];
    if (!e.valid || !e.addr_valid || !e.data_valid || !e.committed || e.done ||
        e.send) {
      continue;
    }
    if (e.replay == 1) {
      ifc.mshr_replay_count_stq++;
    }
  }

  if (ifc.mshr_replay_count_stq > REPLAY_STORE_COUNT_UPPER_BOUND &&
      ifc.replay_type == 0) {
    ifc.replay_type = 1;
  } else if (ifc.mshr_replay_count_stq < REPLAY_STORE_COUNT_LOWER_BOUND &&
             ifc.replay_type == 1) {
    ifc.replay_type = 0;
  }

  if (ifc.mshr_replay_count_ldq == 0) {
    ifc.replay_type = 1;
  }

  bool has_replay = false;
  auto has_mmio_inflight = [&]() {
    if (ifc.pending_mmio_valid) {
      return true;
    }
    for (int idx = 0; idx < LDQ_SIZE; idx++) {
      const auto &e = ifc.ldq[idx];
      if (e.valid && e.waiting_resp && !e.killed && !e.is_mmio_wait &&
          ifc.is_mmio_addr(e.uop.diag_val)) {
        return true;
      }
    }
    for (int idx = 0; idx < STQ_SIZE; idx++) {
      const auto &e = ifc.stq[idx];
      if (e.valid && e.is_mmio && e.send && !e.done) {
        return true;
      }
    }
    return false;
  };

  const bool fill_wakeup = ifc.in.dcache2lsu->resp_ports.replay_resp.replay;
  const bool mshr_has_free =
      (ifc.in.dcache2lsu->resp_ports.replay_resp.free_slots > 0);

  if (fill_wakeup || mshr_has_free) {
    for (int i = 0; i < LDQ_SIZE; i++) {
      auto &entry = ifc.ldq[i];
      if (!entry.valid || entry.killed || entry.sent || entry.waiting_resp) {
        continue;
      }
      if (fill_wakeup && entry.replay_priority == 2 &&
          cache_line_match(
              entry.uop.diag_val,
              ifc.in.dcache2lsu->resp_ports.replay_resp.replay_addr)) {
        entry.replay_priority = 5;
      }
      if (mshr_has_free && entry.replay_priority == 1 && ifc.replay_type == 0 &&
          !has_replay) {
        entry.replay_priority = 4;
        has_replay = true;
      }
    }
    for (int i = 0; i < STQ_SIZE; i++) {
      auto &entry = ifc.stq[(ifc.stq_head + i) % STQ_SIZE];
      if (!entry.valid || !entry.addr_valid || !entry.data_valid ||
          !entry.committed || entry.done || entry.send) {
        continue;
      }
      if (fill_wakeup && entry.replay == 2 &&
          cache_line_match(
              entry.p_addr,
              ifc.in.dcache2lsu->resp_ports.replay_resp.replay_addr)) {
        entry.replay = 0;
      }
      if (mshr_has_free && entry.replay == 1 && ifc.replay_type == 1 &&
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
      auto &entry = ifc.ldq[j];
      if (!entry.valid || entry.killed || entry.sent || entry.waiting_resp) {
        continue;
      }
      if (entry.uop.cplt_time != REQ_WAIT_SEND) {
        continue;
      }
      if (entry.replay_priority == 1 || entry.replay_priority == 2) {
        continue;
      }
      if (entry.is_mmio_wait) {
        const bool mmio_can_issue =
            !mmio_req_used && !has_mmio_inflight() &&
            ifc.in.rob_bcast->head_valid &&
            !ifc.has_older_store_pending(entry.uop) &&
            entry.uop.rob_idx == (uint32_t)ifc.in.rob_bcast->head_rob_idx &&
            ifc.peripheral_io.out.ready;
        if (!mmio_can_issue) {
          if (ifc.ctx != nullptr) {
            ifc.ctx->perf.mmio_head_block_cycles++;
          }
          continue;
        }
        MicroOp mmio_uop = entry.uop;
        mmio_uop.rob_idx = j;
        mmio_req.is_mmio = 1;
        mmio_req.wen = 0;
        mmio_req.mmio_addr = entry.uop.diag_val;
        mmio_req.mmio_wdata = 0;
        mmio_req.uop = mmio_uop;
        mmio_req_used = true;
        ifc.pending_mmio_valid = true;
        ifc.pending_mmio_req = mmio_req;
        entry.is_mmio_wait = false;
        entry.sent = true;
        entry.waiting_resp = true;
        entry.wait_resp_since = sim_time;
        entry.uop.cplt_time = REQ_WAIT_RESP;
        break;
      }

      int rob_age = 0;
      if (ifc.in.rob_bcast->head_valid) {
        rob_age = (static_cast<int>(entry.uop.rob_idx) -
                   static_cast<int>(ifc.in.rob_bcast->head_rob_idx) +
                   ROB_NUM) %
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
      MicroOp req_uop = ifc.ldq[max_idx].uop;
      req_uop.rob_idx = max_idx;
      ifc.out.lsu2dcache->req_ports.load_ports[i].valid = true;
      ifc.out.lsu2dcache->req_ports.load_ports[i].addr =
          ifc.ldq[max_idx].uop.diag_val;
      ifc.out.lsu2dcache->req_ports.load_ports[i].req_id = max_idx;
      ifc.out.lsu2dcache->req_ports.load_ports[i].uop = req_uop;
      ifc.ldq[max_idx].sent = true;
      ifc.ldq[max_idx].waiting_resp = true;
      ifc.ldq[max_idx].wait_resp_since = sim_time;
      ifc.ldq[max_idx].uop.cplt_time = REQ_WAIT_RESP;
      if (ifc.ldq[max_idx].replay_priority >= 4) {
        ifc.ldq[max_idx].replay_priority = 0;
      }
    }
  }

  int issued_sta = 0;
  memset(ifc.issued_stq_addr_valid_nxt, 0, sizeof(ifc.issued_stq_addr_valid_nxt));
  for (int i = 0; i < ifc.stq_count && issued_sta < LSU_STA_COUNT; i++) {
    int stq_idx = (ifc.stq_head + i) % STQ_SIZE;
    auto &entry = ifc.stq[stq_idx];

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
      int older_stq_idx = (ifc.stq_head + j) % STQ_SIZE;
      auto &older_entry = ifc.stq[older_stq_idx];
      if (!older_entry.valid || !older_entry.addr_valid ||
          !older_entry.data_valid || !older_entry.committed ||
          older_entry.done || older_entry.suppress_write) {
        continue;
      }
      if (older_entry.p_addr == entry.p_addr) {
        continue_flag = true;
        break;
      }
    }
    if (continue_flag) {
      if (ifc.ctx != nullptr) {
        ifc.ctx->perf.stq_same_addr_block_count++;
      }
      continue;
    }
    if (entry.is_mmio) {
      if (mmio_req_used || has_mmio_inflight() || !ifc.peripheral_io.out.ready) {
        continue;
      }
      mmio_req.is_mmio = 1;
      mmio_req.wen = 1;
      mmio_req.mmio_addr = entry.p_addr;
      mmio_req.mmio_wdata = entry.data;
      mmio_req.uop = {};
      mmio_req.uop.op = UOP_STA;
      mmio_req.uop.rob_idx = stq_idx;
      mmio_req.uop.func3 = entry.func3;
      mmio_req_used = true;
      ifc.pending_mmio_valid = true;
      ifc.pending_mmio_req = mmio_req;
      entry.send = true;
      issued_sta++;
      continue;
    }
    ifc.issued_stq_addr_nxt[issued_sta] = entry.addr;
    ifc.issued_stq_addr_valid_nxt[issued_sta] = 1;
    ifc.change_store_info(entry, issued_sta, stq_idx);
    entry.send = true;
    issued_sta++;
  }
}

} // namespace RealLsuBsd
