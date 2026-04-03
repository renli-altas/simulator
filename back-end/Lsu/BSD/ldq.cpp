#include "RealLsu.h"
#include "util.h"

namespace {
inline bool is_amo_lr_uop(const MicroOp &uop) {
  return ((uop.dbg.instruction & 0x7Fu) == 0x2Fu) &&
         ((uop.func7 >> 2) == AmoOp::LR);
}
} // namespace

void RealLsu::init_ldq_state() {
  ldq_count = 0;
  ldq_alloc_tail = 0;
  mshr_replay_count_ldq = 0;
  for (auto &entry : ldq) {
    entry = LdqEntry{};
  }
}

void RealLsu::handle_load_req(const MicroOp &inst) {
  int ldq_idx = inst.ldq_idx;
  Assert(ldq_idx >= 0 && ldq_idx < LDQ_SIZE);
  if (!ldq[ldq_idx].valid || ldq[ldq_idx].killed) {
    return;
  }

  MicroOp task = inst;
  task.tma.is_cache_miss = false;
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
    task.diag_val = task.result;
    task.cplt_time = sim_time + 1;
  } else {
    ldq[ldq_idx].uop = task;
    ldq[ldq_idx].tlb_retry = false;
    update_load_ready_state(ldq[ldq_idx], p_addr, sim_time);
    return;
  }

  ldq[ldq_idx].tlb_retry = false;
  ldq[ldq_idx].uop = task;
}

bool RealLsu::reserve_ldq_entry(int idx, mask_t br_mask, uint32_t rob_idx,
                                uint32_t rob_flag) {
  Assert(idx >= 0 && idx < LDQ_SIZE);
  if (ldq[idx].valid) {
    return false;
  }
  LdqEntry &entry = ldq[idx];
  entry = LdqEntry{};
  entry.valid = true;
  entry.uop.br_mask = br_mask;
  entry.uop.rob_idx = rob_idx;
  entry.uop.rob_flag = rob_flag;
  entry.uop.ldq_idx = idx;
  entry.uop.cplt_time = REQ_WAIT_EXEC;
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
        update_load_ready_state(entry, p_addr, sim_time);
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

void RealLsu::update_load_ready_state(LdqEntry &entry, uint32_t p_addr,
                                      int64_t ready_cycle) {
  entry.uop.diag_val = p_addr;
  entry.is_mmio_wait = is_mmio_addr(p_addr);
  if (entry.is_mmio_wait) {
    entry.uop.flush_pipe = false;
  }

  const StoreForwardResult fwd_res =
      entry.is_mmio_wait ? StoreForwardResult{}
                         : check_store_forward(p_addr, entry.uop);
  if (fwd_res.state == StoreForwardState::Hit) {
    entry.uop.result = fwd_res.data;
    entry.uop.cplt_time = ready_cycle;
  } else if (fwd_res.state == StoreForwardState::NoHit) {
    entry.uop.cplt_time = REQ_WAIT_SEND;
  } else {
    entry.uop.cplt_time = REQ_WAIT_RETRY;
  }
}

void RealLsu::free_ldq_entry(int idx) {
  Assert(idx >= 0 && idx < LDQ_SIZE);
  if (ldq[idx].valid) {
    ldq[idx] = LdqEntry{};
    ldq_count--;
    Assert(ldq_count >= 0);
  }
}
