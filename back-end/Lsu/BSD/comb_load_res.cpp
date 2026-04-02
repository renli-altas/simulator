#include "comb_load_res.h"
#include "util.h"
#include "RealLsu.h"
namespace {

constexpr int64_t REQ_WAIT_SEND = 0x7FFFFFFFFFFFFFFD;

inline bool is_amo_lr_uop(const MicroOp &uop) {
  return ((uop.dbg.instruction & 0x7Fu) == 0x2Fu) &&
         ((uop.func7 >> 2) == AmoOp::LR);
}

} // namespace

namespace RealLsuBsd {

void comb_load_res(CombLoadResInterface ifc) {
  for (int i = 0; i < LSU_LOAD_WB_WIDTH; i++) {
    ifc.out.lsu2exe->wb_req[i].valid = false;
  }
  for (int i = 0; i < LSU_STA_COUNT; i++) {
    ifc.out.lsu2exe->sta_wb_req[i].valid = false;
  }

  for (int i = 0; i < LSU_LDU_COUNT; i++) {
    if (!ifc.in.dcache2lsu->resp_ports.load_resps[i].valid) {
      continue;
    }
    int idx =
        static_cast<int>(ifc.in.dcache2lsu->resp_ports.load_resps[i].req_id);
    if (idx < 0 || idx >= LDQ_SIZE) {
      Assert(false && "Invalid LDQ index in load response");
    }

    auto &entry = ifc.ldq[idx];
    if (!(entry.valid && entry.sent && entry.waiting_resp)) {
      continue;
    }
    const auto &resp_uop = ifc.in.dcache2lsu->resp_ports.load_resps[i].uop;
    const bool same_token = (entry.uop.dbg.inst_idx == resp_uop.dbg.inst_idx) &&
                            (entry.uop.rob_flag == resp_uop.rob_flag);
    if (!same_token) {
      if (ifc.ctx != nullptr) {
        ifc.ctx->perf.ld_resp_stale_drop_count++;
      }
      continue;
    }

    if (!entry.killed) {
      if (ifc.in.dcache2lsu->resp_ports.load_resps[i].replay == 0) {
        uint32_t raw_data = ifc.in.dcache2lsu->resp_ports.load_resps[i].data;
        uint32_t extracted =
            ifc.extract_data(raw_data, entry.uop.diag_val, entry.uop.func3);
        if (is_amo_lr_uop(entry.uop)) {
          ifc.reserve_addr = entry.uop.diag_val;
          ifc.reserve_valid = true;
        }
        entry.uop.result = extracted;
        entry.uop.dbg.difftest_skip =
            ifc.in.dcache2lsu->resp_ports.load_resps[i].uop.dbg.difftest_skip;
        entry.uop.cplt_time = sim_time;
        entry.uop.tma.is_cache_miss =
            !ifc.in.dcache2lsu->resp_ports.load_resps[i].uop.tma.is_cache_miss;
        entry.replay_priority = 0;
        const bool queued = ifc.finished_loads.push_back(entry.uop);
        Assert(queued && "finished_loads overflow");
        ifc.free_ldq_entry(idx);
      } else {
        entry.replay_priority =
            ifc.in.dcache2lsu->resp_ports.load_resps[i].replay;
        entry.sent = false;
        entry.waiting_resp = false;
        entry.wait_resp_since = 0;
        entry.uop.cplt_time = REQ_WAIT_SEND;
      }
    } else {
      ifc.free_ldq_entry(idx);
    }
  }

  if (ifc.peripheral_io.out.is_mmio &&
      ifc.peripheral_io.out.uop.op == UOP_LOAD) {
    int idx = ifc.peripheral_io.out.uop.rob_idx;
    if (idx >= 0 && idx < LDQ_SIZE) {
      auto &entry = ifc.ldq[idx];
      if (entry.valid && entry.sent && entry.waiting_resp) {
        if (!entry.killed) {
          entry.uop.result = ifc.peripheral_io.out.mmio_rdata;
          entry.uop.dbg.difftest_skip =
              ifc.peripheral_io.out.uop.dbg.difftest_skip;
          entry.uop.cplt_time = sim_time;
          entry.uop.tma.is_cache_miss = false;
          const bool queued = ifc.finished_loads.push_back(entry.uop);
          Assert(queued && "finished_loads overflow");
        }
      }
      ifc.free_ldq_entry(idx);
    } else {
      Assert(false && "Invalid LDQ index in MMIO load response");
    }
  }

  for (int i = 0; i < LSU_STA_COUNT; i++) {
    if (!ifc.in.dcache2lsu->resp_ports.store_resps[i].valid) {
      continue;
    }
    int stq_idx = ifc.in.dcache2lsu->resp_ports.store_resps[i].req_id;
    if (stq_idx < 0 || stq_idx >= STQ_SIZE) {
      Assert(false && "Invalid STQ index in store response");
    }

    auto &entry = ifc.stq[stq_idx];
    if (!(entry.valid && !entry.done && entry.send)) {
      continue;
    }

    if (ifc.in.dcache2lsu->resp_ports.store_resps[i].replay == 0) {
      entry.done = true;
      entry.replay = 0;
      entry.send = false;
    } else {
      uint8_t replay_code = ifc.in.dcache2lsu->resp_ports.store_resps[i].replay;
      entry.replay = (replay_code == 3) ? 0 : replay_code;
      entry.send = false;
    }
  }

  if (ifc.peripheral_io.out.is_mmio &&
      ifc.peripheral_io.out.uop.op == UOP_STA) {
    int stq_idx = ifc.peripheral_io.out.uop.rob_idx;
    if (stq_idx >= 0 && stq_idx < STQ_SIZE) {
      auto &entry = ifc.stq[stq_idx];
      if (entry.valid && !entry.done && entry.send) {
        entry.done = true;
        entry.send = false;
      }
    } else {
      Assert(false && "Invalid STQ index in MMIO store response");
    }
  }

  for (int i = 0; i < LSU_LOAD_WB_WIDTH; i++) {
    if (ifc.finished_loads.empty()) {
      break;
    }
    ifc.out.lsu2exe->wb_req[i].valid = true;
    ifc.out.lsu2exe->wb_req[i].uop =
        LsuExeIO::LsuExeRespUop::from_micro_op(ifc.finished_loads.front());
    ifc.finished_loads.pop_front();
  }

  for (int i = 0; i < LSU_STA_COUNT; i++) {
    if (ifc.finished_sta_reqs.empty()) {
      break;
    }
    ifc.out.lsu2exe->sta_wb_req[i].valid = true;
    ifc.out.lsu2exe->sta_wb_req[i].uop =
        LsuExeIO::LsuExeRespUop::from_micro_op(ifc.finished_sta_reqs.front());
    ifc.finished_sta_reqs.pop_front();
  }
}

} // namespace RealLsuBsd
