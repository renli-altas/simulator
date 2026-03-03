#include "include/ICacheTop.h"
#include "../front_module.h"
#include "../frontend.h"
#include "PhysMemory.h"
#include "PtwMemPort.h"
#include "PtwWalkPort.h"
#include "RISCV.h"
#include "SimpleMmu.h"
#include "TlbMmu.h"
#include "config.h" // For SimContext
#include "include/icache_module.h"
#include "Csr.h"
#include <cstdio>
#include <iostream>

// External dependencies
extern ICache icache; // Defined in icache.cpp

namespace {
uint32_t icache_coherent_read(uint32_t p_addr) { return pmem_read(p_addr); }

class IcacheBlockingPtwPort : public PtwMemPort {
public:
  bool send_read_req(uint32_t paddr) override {
    resp_data_reg = icache_coherent_read(paddr);
    resp_valid_reg = true;
    return true;
  }
  bool resp_valid() const override { return resp_valid_reg; }
  uint32_t resp_data() const override { return resp_data_reg; }
  void consume_resp() override { resp_valid_reg = false; }

private:
  bool resp_valid_reg = false;
  uint32_t resp_data_reg = 0;
};
} // namespace

// --- ICacheTop Implementation ---

void ICacheTop::syncPerf() {
  if (ctx) {
    ctx->perf.icache_access_num += access_delta;
    ctx->perf.icache_miss_num += miss_delta;
  }
  // Reset deltas
  access_delta = 0;
  miss_delta = 0;
}

// --- TrueICacheTop Implementation ---

TrueICacheTop::TrueICacheTop(ICache &hw) : icache_hw(hw) {}

void TrueICacheTop::set_ptw_mem_port(PtwMemPort *port) {
  ptw_mem_port = port;
  if (mmu_model != nullptr) {
    mmu_model->set_ptw_mem_port(port);
  }
}
void TrueICacheTop::set_ptw_walk_port(PtwWalkPort *port) {
  ptw_walk_port = port;
  if (mmu_model != nullptr) {
    mmu_model->set_ptw_walk_port(port);
  }
}

void TrueICacheTop::comb() {
  out->icache_read_ready_2 = false;
  out->icache_read_complete_2 = false;
  out->fetch_pc_2 = 0;
  for (int i = 0; i < FETCH_WIDTH; i++) {
    out->fetch_group_2[i] = INST_NOP;
    out->page_fault_inst_2[i] = false;
    out->inst_valid_2[i] = false;
  }

  static IcacheBlockingPtwPort ptw_port;
  if (mmu_model == nullptr && ctx != nullptr) {
#ifdef CONFIG_TLB_MMU
    mmu_model =
        new TlbMmu(ctx, ptw_mem_port ? ptw_mem_port : &ptw_port, ITLB_ENTRIES);
    if (ptw_walk_port != nullptr) {
      mmu_model->set_ptw_walk_port(ptw_walk_port);
    }
#else
    mmu_model = new SimpleMmu(ctx, nullptr);
#endif
  }

  tlb_set_pending_comb = false;
  tlb_clear_pending_comb = false;

  if (in->reset) {
    DEBUG_LOG("[icache] reset\n");
    icache_hw.reset();
    if (mmu_model != nullptr) {
      mmu_model->flush();
    }
    satp_seen = false;
    valid_reg = false;
    mem_busy = false;
    mem_latency_cnt = 0;
    tlb_pending = false;
    tlb_pending_vaddr = 0;
    out->icache_read_ready = true;
    return;
  }

  uint32_t cur_satp = in->csr_status ? static_cast<uint32_t>(in->csr_status->satp) : 0;
  if (!satp_seen || cur_satp != last_satp || in->refetch) {
    if (mmu_model != nullptr) {
      mmu_model->flush();
    }
    satp_seen = true;
    last_satp = cur_satp;
    tlb_pending = false;
    tlb_pending_vaddr = 0;
  }

  // deal with "refetch" signal (Async Reset behavior)
  if (in->refetch) {
    icache_hw.set_refetch();
    valid_reg = false;
    mem_busy = false;
    mem_latency_cnt = 0;
    tlb_pending = false;
    tlb_pending_vaddr = 0;
  }

  // set input for 1st pipeline stage (IFU)
  icache_hw.io.in.pc = in->fetch_address;
  icache_hw.io.in.ifu_req_valid = in->icache_read_valid;

  // set input for 2nd pipeline stage (IFU)
  icache_hw.io.in.ifu_resp_ready = true;

  // set input for 2nd pipeline stage (MMU): convert local AbstractMmu result
  icache_hw.io.in.ppn = 0;
  icache_hw.io.in.ppn_valid = false;
  icache_hw.io.in.page_fault = false;
  bool mmu_req_valid = false;
  uint32_t mmu_vaddr = 0;
  bool mmu_req_from_new = false;
  if (tlb_pending) {
    mmu_req_valid = true;
    mmu_vaddr = tlb_pending_vaddr;
  } else if (icache_hw.io.out.ifu_req_ready && icache_hw.io.in.ifu_req_valid) {
    mmu_req_valid = true;
    mmu_vaddr = in->fetch_address;
    mmu_req_from_new = true;
  } else if (!icache_hw.io.out.ifu_req_ready && valid_reg) {
    // replay translation while IFU is stalled
    mmu_req_valid = true;
    mmu_vaddr = current_vaddr_reg;
  }
  if (!in->run_comb_only && mmu_req_valid && mmu_model != nullptr &&
      !in->refetch) {
    uint32_t p_addr = 0;
    AbstractMmu::Result ret = mmu_model->translate(p_addr, mmu_vaddr, 0, in->csr_status);
    if (ret == AbstractMmu::Result::OK) {
      icache_hw.io.in.ppn = p_addr >> 12;
      icache_hw.io.in.ppn_valid = true;
      tlb_clear_pending_comb = true;
    } else if (ret == AbstractMmu::Result::FAULT) {
      // icache_module consumes page_fault only when ppn_valid is asserted.
      icache_hw.io.in.ppn_valid = true;
      icache_hw.io.in.page_fault = true;
      tlb_clear_pending_comb = true;
    } else if (ret == AbstractMmu::Result::RETRY && mmu_req_from_new) {
      tlb_set_pending_comb = true;
      tlb_pending_vaddr = mmu_vaddr;
    }
  }

  // set input for 2nd pipeline stage (Memory)
  if (mem_busy) {
    if (mem_latency_cnt >= ICACHE_MISS_LATENCY) {
      icache_hw.io.in.mem_resp_valid = true;
    } else {
      icache_hw.io.in.mem_resp_valid = false;
    }
    bool mem_resp_valid = icache_hw.io.in.mem_resp_valid;
    if (mem_resp_valid) {
      uint32_t mask = ~(ICACHE_LINE_SIZE - 1);
      uint32_t cacheline_base_addr = icache_hw.io.out.mem_req_addr & mask;
      for (int i = 0; i < ICACHE_LINE_SIZE / 4; i++) {
        icache_hw.io.in.mem_resp_data[i] =
            pmem_read(cacheline_base_addr + (uint32_t)(i << 2));
      }
    }
  } else {
    icache_hw.io.in.mem_req_ready = true;
  }

  icache_hw.comb();

  if (in->run_comb_only) {
    out->icache_read_ready = icache_hw.io.out.ifu_req_ready;
    return;
  }

  bool ifu_resp_valid = icache_hw.io.out.ifu_resp_valid;
  bool ifu_resp_ready = icache_hw.io.in.ifu_resp_ready;
  bool miss = icache_hw.io.out.miss;
  if (ifu_resp_valid && ifu_resp_ready) {
    out->icache_read_complete = true;
    if (miss) {
      std::cout << "[icache_top] WARNING: miss is true when ifu_resp is valid"
                << std::endl;
      std::cout << "[icache_top] sim_time: " << std::dec << sim_time
                << std::endl;
      exit(1);
    }
    out->fetch_pc = current_vaddr_reg;
    uint32_t mask = ICACHE_LINE_SIZE - 1;
    int base_idx = (current_vaddr_reg & mask) / 4;
    for (int i = 0; i < FETCH_WIDTH; i++) {
      if (base_idx + i >= ICACHE_LINE_SIZE / 4) {
        out->fetch_group[i] = INST_NOP;
        out->page_fault_inst[i] = false;
        out->inst_valid[i] = false;
        continue;
      }
      out->fetch_group[i] = icache_hw.io.out.ifu_page_fault
                                ? INST_NOP
                                : icache_hw.io.out.rd_data[i + base_idx];
      out->page_fault_inst[i] = icache_hw.io.out.ifu_page_fault;
      out->inst_valid[i] = true;
    }
  } else {
    out->icache_read_complete = false;
    for (int i = 0; i < FETCH_WIDTH; i++) {
      out->fetch_group[i] = INST_NOP;
      out->page_fault_inst[i] = false;
      out->inst_valid[i] = false;
    }
  }
}

void TrueICacheTop::seq() {
  if (in->reset)
    return;

  icache_hw.seq();

  if (mem_busy) {
    mem_latency_cnt++;
  }
  bool mem_req_ready = !mem_busy;
  bool mem_req_valid = icache_hw.io.out.mem_req_valid;
  if (mem_req_ready && mem_req_valid) {
    mem_busy = true;
    mem_latency_cnt = 0;
    miss_delta++; // Use local delta
  }
  bool mem_resp_valid = icache_hw.io.in.mem_resp_valid;
  bool mem_resp_ready = icache_hw.io.out.mem_resp_ready;
  if (mem_resp_valid && mem_resp_ready) {
    mem_busy = false;
    icache_hw.io.in.mem_resp_valid = false;
  }

  bool ifu_resp_valid = icache_hw.io.out.ifu_resp_valid;
  bool ifu_resp_ready = icache_hw.io.in.ifu_resp_ready;

  if (icache_hw.io.in.ifu_req_valid && icache_hw.io.out.ifu_req_ready) {
    current_vaddr_reg = in->fetch_address;
    valid_reg = true;
    access_delta++; // Use local delta
  } else if (ifu_resp_valid && ifu_resp_ready) {
    valid_reg = false;
  }

  if (tlb_set_pending_comb) {
    tlb_pending = true;
  }
  if (tlb_clear_pending_comb) {
    tlb_pending = false;
  }
}

// --- SimpleICacheTop Implementation ---

void SimpleICacheTop::comb() {
  out->icache_read_ready_2 = false;
  out->icache_read_complete_2 = false;
  out->fetch_pc_2 = 0;
  for (int i = 0; i < FETCH_WIDTH; i++) {
    out->fetch_group_2[i] = INST_NOP;
    out->page_fault_inst_2[i] = false;
    out->inst_valid_2[i] = false;
  }

  static IcacheBlockingPtwPort ptw_port;
  if (mmu_model == nullptr && ctx != nullptr) {
#ifdef CONFIG_TLB_MMU
    mmu_model =
        new TlbMmu(ctx, ptw_mem_port ? ptw_mem_port : &ptw_port, ITLB_ENTRIES);
    if (ptw_walk_port != nullptr) {
      mmu_model->set_ptw_walk_port(ptw_walk_port);
    }
#else
    mmu_model = new SimpleMmu(ctx, nullptr);
#endif
  }

  pend_on_retry_comb = false;
  resp_fire_comb = false;

  if (in->reset) {
    DEBUG_LOG("[icache] reset\n");
    if (mmu_model != nullptr) {
      mmu_model->flush();
    }
    satp_seen = false;
    pending_req_valid = false;
    pending_fetch_addr = 0;
    out->icache_read_ready = true;
    out->icache_read_complete = false;
    return;
  }

  out->icache_read_complete = false;
  out->icache_read_ready = !pending_req_valid;
  out->fetch_pc = pending_req_valid ? pending_fetch_addr : in->fetch_address;
  for (int i = 0; i < FETCH_WIDTH; i++) {
    out->fetch_group[i] = INST_NOP;
    out->page_fault_inst[i] = false;
    out->inst_valid[i] = false;
  }

  uint32_t cur_satp = in->csr_status ? static_cast<uint32_t>(in->csr_status->satp) : 0;
  if (!satp_seen || cur_satp != last_satp || in->refetch) {
    if (mmu_model != nullptr) {
      mmu_model->flush();
    }
    satp_seen = true;
    last_satp = cur_satp;
    pending_req_valid = false;
    pending_fetch_addr = 0;
    out->icache_read_ready = true;
  }

  if (in->run_comb_only) {
    return;
  }

  if (!pending_req_valid && !in->icache_read_valid) {
    return;
  }

  const uint32_t fetch_addr = pending_req_valid ? pending_fetch_addr : in->fetch_address;
  out->fetch_pc = fetch_addr;
  out->icache_read_complete = true;
  for (int i = 0; i < FETCH_WIDTH; i++) {
      uint32_t v_addr = fetch_addr + (i * 4);
      uint32_t p_addr = 0;

      if (v_addr / ICACHE_LINE_SIZE != (fetch_addr) / ICACHE_LINE_SIZE) {
        out->fetch_group[i] = INST_NOP;
        out->page_fault_inst[i] = false;
        out->inst_valid[i] = false;
        continue;
      }
      out->inst_valid[i] = true;

      AbstractMmu::Result ret = AbstractMmu::Result::FAULT;
      if (mmu_model != nullptr) {
        ret = mmu_model->translate(p_addr, v_addr, 0, in->csr_status);
        for (int spin = 0; spin < 8 && ret == AbstractMmu::Result::RETRY; spin++) {
          ret = mmu_model->translate(p_addr, v_addr, 0, in->csr_status);
        }
      }

      if (ret == AbstractMmu::Result::RETRY) {
        out->icache_read_complete = false;
        out->icache_read_ready = false;
        if (!pending_req_valid) {
          pend_on_retry_comb = true;
        }
        for (int j = i; j < FETCH_WIDTH; j++) {
          out->fetch_group[j] = INST_NOP;
          out->page_fault_inst[j] = false;
          out->inst_valid[j] = false;
        }
        break;
      }

      if (ret == AbstractMmu::Result::FAULT) {
        out->page_fault_inst[i] = true;
        out->fetch_group[i] = INST_NOP;
        for (int j = i + 1; j < FETCH_WIDTH; j++) {
          out->fetch_group[j] = INST_NOP;
          out->page_fault_inst[j] = false;
          out->inst_valid[j] = false;
        }
        break;
      }

      out->page_fault_inst[i] = false;
      out->fetch_group[i] = pmem_read(p_addr);

      if (DEBUG_PRINT) {
        uint32_t satp = in->csr_status ? static_cast<uint32_t>(in->csr_status->satp) : 0;
        uint32_t privilege =
            in->csr_status ? static_cast<uint32_t>(in->csr_status->privilege) : 0;
        printf("[icache] vaddr: %08x -> paddr: %08x, inst: %08x, satp: %x, "
               "priv: %d\n",
               v_addr, p_addr, out->fetch_group[i], satp, privilege);
      }
  }

  if (out->icache_read_complete) {
    resp_fire_comb = true;
  }
}

void SimpleICacheTop::set_ptw_mem_port(PtwMemPort *port) {
  ptw_mem_port = port;
  if (mmu_model != nullptr) {
    mmu_model->set_ptw_mem_port(port);
  }
}
void SimpleICacheTop::set_ptw_walk_port(PtwWalkPort *port) {
  ptw_walk_port = port;
  if (mmu_model != nullptr) {
    mmu_model->set_ptw_walk_port(port);
  }
}

void SimpleICacheTop::seq() {
  if (in->reset) {
    pending_req_valid = false;
    pending_fetch_addr = 0;
    pend_on_retry_comb = false;
    resp_fire_comb = false;
    return;
  }

  if (in->refetch) {
    pending_req_valid = false;
    pending_fetch_addr = 0;
    pend_on_retry_comb = false;
    resp_fire_comb = false;
    return;
  }

  if (pend_on_retry_comb) {
    pending_req_valid = true;
    pending_fetch_addr = in->fetch_address;
    access_delta++;
  }

  if (resp_fire_comb) {
    pending_req_valid = false;
    pending_fetch_addr = 0;
  }
}

// --- Factory ---

ICacheTop *get_icache_instance() {
  static std::unique_ptr<ICacheTop> instance = nullptr;
  if (!instance) {
#ifdef USE_IDEAL_ICACHE
    instance = std::make_unique<SimpleICacheTop>();
#else
    instance = std::make_unique<TrueICacheTop>(icache);
#endif
  }
  return instance.get();
}
