#include "diff.h"
#include "Csr.h"
#include "DcacheConfig.h"
#include "DiffMemTrace.h"
#include "RISCV.h"
#include "config.h"
#include "util.h"

#include <cstddef>
#include <cstring>
#include <iostream>

CPU_state dut_cpu;
RefCpu ref_cpu;

namespace {
inline uint32_t sign_extend_12(uint32_t imm12) {
  return static_cast<uint32_t>(static_cast<int32_t>(imm12 << 20) >> 20);
}

inline uint32_t sign_extend_bits(uint32_t value, int bits) {
  const int shift = 32 - bits;
  return static_cast<uint32_t>(static_cast<int32_t>(value << shift) >> shift);
}

enum class MemInstKind : uint8_t {
  None = 0,
  Load,
  Store,
  Amo,
};

MemInstKind classify_mem_inst(uint32_t inst) {
  const uint32_t opcode = inst & 0x7fu;
  switch (opcode) {
  case 0x03: // load
  case 0x07: // floating load
    return MemInstKind::Load;
  case 0x23: // store
  case 0x27: // floating store
    return MemInstKind::Store;
  case 0x2f: // AMO
    return MemInstKind::Amo;
  default:
    return MemInstKind::None;
  }
}

const char *mem_inst_kind_name(MemInstKind kind) {
  switch (kind) {
  case MemInstKind::Load:
    return "load";
  case MemInstKind::Store:
    return "store";
  case MemInstKind::Amo:
    return "amo";
  default:
    return "none";
  }
}

struct DecodedMemInst {
  MemInstKind kind = MemInstKind::None;
  uint32_t opcode = 0;
  uint32_t funct3 = 0;
  uint32_t rs1 = 0;
  uint32_t rs2 = 0;
  uint32_t rd = 0;
  uint32_t imm = 0;
  uint32_t width_bytes = 0;
  bool sign_extend = false;
  bool is_unsigned_load = false;
  bool is_lr = false;
  bool is_sc = false;
  bool valid = false;
};

DecodedMemInst decode_mem_inst(uint32_t inst) {
  DecodedMemInst d{};
  d.kind = classify_mem_inst(inst);
  d.opcode = inst & 0x7fu;
  d.funct3 = (inst >> 12) & 0x7u;
  d.rd = (inst >> 7) & 0x1fu;
  d.rs1 = (inst >> 15) & 0x1fu;
  d.rs2 = (inst >> 20) & 0x1fu;
  if (d.kind == MemInstKind::Load) {
    d.imm = sign_extend_12((inst >> 20) & 0xfffu);
    d.valid = true;
    switch (d.funct3) {
    case 0:
      d.width_bytes = 1;
      d.sign_extend = true;
      break;
    case 1:
      d.width_bytes = 2;
      d.sign_extend = true;
      break;
    case 2:
      d.width_bytes = 4;
      d.sign_extend = true;
      break;
    case 4:
      d.width_bytes = 1;
      d.is_unsigned_load = true;
      break;
    case 5:
      d.width_bytes = 2;
      d.is_unsigned_load = true;
      break;
    default:
      break;
    }
    return d;
  }

  if (d.kind == MemInstKind::Store) {
    const uint32_t imm =
        (((inst >> 25) & 0x7fu) << 5) | ((inst >> 7) & 0x1fu);
    d.imm = sign_extend_12(imm);
    d.valid = true;
    switch (d.funct3) {
    case 0:
      d.width_bytes = 1;
      break;
    case 1:
      d.width_bytes = 2;
      break;
    case 2:
      d.width_bytes = 4;
      break;
    default:
      break;
    }
    return d;
  }

  if (d.kind == MemInstKind::Amo) {
    d.imm = 0;
    d.valid = true;
    d.width_bytes = 4;
    const uint32_t funct5 = (inst >> 27) & 0x1fu;
    d.is_lr = (funct5 == 0x02u);
    d.is_sc = (funct5 == 0x03u);
  }
  return d;
}

const char *mem_access_name(const DecodedMemInst &d) {
  if (d.kind == MemInstKind::Load) {
    switch (d.funct3) {
    case 0:
      return "lb";
    case 1:
      return "lh";
    case 2:
      return "lw";
    case 4:
      return "lbu";
    case 5:
      return "lhu";
    default:
      return "load?";
    }
  }
  if (d.kind == MemInstKind::Store) {
    switch (d.funct3) {
    case 0:
      return "sb";
    case 1:
      return "sh";
    case 2:
      return "sw";
    default:
      return "store?";
    }
  }
  if (d.kind == MemInstKind::Amo) {
    const uint32_t funct5 = (dut_cpu.instruction >> 27) & 0x1fu;
    switch (funct5) {
    case 0x00:
      return "amoadd.w";
    case 0x01:
      return "amoswap.w";
    case 0x02:
      return "lr.w";
    case 0x03:
      return "sc.w";
    case 0x04:
      return "amoxor.w";
    case 0x08:
      return "amoor.w";
    case 0x0c:
      return "amoand.w";
    case 0x10:
      return "amomin.w";
    case 0x14:
      return "amomax.w";
    case 0x18:
      return "amominu.w";
    case 0x1c:
      return "amomaxu.w";
    default:
      return "amo?";
    }
  }
  return "none";
}

bool paddr_word_in_range(uint32_t paddr) {
  const uint64_t word_idx = static_cast<uint64_t>(paddr) >> 2;
  return word_idx < static_cast<uint64_t>(PHYSICAL_MEMORY_LENGTH);
}

bool paddr_byte_range_in_range(uint32_t paddr, uint32_t bytes) {
  if (bytes == 0) {
    return false;
  }
  const uint64_t last_byte = static_cast<uint64_t>(paddr) + bytes - 1u;
  return (last_byte >> 2) < static_cast<uint64_t>(PHYSICAL_MEMORY_LENGTH);
}

uint32_t load_data_from_mem_words(const uint32_t *mem, uint32_t paddr,
                                  const DecodedMemInst &d) {
  if (d.width_bytes == 0 || !paddr_byte_range_in_range(paddr, d.width_bytes)) {
    return 0;
  }
  const uint32_t byte_off = paddr & 0x3u;
  uint64_t merged = mem[paddr >> 2];
  if (byte_off + d.width_bytes > 4) {
    merged |= static_cast<uint64_t>(mem[(paddr >> 2) + 1]) << 32;
  }
  uint32_t raw = static_cast<uint32_t>(merged >> (byte_off * 8));
  switch (d.width_bytes) {
  case 1:
    raw &= 0xffu;
    return (d.sign_extend && !d.is_unsigned_load)
               ? sign_extend_bits(raw, 8)
               : raw;
  case 2:
    raw &= 0xffffu;
    return (d.sign_extend && !d.is_unsigned_load)
               ? sign_extend_bits(raw, 16)
               : raw;
  case 4:
  default:
    return raw;
  }
}

void dump_backing_memory_window(const char *tag, const uint32_t *mem,
                                uint32_t paddr) {
  const uint32_t line_base =
      paddr & ~(static_cast<uint32_t>(DCACHE_LINE_BYTES) - 1u);
  std::printf("[DIFF][MEMWIN][%s] line_base=0x%08x\n", tag, line_base);
  for (int row = 0; row < DCACHE_LINE_WORDS; row += 4) {
    const uint32_t addr = line_base + static_cast<uint32_t>(row * 4);
    std::printf("[DIFF][MEMWIN][%s] +0x%02x:", tag, row * 4);
    for (int col = 0; col < 4; ++col) {
      const uint32_t cur_addr = addr + static_cast<uint32_t>(col * 4);
      if (paddr_word_in_range(cur_addr)) {
        std::printf(" %08x", mem[cur_addr >> 2]);
      } else {
        std::printf(" --------");
      }
    }
    std::printf("\n");
  }
}

void dump_dcache_line_snapshot(uint32_t paddr) {
  const AddrFields f = decode(paddr);
  const uint32_t line_base =
      paddr & ~(static_cast<uint32_t>(DCACHE_LINE_BYTES) - 1u);
  std::printf(
      "[DIFF][DCACHE_LINE] paddr=0x%08x line_base=0x%08x set=%u tag=0x%x "
      "word_off=%u\n",
      paddr, line_base, f.set_idx, f.tag, f.word_off);

  for (int way = 0; way < DCACHE_WAYS; ++way) {
    const bool valid = valid_array[f.set_idx][way];
    const uint32_t tag = tag_array[f.set_idx][way];
    const bool match = valid && (tag == f.tag);
    std::printf(
        "[DIFF][DCACHE_LINE][WAY %d] valid=%d tag=0x%x dirty=%d match=%d "
        "data=[",
        way, static_cast<int>(valid), tag,
        static_cast<int>(dirty_array[f.set_idx][way]), static_cast<int>(match));
    for (int w = 0; w < DCACHE_LINE_WORDS; ++w) {
      std::printf("%s%08x", (w == 0) ? "" : " ",
                  data_array[f.set_idx][way][w]);
    }
    std::printf("]\n");
  }
}

void dump_focus_line_pending_sources(uint32_t paddr) {
  const uint32_t line_base =
      paddr & ~(static_cast<uint32_t>(DCACHE_LINE_BYTES) - 1u);
  bool found_match = false;

  for (int i = 0; i < DCACHE_MSHR_ENTRIES; ++i) {
    const auto &e = mshr_entries[i];
    if (!e.valid) {
      continue;
    }
    const uint32_t entry_line_base = get_addr(e.index, e.tag, 0);
    if (entry_line_base != line_base) {
      continue;
    }
    found_match = true;
    std::printf(
        "[DIFF][FOCUS][MSHR] idx=%d issued=%d fill=%d merged_store_dirty=%d "
        "line=0x%08x\n",
        i, static_cast<int>(e.issued), static_cast<int>(e.fill),
        static_cast<int>(e.merged_store_dirty), entry_line_base);
    if (e.merged_store_dirty) {
      std::printf("[DIFF][FOCUS][MSHR] merged_data=[");
      for (int w = 0; w < DCACHE_LINE_WORDS; ++w) {
        std::printf("%s%08x", (w == 0) ? "" : " ", e.merged_store_data[w]);
      }
      std::printf("]\n");
      std::printf("[DIFF][FOCUS][MSHR] merged_strb=[");
      for (int w = 0; w < DCACHE_LINE_WORDS; ++w) {
        std::printf("%s%01x", (w == 0) ? "" : " ", e.merged_store_strb[w]);
      }
      std::printf("]\n");
    }
  }

  for (size_t i = 0; i < write_buffer.size(); ++i) {
    const auto *e = write_buffer.at(i);
    if (e == nullptr || !e->valid) {
      continue;
    }
    if (!cache_line_match(e->addr, paddr)) {
      continue;
    }
    found_match = true;
    std::printf("[DIFF][FOCUS][WB] idx=%zu send=%d line=0x%08x data=[", i,
                static_cast<int>(e->send), e->addr);
    for (int w = 0; w < DCACHE_LINE_WORDS; ++w) {
      std::printf("%s%08x", (w == 0) ? "" : " ", e->data[w]);
    }
    std::printf("]\n");
  }

  if (!found_match) {
    std::printf("[DIFF][FOCUS] no MSHR/WB entry matches line 0x%08x\n",
                line_base);
  }
}

void dump_mem_access_analysis(const DecodedMemInst &d) {
  if (!d.valid || d.kind == MemInstKind::None) {
    return;
  }

  const uint32_t rs1_val = dut_cpu.gpr[d.rs1];
  const uint32_t rs2_val = dut_cpu.gpr[d.rs2];
  const uint32_t vaddr = rs1_val + d.imm;
  const bool has_sideband_paddr =
      !dut_cpu.page_fault_load && !dut_cpu.page_fault_store &&
      ((d.kind != MemInstKind::Load) || dut_cpu.store || dut_cpu.store_addr != 0);
  const uint32_t sideband_paddr = dut_cpu.store_addr;

  std::printf(
      "[DIFF][MEMDECODE] op=%s inst=0x%08x rs1=%s(0x%08x) rs2=%s(0x%08x) "
      "rd=%s imm=0x%08x(%d) width=%uB\n",
      mem_access_name(d), dut_cpu.instruction, reg_names[d.rs1].c_str(),
      rs1_val, reg_names[d.rs2].c_str(), rs2_val, reg_names[d.rd].c_str(),
      d.imm, static_cast<int32_t>(d.imm), d.width_bytes);
  std::printf("[DIFF][MEMDECODE] effective_vaddr=0x%08x\n", vaddr);

  if (has_sideband_paddr) {
    std::printf("[DIFF][MEMDECODE] dut_sideband_paddr=0x%08x", sideband_paddr);
    if (sideband_paddr != vaddr) {
      std::printf(
          " (differs from decoded vaddr; for load mismatch this often means "
          "store_addr is stale sideband from an older store/amo)");
    }
    std::printf("\n");
  } else {
    std::printf(
        "[DIFF][MEMDECODE] dut_sideband_paddr unavailable for this instruction\n");
  }

  if (d.kind == MemInstKind::Load && paddr_byte_range_in_range(vaddr, d.width_bytes)) {
    const uint32_t dut_backing = load_data_from_mem_words(p_memory, vaddr, d);
    const uint32_t ref_backing = load_data_from_mem_words(ref_cpu.memory, vaddr, d);
    std::printf(
        "[DIFF][LOAD_EVAL] decoded_addr=0x%08x dut_backing_result=0x%08x "
        "ref_backing_result=0x%08x dut_rd(%s)=0x%08x ref_rd=0x%08x\n",
        vaddr, dut_backing, ref_backing, reg_names[d.rd].c_str(),
        dut_cpu.gpr[d.rd], ref_cpu.state.gpr[d.rd]);
  }

  if (d.kind == MemInstKind::Store) {
    std::printf("[DIFF][STORE_EVAL] store_data=0x%08x store_strb=0x%x\n",
                dut_cpu.store_data, dut_cpu.store_strb);
  }

  if (has_sideband_paddr && paddr_word_in_range(sideband_paddr)) {
    std::printf("[DIFF][MEM] p_memory[0x%08x]=0x%08x ref_mem=0x%08x\n",
                sideband_paddr, p_memory[sideband_paddr >> 2],
                ref_cpu.memory[sideband_paddr >> 2]);
    dump_backing_memory_window("DUT_BACKING", p_memory, sideband_paddr);
    dump_backing_memory_window("REF_BACKING", ref_cpu.memory, sideband_paddr);
    dump_focus_line_pending_sources(sideband_paddr);
  }
}

void print_likely_cause_hint(const DecodedMemInst &d) {
  if (!d.valid || d.kind == MemInstKind::None) {
    return;
  }

  const uint32_t rs1_val = dut_cpu.gpr[d.rs1];
  const uint32_t vaddr = rs1_val + d.imm;
  std::printf("[DIFF][HINT] ");
  if (d.kind == MemInstKind::Load) {
    std::printf(
        "current mismatch is on a %s to rd=%s from decoded vaddr=0x%08x. ",
        mem_access_name(d), reg_names[d.rd].c_str(), vaddr);
    if (dut_cpu.store_addr != vaddr) {
      std::printf(
          "The previously printed store_addr-focused memory dump may belong to "
          "an older store/amo sideband, so prioritize the decoded vaddr/load "
          "result when judging root cause. ");
    }
    std::printf(
        "Most likely causes are load return-path issues: wrong WB/MSHR/DCache "
        "bypass source, stale replay/fill ordering, or wrong byte/half/word "
        "selection/sign-extension.");
    std::printf("\n");
    return;
  }

  if (d.kind == MemInstKind::Store) {
    std::printf(
        "current mismatch happened around a %s to decoded vaddr=0x%08x. Check "
        "store_strb/store_data packing and whether DCache/MSHR/WB merged the "
        "store into the correct word before refill/eviction.\n",
        mem_access_name(d), vaddr);
    return;
  }

  std::printf(
      "current mismatch happened around %s. Check LR/SC or AMO read-modify-"
      "write ordering, especially interaction with reservation state and "
      "merged_store_data in matching MSHR/WB entries.\n",
      mem_access_name(d));
}

void dump_mem_subsystem_snapshot(bool has_focus_paddr, uint32_t focus_paddr) {
  for (int i = 0; i < DCACHE_MSHR_ENTRIES; ++i) {
    const auto &e = mshr_entries[i];
    
    std::printf("[DIFF][MSHR] idx=%d v=%d issued=%d fill=%d set=%u tag=0x%x "
                "line=0x%08x\n",
                i, static_cast<int>(e.valid), static_cast<int>(e.issued),
                static_cast<int>(e.fill), e.index, e.tag,
                get_addr(e.index, e.tag, 0));
  }
  if(write_buffer.size() == 0) {
    std::printf("[DIFF][WB] empty\n");
  }
  for (size_t i = 0; i < write_buffer.size(); ++i) {
    const auto *e = write_buffer.at(i);
    if (e == nullptr) {
      printf("[DIFF][WB] idx=%zu empty\n", i);
      continue;
    }
    std::printf("[DIFF][WB] addr=0x%08x valid=%d send=%d data=\n", e->addr, static_cast<int>(e->valid),
                static_cast<int>(e->send));
    for(int w = 0; w < DCACHE_LINE_WORDS; ++w) {
      std::printf("%08x ",e->data[w]);
    }
  }

  if (has_focus_paddr) {
    dump_dcache_line_snapshot(focus_paddr);
  }
}

void dump_code_line_snapshot(const char *tag, uint32_t pc) {
  const uint32_t line_base =
      pc & ~(static_cast<uint32_t>(ICACHE_LINE_SIZE) - 1u);
  const uint32_t start_idx = line_base >> 2;
  const uint32_t word_off = (pc - line_base) >> 2;
  std::printf(
      "[DIFF][ICACHE_LINE][%s] pc=0x%08x line_base=0x%08x word_off=%u\n", tag,
      pc, line_base, word_off);
  for (int row = 0; row < ICACHE_WORD_NUM; row += 4) {
    std::printf("[DIFF][ICACHE_LINE][%s][DUT] +0x%02x: %08x %08x %08x %08x\n",
                tag, row * 4, p_memory[start_idx + row + 0],
                p_memory[start_idx + row + 1], p_memory[start_idx + row + 2],
                p_memory[start_idx + row + 3]);
    std::printf("[DIFF][ICACHE_LINE][%s][REF] +0x%02x: %08x %08x %08x %08x\n",
                tag, row * 4, ref_cpu.memory[start_idx + row + 0],
                ref_cpu.memory[start_idx + row + 1],
                ref_cpu.memory[start_idx + row + 2],
                ref_cpu.memory[start_idx + row + 3]);
  }
}

} // namespace

// relocate the init_difftest function to avoid multiple definition error
void init_difftest(int img_size) {
  ref_cpu.init(0);
  std::memcpy(ref_cpu.memory + 0x80000000 / 4, p_memory + 0x80000000 / 4,
              img_size);
  ref_cpu.memory[0x10000004 / 4] = 0x00006000; // 和进入 OpenSBI 相关
  ref_cpu.memory[uint32_t(0x0 / 4)] = 0xf1402573;
  ref_cpu.memory[uint32_t(0x4 / 4)] = 0x83e005b7;
  ref_cpu.memory[uint32_t(0x8 / 4)] = 0x800002b7;
  ref_cpu.memory[uint32_t(0xc / 4)] = 0x00028067;
}

void init_diff_ckpt(CPU_state ckpt_state, uint32_t *ckpt_memory) {
  std::cout << "Restore for ref cpu " << std::endl;
  ref_cpu.init(0);
  ref_cpu.state = ckpt_state;
  ref_cpu.privilege = RISCV_MODE_U;

  std::memcpy(ref_cpu.memory, ckpt_memory,
              (uint64_t)PHYSICAL_MEMORY_LENGTH * sizeof(uint32_t));

  // Keep checkpoint bootstrap aligned with RefCpu::exec(): only probe a
  // translation when SATP is active, and do not require the restored PC to be
  // immediately translatable at init time.
  if ((ref_cpu.state.csr[csr_satp] & 0x80000000u) != 0 &&
      ref_cpu.privilege != RISCV_MODE_M) {
    uint32_t p_addr = 0;
    (void)ref_cpu.va2pa(p_addr, ref_cpu.state.pc, 0);
  }
}

void get_state(CPU_state &dut_state, uint8_t &privilege, uint32_t *dut_memory) {
  dut_state = ref_cpu.state;
  privilege = ref_cpu.privilege;
  memcpy(dut_memory, ref_cpu.memory,
         (uint64_t)PHYSICAL_MEMORY_LENGTH * sizeof(uint32_t));
}

static void checkregs() {
  int i;

  if (ref_cpu.state.pc != dut_cpu.pc)
    goto fault;

  // 如果没有指令缺页异常，且指令不匹配，报错
  if (!ref_cpu.page_fault_inst && ref_cpu.Instruction != dut_cpu.instruction)
    goto fault;

  if (ref_cpu.page_fault_inst != dut_cpu.page_fault_inst)
    goto fault;
  if (ref_cpu.page_fault_load != dut_cpu.page_fault_load)
    goto fault;
  if (ref_cpu.page_fault_store != dut_cpu.page_fault_store)
    goto fault;

  // 通用寄存器
  for (i = 0; i < 32; i++) {
    if (ref_cpu.state.gpr[i] != dut_cpu.gpr[i])
      goto fault;
  }

  // csr
  for (i = 0; i < CSR_NUM; i++) {
    if (ref_cpu.state.csr[i] != dut_cpu.csr[i])
      goto fault;
  }

  if (ref_cpu.state.store) {
    if (dut_cpu.store != ref_cpu.state.store)
      goto fault;

    if (dut_cpu.store_data != ref_cpu.state.store_data)
      goto fault;

    if (dut_cpu.store_addr != ref_cpu.state.store_addr)
      goto fault;
  }

  return;

fault:
  cout << "Difftest: error" << endl;
  cout << "cycle: " << dec << sim_time << endl;

  auto print_mismatch = [](const char *name, uint32_t ref, uint32_t dut) {
    printf("%10s:\t%08x\t%08x%s\n", name, ref, dut,
           (ref != dut ? "\t Error" : ""));
  };

  printf("        PC:\t%08x\t%08x\n", ref_cpu.state.pc, dut_cpu.pc);
  cout << "\t\tReference\tDut" << endl;
  for (int i = 0; i < 32; i++) {
    print_mismatch(reg_names[i].c_str(), ref_cpu.state.gpr[i], dut_cpu.gpr[i]);
  }

  cout << endl;
  for (int i = 0; i < CSR_NUM; i++) {
    print_mismatch(csr_names[i].c_str(), ref_cpu.state.csr[i], dut_cpu.csr[i]);
  }

  cout << endl;
  print_mismatch("store", ref_cpu.state.store, dut_cpu.store);
  print_mismatch("data", ref_cpu.state.store_data, dut_cpu.store_data);
  print_mismatch("addr", ref_cpu.state.store_addr, dut_cpu.store_addr);
  print_mismatch("strb", ref_cpu.state.store_strb, dut_cpu.store_strb);

  printf("Ref Inst: %08x\tDUT Inst: %08x\n", ref_cpu.Instruction,
         dut_cpu.instruction);
  std::printf("Commit PC: 0x%08x\tDUT next PC: 0x%08x\tREF next PC: 0x%08x\n",
              dut_cpu.commit_pc, dut_cpu.pc, ref_cpu.state.pc);
  const DecodedMemInst decoded = decode_mem_inst(dut_cpu.instruction);
  const MemInstKind mem_kind = decoded.kind;
  bool has_focus_paddr = false;
  uint32_t focus_paddr = 0;
  if (mem_kind != MemInstKind::None && !dut_cpu.page_fault_load &&
      !dut_cpu.page_fault_store) {
    focus_paddr = dut_cpu.store_addr;
    has_focus_paddr = true;
    std::printf(
        "[DIFF][MEM] kind=%s paddr=0x%08x (from dut sideband store_addr)\n",
        mem_inst_kind_name(mem_kind), focus_paddr);
    if (!paddr_word_in_range(focus_paddr)) {
      std::printf(
          "[DIFF][MEM] paddr 0x%08x out of backing range, skip p_memory "
          "word dump\n",
          focus_paddr);
    }
  } else if (mem_kind != MemInstKind::None) {
    std::printf(
        "[DIFF][MEM] kind=%s but page fault is set (load=%d store=%d), "
        "physical address unavailable\n",
        mem_inst_kind_name(mem_kind), static_cast<int>(dut_cpu.page_fault_load),
        static_cast<int>(dut_cpu.page_fault_store));
  }
  dump_mem_access_analysis(decoded);
  print_likely_cause_hint(decoded);
  dump_code_line_snapshot("commit_pc", dut_cpu.commit_pc);
#if defined(LOG_ENABLE) && defined(LOG_LSU_MEM_ENABLE)
  diff_mem_trace::dump_recent();
#endif
  dump_mem_subsystem_snapshot(has_focus_paddr, focus_paddr);

  Assert(0 && "Difftest: Register or Memory mismatch detected.");
}

void difftest_skip() {
  ref_cpu.dut_expect_pf_inst = dut_cpu.page_fault_inst;
  ref_cpu.dut_expect_pf_load = dut_cpu.page_fault_load;
  ref_cpu.dut_expect_pf_store = dut_cpu.page_fault_store;
  ref_cpu.exec();
  for (int i = 0; i < 32; i++) {
    ref_cpu.state.gpr[i] = dut_cpu.gpr[i];
  }
}

void difftest_step(bool check) {
  ref_cpu.dut_expect_pf_inst = dut_cpu.page_fault_inst;
  ref_cpu.dut_expect_pf_load = dut_cpu.page_fault_load;
  ref_cpu.dut_expect_pf_store = dut_cpu.page_fault_store;
  ref_cpu.exec();
  if (check)
    checkregs();
}
