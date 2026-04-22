#pragma once

#include "IO.h"
#include <cstdint>

// 输入信号 (来自各个流水级)
typedef struct {
  RobCommitIO *rob_commit;
  RobBroadcastIO *rob_bcast;
  DecBroadcastIO *dec_bcast;
  CsrStatusIO *csr_status;
  DisLsuIO *dis2lsu;
  ExeLsuIO *exe2lsu;
  PeripheralRespIO *peripheral_resp;
  // MemRespIO *dcache_resp;
  // MemReadyIO *dcache_wready;

  DcacheLsuIO *dcache2lsu;
} LsuIn;

// 输出信号 (发送给各个流水级)
typedef struct {
  LsuDisIO *lsu2dis;
  LsuRobIO *lsu2rob;
  LsuExeIO *lsu2exe;
  PeripheralReqIO *peripheral_req;
  // MemReqIO *dcache_req;
  // MemReqIO *dcache_wreq;
  LsuDcacheIO *lsu2dcache;
} LsuOut;

inline int lsu_get_mem_width(int func3) {
  switch (func3 & 0b11) {
  case 0b00:
    return 1; // Byte
  case 0b01:
    return 2; // Half
  case 0b10:
    return 4; // Word
  default:
    return 4;
  }
}

inline uint32_t lsu_extract_data(uint32_t raw_mem_val, uint32_t addr,
                                 int func3) {
  int bit_offset = (addr & 0x3) * 8;
  uint32_t result = 0;
  uint32_t shifted = raw_mem_val >> bit_offset;

  switch (func3) {
  case 0b000: // LB (Sign Ext)
    result = shifted & 0xFF;
    if (result & 0x80) {
      result |= 0xFFFFFF00;
    }
    break;
  case 0b001: // LH (Sign Ext)
    result = shifted & 0xFFFF;
    if (result & 0x8000) {
      result |= 0xFFFF0000;
    }
    break;
  case 0b010: // LW
    result = shifted;
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

inline uint32_t lsu_merge_data_to_word(uint32_t old_word, uint32_t new_data,
                                       uint32_t addr, int func3) {
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
