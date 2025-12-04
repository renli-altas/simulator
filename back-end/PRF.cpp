#include "TOP.h"
#include "config.h"
#include <IO.h>
#include <PRF.h>
#include <cstring>
#include <iostream>
#include <util.h>
extern Back_Top back;

void PRF::init() {
  for (int i = 0; i < ISSUE_WAY; i++)
    io.prf2exe->ready[i] = true;
}

void PRF::comb_br_check() {
  // 根据分支结果向前端返回信息

  io.prf2dec->mispred = false;

  if (!io.rob_bcast->flush) {
    int inst_idx = 0;
    for (int i = 0; i < BRU_NUM; i++) {
      int iq_br = IQ_BR0 + i;
      if (inst_r[iq_br].valid && inst_r[iq_br].uop.mispred) {
        if (!io.prf2dec->mispred || inst_r[iq_br].uop.inst_idx < inst_idx) {
          io.prf2dec->mispred = true;
          io.prf2dec->redirect_pc = inst_r[iq_br].uop.pc_next;
          io.prf2dec->redirect_rob_idx = inst_r[iq_br].uop.rob_idx;
          io.prf2dec->br_tag = inst_r[iq_br].uop.tag;
          inst_idx = inst_r[iq_br].uop.inst_idx;
          if (LOG)
            cout << "PC " << hex << inst_r[iq_br].uop.pc
                 << " misprediction redirect_pc 0x" << hex
                 << io.prf2dec->redirect_pc << endl;
        }
      }
    }
  }
}

void PRF::comb_read() {
  // bypass
  for (int i = 0; i < ISSUE_WAY; i++) {
    io.prf2exe->iss_entry[i] = io.iss2prf->iss_entry[i];
    Inst_entry *entry = &io.prf2exe->iss_entry[i];

    if (entry->valid) {
      if (entry->uop.src1_en) {
        entry->uop.src1_rdata = reg_file[entry->uop.src1_preg];
        for (int j = 0; j < ALU_NUM + 1; j++) {
          if (inst_r[j].valid && inst_r[j].uop.dest_en &&
              inst_r[j].uop.dest_preg == entry->uop.src1_preg)
            entry->uop.src1_rdata = inst_r[j].uop.result;
        }

        for (int j = 0; j < ALU_NUM ; j++) {
          if (io.exe2prf->entry[j].valid && io.exe2prf->entry[j].uop.dest_en &&
              io.exe2prf->entry[j].uop.dest_preg == entry->uop.src1_preg)
            entry->uop.src1_rdata = io.exe2prf->entry[j].uop.result;
        }

        if(io.cache2prf->valid && io.cache2prf->uop.dest_preg == entry->uop.src1_preg){
          entry->uop.src1_rdata = load_data;
        }
      }

      if (entry->uop.src2_en) {
        entry->uop.src2_rdata = reg_file[entry->uop.src2_preg];
        for (int j = 0; j < ALU_NUM + 1; j++) {
          if (inst_r[j].valid && inst_r[j].uop.dest_en &&
              inst_r[j].uop.dest_preg == entry->uop.src2_preg)
            entry->uop.src2_rdata = inst_r[j].uop.result;
        }

        for (int j = 0; j < ALU_NUM ; j++) {
          if (io.exe2prf->entry[j].valid && io.exe2prf->entry[j].uop.dest_en &&
              io.exe2prf->entry[j].uop.dest_preg == entry->uop.src2_preg)
            entry->uop.src2_rdata = io.exe2prf->entry[j].uop.result;
        }
        if(io.cache2prf->valid && io.cache2prf->uop.dest_preg == entry->uop.src2_preg){
          entry->uop.src2_rdata = load_data;
        }
      }
    }
  }
}
void PRF::comb_load(){
  if(io.cache2prf->valid){
    int addr = io.cache2prf->uop.src1_rdata + io.cache2prf->uop.imm;
    int size = io.cache2prf->uop.func3 & 0b11;
    int offset = addr & 0b11;


    uint32_t mask = 0;
    uint32_t sign = 0;

    if (io.cache2prf->uop.amoop != AMONONE)
    {
      size = 0b10;
      offset = 0b0;
    }
    uint32_t data = io.cache2prf->data;
    data = data >> (offset * 8);
    if (size == 0) {
      mask = 0xFF;
      if (data & 0x80)
        sign = 0xFFFFFF00;
    } else if (size == 0b01) {
      mask = 0xFFFF;
      if (data & 0x8000)
        sign = 0xFFFF0000;
    } else {
      mask = 0xFFFFFFFF;
    }

    data = data & mask;

    // 有符号数
    if (!(io.cache2prf->uop.func3 & 0b100)) {
      data = data | sign;
    }
    load_data = io.cache2prf->uop.page_fault_load ? io.cache2prf->uop.src1_rdata + io.cache2prf->uop.imm : data;
    back.stq.st2ld_fwd( io.cache2prf->uop.addr, load_data,io.cache2prf->uop.rob_idx);
  }
}
void PRF::comb_complete() {
  for (int i = 0; i < ISSUE_WAY; i++) {
    if (inst_r[i].valid)
      io.prf2rob->entry[i] = inst_r[i];
    else
      io.prf2rob->entry[i].valid = false;
    // if(DCACHE_LOG){
    //   printf("io.prf2rob->entry[%d]: valid:%d inst:0x%08x rob_idx:%d preg:%d\n",i,io.prf2rob->entry[i].valid,io.prf2rob->entry[i].uop.instruction,io.prf2rob->entry[i].uop.rob_idx, io.prf2rob->entry[i].uop.dest_preg);
    // }
  }
}

void PRF::comb_awake() {
  if (inst_r[IQ_LD].valid && inst_r[IQ_LD].uop.dest_en &&
      !inst_r[IQ_LD].uop.page_fault_load) {
    io.prf_awake->wake.valid = true;
    io.prf_awake->wake.preg = inst_r[IQ_LD].uop.dest_preg;
  } else {
    io.prf_awake->wake.valid = false;
  }
}

void PRF::comb_branch() {
  if (io.dec_bcast->mispred) {
    for (int i = 0; i < ISSUE_WAY; i++) {
      if (inst_r[i].valid &&
          (io.dec_bcast->br_mask & (1 << inst_r[i].uop.tag))) {
        inst_r_1[i].valid = false;
      }
    }
  }
}

void PRF::comb_flush() {
  if (io.rob_bcast->flush) {
    for (int i = 0; i < ISSUE_WAY; i++) {
      inst_r_1[i].valid = false;
    }
  }
}

void PRF::comb_write() {
  for (int i = 0; i < ALU_NUM + 1; i++) {
    if (inst_r[i].valid && inst_r[i].uop.dest_en &&
        !is_page_fault(inst_r[i].uop)) {
      reg_file_1[inst_r[i].uop.dest_preg] = inst_r[i].uop.result;
    }
  }
}

void PRF::comb_pipeline() {
  for (int i = 0; i < ISSUE_WAY; i++) {

    if (i!=IQ_LD&&io.exe2prf->entry[i].valid && io.prf2exe->ready[i]) {
      inst_r_1[i] = io.exe2prf->entry[i];
    }
    else if(i==IQ_LD&&io.prf2exe->ready[i]&&io.cache2prf->valid){ 
      
      inst_r_1[i].valid = io.cache2prf->valid;
      inst_r_1[i].uop.dest_en = true;
      inst_r_1[i].uop.dest_preg = io.cache2prf->uop.dest_preg;  
      inst_r_1[i].uop.result = load_data;
      inst_r_1[i].uop.page_fault_load = io.cache2prf->uop.page_fault_load;
      inst_r_1[i].uop.tag = io.cache2prf->uop.tag; 
      inst_r_1[i].uop.rob_idx = io.cache2prf->uop.rob_idx;
      inst_r_1[i].uop.instruction = io.cache2prf->uop.instruction;
      uint32_t addr = io.cache2prf->uop.src1_rdata + io.cache2prf->uop.imm;
      if (addr == 0x1fd0e000) {
        inst_r_1[i].uop.difftest_skip = true;
      }
      if(DCACHE_LOG){
        printf("\n\nLoad Return: valid:%d pc:0x%08x inst:0x%08x addr:0x%08x data:0x%x page_fault:%d rob_idx:%d preg:%d\n",io.cache2prf->valid,io.cache2prf->uop.pc,io.cache2prf->uop.instruction,addr,load_data,io.cache2prf->uop.page_fault_load, io.cache2prf->uop.rob_idx, io.cache2prf->uop.dest_preg);
      }
    }
    else {
      inst_r_1[i].valid = false;
    }
  }
}

void PRF::seq() {
  for (int i = 0; i < PRF_NUM; i++) {
    reg_file[i] = reg_file_1[i];
  }

  for (int i = 0; i < ISSUE_WAY; i++) {
    inst_r[i] = inst_r_1[i];
  }
}
