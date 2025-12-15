#include "TOP.h"
#include <EXU.h>
#include <config.h>
#include <cstdint>
#include <cvt.h>
#include <util.h>
extern Back_Top back;
extern uint32_t *p_memory;

void alu(Inst_uop &inst);
void bru(Inst_uop &inst);
void ldu(Inst_uop &inst, Mem_IN *&io);
void stu_addr(Inst_uop &inst);
void stu_data(Inst_uop &inst);
void mul(Inst_uop &inst);
void div(Inst_uop &inst);

void FU::exec(Inst_uop &inst, Mem_IN *&io,bool mispred)
{

  if (cycle == 0)
  {
    if (inst.op == UOP_MUL)
    { // mul
      latency = 1;
    }
    else if (inst.op == UOP_DIV)
    { // div
      latency = 1;
    }
    else if (inst.op == UOP_LOAD)
    {
      latency = 1;
    }
    else
    {
      latency = 1;
    }
  }

  cycle++;
  if(mispred){
    complete = true;
    cycle = 0;
    return;
  }
  if (cycle == latency)
  {
    // if(DCACHE_LOG){
    //   printf("FU exec complete: op:%d pc:0x%08x inst:0x%08x dest_preg:%d rob-idx:%d\n",inst.op,inst.pc,inst.instruction,inst.dest_preg, inst.rob_idx);
    // }
    if(is_load_uop(inst.op))
    {
      ldu(inst, io);
    }
    else if (is_sta_uop(inst.op))
    {
      stu_addr(inst);
    }
    else if (is_std_uop(inst.op))
    {
      stu_data(inst);
    }
    else if (is_branch_uop(inst.op))
    {
      bru(inst);
    }
    else if (inst.op == UOP_MUL)
    {
      mul(inst);
    }
    else if (inst.op == UOP_DIV)
    {
      div(inst);
    }
    else if (inst.op == UOP_SFENCE_VMA)
    {
      uint32_t vaddr = 0;
      uint32_t asid = 0;
      // TODO: sfence.vma

      // if(DCACHE_LOG){
      //   printf("SFENCE_VMA called pc:0x%08x inst:0x%08x\n", inst.pc, inst.instruction);
      // }
    }
    else
      alu(inst);

    if(!is_load_uop(inst.op)){
      complete = true;
      cycle = 0;
    }
  }
}
void EXU::init()
{
  for (int i = 0; i < ISSUE_WAY; i++)
  {
    inst_r[i].valid = false;
  }
}

void EXU::comb_ready()
{
  for (int i = 0; i < ISSUE_WAY; i++)
  {
    io.exe2iss->ready[i] =
        (!inst_r[i].valid || fu[i].complete) && !io.dec_bcast->mispred;
  }
}

void EXU::comb_latency()
{
  if (inst_r[IQ_LD].valid)
  {
    if(io.rob_bcast->flush){
      fu[IQ_LD].complete = true;
      fu[IQ_LD].cycle = 0;
      io.exe2prf->entry[IQ_LD].valid = false;
      return;
    }
    if(io.dec_bcast->mispred&&((1<<inst_r[IQ_LD].uop.tag)&io.dec_bcast->br_mask)){
      fu[IQ_LD].complete = true;
      fu[IQ_LD].cycle = 0;
      io.exe2prf->entry[IQ_LD].valid = false;
      return;
    }
    if(io.ldq2cache->ready){
      fu[IQ_LD].complete = true;
      fu[IQ_LD].cycle = 0;
      io.exe2prf->entry[IQ_LD].valid = true;
    }else{
      fu[IQ_LD].complete = false;
      fu[IQ_LD].latency++;
      io.exe2prf->entry[IQ_LD].valid = false;
    }
  }
  
}
void EXU::comb_exec()
{
  io.ldq2cache->req = false;
  for (int i = 0; i < ISSUE_WAY; i++)
  {
    io.exe2prf->entry[i].valid = false;
    io.exe2prf->entry[i].uop = inst_r[i].uop;
    if(inst_r[i].valid){
      fu[i].exec(io.exe2prf->entry[i].uop, io.ldq2cache, (io.dec_bcast->mispred&&((1<<inst_r[i].uop.tag)&io.dec_bcast->br_mask)) || io.rob_bcast->flush);
      if(i==IQ_LD)continue;
      if (fu[i].complete &&
          !(io.dec_bcast->mispred &&
            ((1 << inst_r[i].uop.tag) & io.dec_bcast->br_mask)) &&
          !io.rob_bcast->flush)
      {
        io.exe2prf->entry[i].valid = true;
      }
      else
      {
        io.exe2prf->entry[i].valid = false;
      }
    }
  }
  // store
  if (inst_r[IQ_STA].valid)
  {
    io.exe2stq->addr_entry = io.exe2prf->entry[IQ_STA];
  }
  else
  {
    io.exe2stq->addr_entry.valid = false;
  }

  if (inst_r[IQ_STD].valid)
  {
    io.exe2stq->data_entry = io.exe2prf->entry[IQ_STD];
  }
  else
  {
    io.exe2stq->data_entry.valid = false;
  }

  io.exe2cache->flush = io.rob_bcast->flush;
  io.exe2cache->mispred = io.dec_bcast->mispred;
  io.exe2cache->br_mask = io.dec_bcast->br_mask;
}
void EXU::comb_to_csr()
{
  io.exe2csr->we = false;
  io.exe2csr->re = false;

  if (inst_r[0].valid && inst_r[0].uop.op == UOP_CSR && !io.rob_bcast->flush)
  {
    io.exe2csr->we = inst_r[0].uop.func3 == 1 || inst_r[0].uop.src1_areg != 0;

    io.exe2csr->re = inst_r[0].uop.func3 != 1 || inst_r[0].uop.dest_areg != 0;

    io.exe2csr->idx = inst_r[0].uop.csr_idx;
    io.exe2csr->wcmd = inst_r[0].uop.func3 & 0b11;
    if (inst_r[0].uop.src2_is_imm)
    {
      io.exe2csr->wdata = inst_r[0].uop.imm;
    }
    else
    {
      io.exe2csr->wdata = inst_r[0].uop.src1_rdata;
    }
  }
}

void EXU::comb_from_csr()
{
  if (inst_r[0].valid && inst_r[0].uop.op == UOP_CSR && io.exe2csr->re)
  {
    io.exe2prf->entry[0].uop.result = io.csr2exe->rdata;
  }
}

void EXU::comb_pipeline()
{
  for (int i = 0; i < ISSUE_WAY; i++)
  {
    if (io.prf2exe->iss_entry[i].valid && io.exe2iss->ready[i])
    {
      inst_r_1[i] = io.prf2exe->iss_entry[i];
      fu[i].complete = false;
      fu[i].cycle = 0;
    }
    else if (io.exe2prf->entry[i].valid && io.prf2exe->ready[i])
    {
      inst_r_1[i].valid = false;
      fu[i].complete = false;
      fu[i].cycle = 0;
    }
  }
}

void EXU::comb_branch()
{
  if (io.dec_bcast->mispred)
  {
    for (int i = 0; i < ISSUE_WAY; i++)
    {
      if (inst_r[i].valid &&
          (io.dec_bcast->br_mask & (1 << inst_r[i].uop.tag)))
      {
        inst_r_1[i].valid = false;
        fu[i].complete = false;
        fu[i].cycle = 0;
      }
    }
  }
}

void EXU::comb_flush()
{
  if (io.rob_bcast->flush)
  {
    for (int i = 0; i < ISSUE_WAY; i++)
    {
      inst_r_1[i].valid = false;
      fu[i].complete = false;
      fu[i].cycle = 0;
    }
  }
}

void EXU::seq()
{
  for (int i = 0; i < ISSUE_WAY; i++)
  {
    inst_r[i] = inst_r_1[i];
  }
}
