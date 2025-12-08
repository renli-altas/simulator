#include "TOP.h"
#include <STQ.h>
#include <config.h>
#include <cstdint>
#include <iostream>
#include <util.h>

extern Back_Top back;
void STQ::comb_out()
{
  int num = count;

  for (int i = 0; i < 2; i++)
  {
    if (!io.dis2stq->valid[i])
    {
      io.stq2dis->ready[i] = true;
    }
    else
    {
      if (num < STQ_NUM)
      {
        io.stq2dis->ready[i] = true;
        num++;
      }
      else
      {
        io.stq2dis->ready[i] = false;
      }
    }
  }

  // 写端口 同时给ld_IQ发送唤醒信息
  if (entry[deq_ptr].valid && entry[deq_ptr].complete)
  {
    extern uint32_t *p_memory;
    uint32_t wdata = entry[deq_ptr].data;
    uint32_t waddr = entry[deq_ptr].addr;
    uint32_t wstrb;
    if (entry[deq_ptr].size == 0b00)
      wstrb = 0b1;
    else if (entry[deq_ptr].size == 0b01)
      wstrb = 0b11;
    else
      wstrb = 0b1111;

    int offset = entry[deq_ptr].addr & 0x3;
    wstrb = wstrb << offset;
    wdata = wdata << (offset * 8);
    write_flag = 0;
    if (waddr == UART_BASE)
    {
      char temp;
      temp = wdata & 0x000000ff;
      p_memory[0x10000000 / 4] = p_memory[0x10000000 / 4] & 0xffffff00;
      cout << temp;
    }

    else if (waddr == 0x10000001 && (wdata & 0x000000ff) == 7)
    {
      p_memory[0xc201004 / 4] = 0xa;
      p_memory[0x10000000 / 4] = p_memory[0x10000000 / 4] & 0xfff0ffff;
    }
    else if (waddr == 0x10000001 && (wdata & 0x000000ff) == 5)
    {
      p_memory[0x10000000 / 4] =
          p_memory[0x10000000 / 4] & 0xfff0ffff | 0x00030000;
    }
    else if (waddr == 0xc201004 && (wdata & 0x000000ff) == 0xa)
    {
      p_memory[0xc201004 / 4] = 0x0;
    }
    else
    {
      io.stq2cache->req = true;
      io.stq2cache->wr = true;
      io.stq2cache->wstrb = wstrb;
      io.stq2cache->wdata = wdata;
      io.stq2cache->addr = waddr;
      io.stq2cache->uop.tag = entry[deq_ptr].tag;
      io.stq2cache->uop.dest_preg = deq_ptr;
      io.stq2cache->uop.rob_idx = (back.rob.deq_ptr << 2);
      write_flag = 1;
    }
    if (write_flag == 0)
    {
      entry[deq_ptr].issued = 2;
    }
    else {
      entry[deq_ptr].issued = 1;
    }
  }
  else
  {
    io.stq2cache->req = false;
    write_flag = 2;
  }

  

  // commit标记为可执行
  for (int i = 0; i < COMMIT_WIDTH; i++)
  {
    if (io.rob_commit->commit_entry[i].valid &&
        (is_store(io.rob_commit->commit_entry[i].uop)) &&
        !io.rob_commit->commit_entry[i].uop.page_fault_store)
    {
      entry[commit_ptr].complete = true;
      commit_count++;
      LOOP_INC(commit_ptr, STQ_NUM);
      if((entry[commit_ptr].addr &0xfffffffc) == 0x808bbf30  && entry[commit_ptr].addr_valid){
        printf("Debug STQ commit store to addr simtime:%lld 0x%08x data 0x%08x inst 0x%08x\n",sim_time, entry[commit_ptr].addr, entry[commit_ptr].data, entry[commit_ptr].inst);
      }
    }
  }
  if(DCACHE_LOG){
    printf("STQ State write_flag:%d flush:%d mispread:%d\n",write_flag, io.rob_bcast->flush, io.dec_bcast->mispred);
  stq_print();
  }
}
void STQ::comb_in()
{
  if (io.cache2stq->valid)
  {
    entry[io.cache2stq->uop.dest_preg].issued = 2;
    if(DCACHE_LOG){
      printf("STQ receive write done from cache for STQ entry %d address %08x inst %08x\n", io.cache2stq->uop.dest_preg, entry[io.cache2stq->uop.dest_preg].addr, entry[io.cache2stq->uop.dest_preg].inst);
    }
  }
  
  if ((io.stq2cache->ready&&io.stq2cache->req) || write_flag == 0)
  {
    LOOP_INC(deq_ptr, STQ_NUM);
  }
  if (entry[fwd_ptr].issued == 2 && entry[fwd_ptr].valid == true && entry[fwd_ptr].complete == true)
  {
    printf("STQ commit fwd\n");
    entry[fwd_ptr].valid = false;  
    entry[fwd_ptr].complete = false; 
    entry[fwd_ptr].issued = 0; 
    LOOP_INC(fwd_ptr, STQ_NUM);
    commit_count--;
    count--;
  }
}
void STQ::seq()
{

  // 入队
  for (int i = 0; i < 2; i++)
  {
    if (io.dis2stq->dis_fire[i] && io.dis2stq->valid[i])
    {
      entry[enq_ptr].tag = io.dis2stq->tag[i];
      entry[enq_ptr].valid = true;
      entry[enq_ptr].issued = 0;
      entry[enq_ptr].addr_valid = false;
      entry[enq_ptr].data_valid = false;
      count++;
      LOOP_INC(enq_ptr, STQ_NUM);
    }
  }

  // 地址数据写入 若项无效说明被br清除
  Inst_uop *inst = &io.exe2stq->addr_entry.uop;
  int idx = inst->stq_idx;
  if (io.exe2stq->addr_entry.valid && entry[idx].valid)
  {
    entry[idx].addr = inst->result;
    entry[idx].size = inst->func3;
    entry[idx].addr_valid = true;
    entry[idx].inst = inst->instruction;
  }

  inst = &io.exe2stq->data_entry.uop;
  idx = inst->stq_idx;

  if (io.exe2stq->data_entry.valid && entry[idx].valid)
  {
    entry[idx].data = inst->result;
    entry[idx].data_valid = true;
  }

  // 分支清空
  if (io.dec_bcast->mispred)
  {
    for (int i = 0; i < STQ_NUM; i++)
    {
      if (entry[i].valid&& !entry[i].complete &&
          (io.dec_bcast->br_mask & (1 << entry[i].tag)))
      {
        entry[i].valid = false;
        entry[i].issued = 0;
        entry[i].complete = false;
        count--;
        LOOP_DEC(enq_ptr, STQ_NUM);
      }
    }
  }

  if (io.rob_bcast->flush)
  {
    for (int i = 0; i < STQ_NUM; i++)
    {
      if (entry[i].valid&& !entry[i].complete)
      {
        entry[i].valid = false;
        entry[i].issued = 0;
        entry[i].complete = false;
        count--;
        LOOP_DEC(enq_ptr, STQ_NUM);
      }
    }
  }

  io.stq2dis->stq_idx = enq_ptr;
}

extern uint32_t *p_memory;
void STQ::st2ld_fwd(uint32_t addr, uint32_t &data, int rob_idx)
{ //, bool &stall_load) {


  int i = fwd_ptr;
  int fwd_count = commit_count;
  while (fwd_count != 0)
  {
    if ((entry[i].addr & 0xFFFFFFFC) == (addr & 0xFFFFFFFC))
    {

      uint32_t wdata = entry[i].data;
      uint32_t waddr = entry[i].addr;
      uint32_t wstrb;
      if (entry[i].size == 0b00)
        wstrb = 0b1;
      else if (entry[i].size == 0b01)
        wstrb = 0b11;
      else
        wstrb = 0b1111;

      int offset = entry[i].addr & 0x3;
      wstrb = wstrb << offset;
      wdata = wdata << (offset * 8);

      uint32_t mask = 0;
      if (wstrb & 0b1)
        mask |= 0xFF;
      if (wstrb & 0b10)
        mask |= 0xFF00;
      if (wstrb & 0b100)
        mask |= 0xFF0000;
      if (wstrb & 0b1000)
        mask |= 0xFF000000;
      
      data = (mask & wdata) | (~mask & data);
      if (DCACHE_LOG)
        printf("STQ Load Forwarding from STQ entry %d address %08x inst %08x wdata %08x count:%d mask:%08x\n", i, entry[i].addr, entry[i].inst, data, count, mask);
      
    }
    LOOP_INC(i, STQ_NUM);
    fwd_count--;
  }

  int idx = back.rob.deq_ptr << 2;

  while (idx != rob_idx)
  {
    int line_idx = idx >> 2;
    int bank_idx = idx & 0b11;
    if (back.rob.entry[bank_idx][line_idx].valid &&
        is_store(back.rob.entry[bank_idx][line_idx].uop))
    {
      int stq_idx = back.rob.entry[bank_idx][line_idx].uop.stq_idx;
      // if (entry[stq_idx].valid &&
      //     (!entry[stq_idx].data_valid || !entry[stq_idx].addr_valid)) {
      //   // 有未准备好的store，停止转发
      //   stall_load = true;
      //   return;
      // }
      if ((entry[stq_idx].addr & 0xFFFFFFFC) == (addr & 0xFFFFFFFC) && entry[stq_idx].valid)
      {
        if (DCACHE_LOG)
          printf("STQ Load Forwarding from STQ entry %d for ROB entry %d address %08x wdata %08x\n", stq_idx, idx, entry[stq_idx].addr, entry[stq_idx].data);
        
        uint32_t wdata = entry[stq_idx].data;
        uint32_t waddr = entry[stq_idx].addr;
        uint32_t wstrb;
        if (entry[stq_idx].size == 0b00)
          wstrb = 0b1;
        else if (entry[stq_idx].size == 0b01)
          wstrb = 0b11;
        else
          wstrb = 0b1111;

        int offset = entry[stq_idx].addr & 0x3;
        wstrb = wstrb << offset;
        wdata = wdata << (offset * 8);

        uint32_t mask = 0;
        if (wstrb & 0b1)
          mask |= 0xFF;
        if (wstrb & 0b10)
          mask |= 0xFF00;
        if (wstrb & 0b100)
          mask |= 0xFF0000;
        if (wstrb & 0b1000)
          mask |= 0xFF000000;

        data = (mask & wdata) | (~mask & data);
      }
    }
    LOOP_INC(idx, ROB_NUM);
  }
}
void STQ::stq_print()
{
  printf("-----STQ Print-----\n");
  printf("stq enq_ptr:%d deq_ptr:%d commit_ptr:%d fwd_ptr:%d count:%d commit_count:%d\n", enq_ptr, deq_ptr, commit_ptr, fwd_ptr, count, commit_count);
  for (int i = 0; i != STQ_NUM; i = i + 1)
  {
    printf("STQ entry %d valid:%d issued:%d addr:%08x data:%08x size:%d tag:%d addr_valid:%d data_valid:%d complete:%d\n", i, entry[i].valid, entry[i].issued, entry[i].addr, entry[i].data, entry[i].size, entry[i].tag, entry[i].addr_valid, entry[i].data_valid, entry[i].complete);
  }
  for (int i = 0; i < ROB_NUM; i++)
  {
    int line_idx = i >> 2;
    int bank_idx = i & 0b11;
    if (back.rob.entry[bank_idx][line_idx].valid && is_store(back.rob.entry[bank_idx][line_idx].uop))
    {
      int stq_idx = back.rob.entry[bank_idx][line_idx].uop.stq_idx;
      printf("ROB entry %d maps to STQ entry %d data:%08x addr:%08x\n", i, stq_idx, entry[stq_idx].data, entry[stq_idx].addr);
    }
  }
}