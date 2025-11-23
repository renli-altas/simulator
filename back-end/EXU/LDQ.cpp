#include "TOP.h"
#include <Cache.h>
#include <LDQ.h>
#include <config.h>
#include <cstdint>
#include <iostream>
#include <util.h>

extern Back_Top back;
extern Cache cache;
extern uint32_t commit_num;

void LDQ::comb()
{
  int num = count;

  for (int i = 0; i < 2; i++)
  {
    if (!in.dis2ldq->valid[i])
    {
      out.ldq2dis->ready[i] = true;
    }
    else
    {
      if (num < LDQ_NUM)
      {
        out.ldq2dis->ready[i] = true;
        num++;
      }
      else
      {
        out.ldq2dis->ready[i] = false;
      }
    }
  }

  if (entry[deq_ptr].valid && entry[deq_ptr].data_valid && !entry[deq_ptr].complete)
  {
    uint32_t addr = entry[deq_ptr].addr;
    uint32_t data = in.cache2ldq->rdata;
    bool stall_load = entry[deq_ptr].stall_load;
    uint32_t mask = 0;
    uint32_t sign = 0;
    if (addr == 0x1fd0e000)
    {
      out.ldq2prf->rdata = commit_num;
      out.ldq2prf->preg = entry[deq_ptr].preg;
      out.ldq2prf->data_valid = true;
      out.ldq2cache->req = false;
      out.ldq2cache->wr = 0;
      out.ldq2cache->wdata = 0;
      out.ldq2cache->wstrb = 0;
      out.ldq2cache->addr = 0;
      entry[deq_ptr].complete = true;
      entry[deq_ptr].stall_load = false;
    }
    else if (addr == 0x1fd0e004)
    {
      out.ldq2prf->rdata = 0;
      out.ldq2prf->preg = entry[deq_ptr].preg;
      out.ldq2prf->data_valid = true;
      out.ldq2cache->req = false;
      out.ldq2cache->wr = 0;
      out.ldq2cache->wdata = 0;
      out.ldq2cache->wstrb = 0;
      out.ldq2cache->addr = 0;
      entry[deq_ptr].complete = true;
      entry[deq_ptr].stall_load = false;
    }
    else
    {
      if (in.cache2ldq->data_valid||stall_load)
      {
        out.ldq2cache->req = false;
        out.ldq2cache->wr = 0;
        out.ldq2cache->wdata = 0;
        out.ldq2cache->wstrb = 0;
        out.ldq2cache->addr = 0;
        back.stq.st2ld_fwd(addr, data, entry[deq_ptr].rob_idx, stall_load);
        if(stall_load)
        {
          entry[deq_ptr].stall_load = true;
          return;
        }
        entry[deq_ptr].stall_load = false;
        data = data >> ((entry[deq_ptr].addr & 0x3) * 8);
        uint32_t size = entry[deq_ptr].size & 0b11;
        if (size == 0)
        {
          mask = 0xFF;
          if (data & 0x80)
            sign = 0xFFFFFF00;
        }
        else if (size == 0b01)
        {
          mask = 0xFFFF;
          if (data & 0x8000)
            sign = 0xFFFF0000;
        }
        else
        {
          mask = 0xFFFFFFFF;
        }

        data = data & mask;
        if (!(entry[deq_ptr].size & 0b100))
        {
          data = data | sign;
        }
        entry[deq_ptr].complete = true;
        out.ldq2prf->rdata = data;
        out.ldq2prf->preg = entry[deq_ptr].preg;
        out.ldq2prf->data_valid = true;
      }
      else
      {
        out.ldq2cache->req = true;
        out.ldq2cache->wr = 0;
        out.ldq2cache->wdata = 0;
        out.ldq2cache->wstrb = 0;
        out.ldq2cache->addr = addr;
        out.ldq2prf->data_valid = false;
        out.ldq2prf->preg = entry[deq_ptr].preg;
        out.ldq2prf->rdata = 0;
      }
    }
  }
  else
  {
    out.ldq2cache->req = false;
    out.ldq2cache->wr = 0;
    out.ldq2cache->wdata = 0;
    out.ldq2cache->wstrb = 0;
    out.ldq2cache->addr = 0;
    out.ldq2prf->data_valid = false;
  }
}

void LDQ::seq()
{

  // 入队
  for (int i = 0; i < 2; i++)
  {
    if (in.dis2ldq->dis_fire[i] && in.dis2ldq->valid[i])
    {
      entry[enq_ptr].tag = in.dis2ldq->tag[i];
      entry[enq_ptr].valid = true;
      entry[enq_ptr].data_valid = false;
      entry[enq_ptr].complete = false;
      entry[enq_ptr].stall_load = false;
      count++;
      LOOP_INC(enq_ptr, LDQ_NUM);
    }
  }

  // 地址数据写入 若项无效说明被br清除
  Inst_uop *inst = &in.exe2ldq->entry.uop;
  int idx = inst->ldq_idx;
  if (in.exe2ldq->entry.valid && entry[idx].valid)
  {
    entry[idx].addr = inst->result;
    entry[idx].size = inst->func3;
    entry[idx].data_valid = true;
    entry[idx].rob_idx = inst->rob_idx;
  }

  // 分支清空
  if (in.dec_bcast->mispred)
  {
    for (int i = 0; i < LDQ_NUM; i++)
    {
      if (entry[i].valid && !entry[i].complete &&
          (in.dec_bcast->br_mask & (1 << entry[i].tag)))
      {
        entry[i].valid = false;
        entry[i].complete = false;
        entry[i].stall_load = false;
        count--;
        LOOP_DEC(enq_ptr, LDQ_NUM);
      }
    }
  }

  if (in.rob_bcast->flush)
  {
    for (int i = 0; i < LDQ_NUM; i++)
    {
      if (entry[i].valid && !entry[i].complete)
      {
        entry[i].valid = false;
        entry[i].complete = false;
        entry[i].stall_load = false;
        count--;
        LOOP_DEC(enq_ptr, LDQ_NUM);
      }
    }
  }

  out.ldq2dis->ldq_idx = enq_ptr;
}
