#include "TOP.h"
#include <STQ.h>
#include <config.h>
#include <cstdint>
#include <iostream>
#include <util.h>

extern Back_Top back;
void STQ::init(){
  free_queue_head = 0;
  free_queue_tail = 0;
  work_queue_head = 0;
  work_queue_tail = 0;
  work_queue_used = 0;
  count = 0;
  commit_count = 0;
  for(int i=0;i<STQ_NUM;i++){
    entry[i].valid = false;
    free_queue[i]=i;
    work_queue[i]=0;
  }
}
void STQ::comb() {
  int num = count;

  for (int i = 0; i < 2; i++) {
    if (!io.dis2stq->valid[i]) {
      io.stq2dis->ready[i] = true;
    } else {
      if (num < STQ_NUM) {
        io.stq2dis->ready[i] = true;
        num++;
      } else {
        io.stq2dis->ready[i] = false;
      }
    }
  }

  // 写端口 同时给ld_IQ发送唤醒信息
  if (entry[write_ptr].valid && entry[write_ptr].complete) {
    extern uint32_t *p_memory;
    uint32_t wdata = entry[write_ptr].data;
    uint32_t waddr = entry[write_ptr].addr;
    uint32_t wstrb;
    if (entry[write_ptr].size == 0b00)
      wstrb = 0b1;
    else if (entry[write_ptr].size == 0b01)
      wstrb = 0b11;
    else
      wstrb = 0b1111;

    int offset = entry[write_ptr].addr & 0x3;
    wstrb = wstrb << offset;
    wdata = wdata << (offset * 8);

    write_flag=0;
    if (waddr == UART_BASE) {
      char temp;
      temp = wdata & 0x000000ff;
      p_memory[0x10000000 / 4] = p_memory[0x10000000 / 4] & 0xffffff00;
      cout << temp;
    }

    else if (waddr == 0x10000001 && (wdata & 0x000000ff) == 7) {
      p_memory[0xc201004 / 4] = 0xa;
      p_memory[0x10000000 / 4] = p_memory[0x10000000 / 4] & 0xfff0ffff;
    }
    else if (waddr == 0x10000001 && (wdata & 0x000000ff) == 5) {
      p_memory[0x10000000 / 4] =
      p_memory[0x10000000 / 4] & 0xfff0ffff | 0x00030000;
    }
    else if (waddr == 0xc201004 && (wdata & 0x000000ff) == 0xa) {
      p_memory[0xc201004 / 4] = 0x0;
    }else if(io.stq2cache->ready==true){
      io.stq2cache->req = true;
      io.stq2cache->wr = true;
      io.stq2cache->wstrb = wstrb;
      io.stq2cache->wdata = wdata;
      io.stq2cache->addr = waddr;
      io.stq2cache->tag = entry[write_ptr].tag;
      io.stq2cache->preg = write_ptr;
      io.stq2cache->rob_idx = entry[write_ptr].index;
      write_flag = 1;
    }
    else{
      io.stq2cache->req = false;
      io.stq2cache->wr = false;
      io.stq2cache->wstrb = 0;
      io.stq2cache->wdata = 0;
      io.stq2cache->addr = 0;
      io.stq2cache->tag = 0;
      io.stq2cache->preg = 0;
      io.stq2cache->rob_idx = 0;
      write_flag = 2;
    }

    if(write_flag!=2)
    {
      entry[write_ptr].valid = write_flag==1 ? true : false;
      entry[write_ptr].complete = false;
      LOOP_INC(write_ptr, STQ_NUM);
      count--;
      commit_count--;
      io.stq2cache->req = false;
      io.stq2cache->wr = false;
    }
  }

  // commit标记为可执行
  for (int i = 0; i < COMMIT_WIDTH; i++) {
    if (io.rob_commit->commit_entry[i].valid &&
        (is_store(io.rob_commit->commit_entry[i].uop)) &&
        !io.rob_commit->commit_entry[i].uop.page_fault_store) {
      entry[commit_ptr].complete = true;
      commit_count++;
      LOOP_INC(commit_ptr, STQ_NUM);
    }
  }
}

void STQ::seq() {

  // 入队
  for (int i = 0; i < 2; i++) {
    if (io.dis2stq->dis_fire[i] && io.dis2stq->valid[i]) {
      entry[free_queue[free_queue_head]].tag = io.dis2stq->tag[i];
      entry[free_queue[free_queue_head]].valid = true;
      entry[free_queue[free_queue_head]].addr_valid = false;
      entry[free_queue[free_queue_head]].data_valid = false;
      count++;
      LOOP_INC(free_queue_head, STQ_NUM);
    }
  }

  // 地址数据写入 若项无效说明被br清除
  Inst_uop *inst = &io.exe2stq->addr_entry.uop;
  int idx = inst->stq_idx;
  if (io.exe2stq->addr_entry.valid && entry[idx].valid) {
    entry[idx].addr = inst->result;
    entry[idx].size = inst->func3;
    entry[idx].addr_valid = true;
  }

  inst = &io.exe2stq->data_entry.uop;
  idx = inst->stq_idx;

  if (io.exe2stq->data_entry.valid && entry[idx].valid) {
    entry[idx].data = inst->result;
    entry[idx].data_valid = true;
  }

  // 分支清空
  if (io.dec_bcast->mispred) {//可能有问题
    uint32_t count_for = count;
    for (int i = 0; i < count_for; i++) {
      uint32_t work_queue_index=work_queue[(i+work_queue_head)%STQ_NUM];
      if (entry[work_queue_index].valid && !entry[work_queue_index].complete &&
          (io.dec_bcast->br_mask & (1 << entry[work_queue_index].tag))) {
        entry[work_queue_index].valid = false;
        count--;
        free_queue[free_queue_tail] = work_queue_index;
        work_queue[(i+work_queue_head)%STQ_NUM]=0;
        LOOP_DEC(work_queue_tail,STQ_NUM);
        LOOP_INC(free_queue_tail, STQ_NUM);
      }
    }
  }

  if (io.rob_bcast->flush) {
    uint32_t count_for = count;
    for (int i = 0; i < count_for; i++) {
      uint32_t work_queue_index=work_queue[(i+work_queue_head)%STQ_NUM];
      if (entry[work_queue_index].valid && !entry[work_queue_index].complete) {
        entry[i].valid = false;
        count--;
        free_queue[free_queue_tail] = i;
        LOOP_INC(free_queue_tail, STQ_NUM);
      }
    }
  }

  io.stq2dis->stq_idx1 = free_queue[free_queue_head];
  io.stq2dis->stq_idx2 = free_queue[(free_queue_head+1)%STQ_NUM];
}

extern uint32_t *p_memory;
void STQ::st2ld_fwd(uint32_t addr, uint32_t &data, int rob_idx){//, bool &stall_load) {

  int i = deq_ptr;
  int count = commit_count;
  while (count != 0) {
    if ((entry[i].addr & 0xFFFFFFFC) == (addr & 0xFFFFFFFC)) {
      
      if(DCACHE_LOG)
      printf("STQ Load Forwarding from STQ entry %d address %08x wdata %08x\n", i, entry[i].addr, entry[i].data);
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
    }
    LOOP_INC(i, STQ_NUM);
    count--;
  }

  int idx = back.rob.deq_ptr << 2;

  while (idx != rob_idx) {
    int line_idx = idx >> 2;
    int bank_idx = idx & 0b11;
    if (back.rob.entry[bank_idx][line_idx].valid &&
        is_store(back.rob.entry[bank_idx][line_idx].uop)) {
      int stq_idx = back.rob.entry[bank_idx][line_idx].uop.stq_idx;
      // if (entry[stq_idx].valid && 
      //     (!entry[stq_idx].data_valid || !entry[stq_idx].addr_valid)) {
      //   // 有未准备好的store，停止转发
      //   stall_load = true;
      //   return;
      // }
      if ((entry[stq_idx].addr & 0xFFFFFFFC) == (addr & 0xFFFFFFFC)) {
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
void STQ::stq_print(){
  printf("-----STQ Print-----\n");
  printf("stq enq_ptr:%d deq_ptr:%d commit_ptr:%d count:%d commit_count:%d\n",enq_ptr,deq_ptr,commit_ptr,count,commit_count);
  for(int i=0;i!=STQ_NUM;i=i+1){
      printf("STQ entry %d valid:%d addr:%08x data:%08x size:%d tag:%d addr_valid:%d data_valid:%d complete:%d\n",i, entry[i].valid, entry[i].addr, entry[i].data, entry[i].size, entry[i].tag, entry[i].addr_valid, entry[i].data_valid, entry[i].complete);
      
  }
  for(int i=0;i<ROB_NUM;i++){
    int line_idx = i>>2;
    int bank_idx = i&0b11;
    if(back.rob.entry[bank_idx][line_idx].valid&&is_store(back.rob.entry[bank_idx][line_idx].uop)){
      int stq_idx = back.rob.entry[bank_idx][line_idx].uop.stq_idx;
      printf("ROB entry %d maps to STQ entry %d data:%08x addr:%08x\n",i,stq_idx,entry[stq_idx].data,entry[stq_idx].addr);
    }
  }
}