#include "Memory.h"
#include <cstdio>
#include <cstring>

uint32_t * p_memory= new uint32_t[PHYSICAL_MEMORY_LENGTH];
void MEMORY::init()
{
    state=MEM_IDLE;
    Latency_cnt=0;
    data_cnt=0;
    io.mem->data.done = false;
    io.mem->data.last = false;
    // memset(p_memory,0,4*PHYSICAL_MEMORY_LENGTH);
}

void MEMORY::comb()
{
   if(state == TRANSFER){
        //开始计数
        io.mem->data.done = true;
        io.mem->data.last = lastreg;
        io.mem->data.data = rdata;
        // printf("memory data.data:%08x data.last:%d\n",io.mem->data.data,io.mem->data.last);
   }
   else{
        io.mem->data.done = false;
        io.mem->data.last = false;
        io.mem->data.data = 0;
   }
}
void MEMORY::seq()
{
    
    if(io.mem->control.en == true && Latency_cnt == 0 && state == MEM_IDLE){
        state = LATENCY;
    }else if(io.mem->control.en == true && Latency_cnt == Latency && state == LATENCY){
        state = TRANSFER;
    }else if(io.mem->control.en == true && lastreg && state == TRANSFER){ //AXI优化
        state = MEM_IDLE;
    }
    else if(io.mem->control.en == false){
        state = MEM_IDLE;
    }

    // printf("memory state:%d\n",state);
    // printf("memory mem.en:%d mem.wen:%d mem.addr:%08x mem.wdata:%08x mem.sel:%d mem.len:%d mem.size%d mem.last:%d\n",io.mem->control.en,io.mem->control.wen,io.mem->control.addr,io.mem->control.wdata,io.mem->control.sel,io.mem->control.len,io.mem->control.size,io.mem->control.last);
    if(state == TRANSFER){
        if(io.mem->control.wen==0){
            rdata = p_memory[(io.mem->control.addr >> 2)];
            if(MEM_LOG){
                printf("\n\nload  data %08x in %08x(%08x) mask %d\n",io.mem->data.data,io.mem->control.addr,io.mem->control.addr>>2,io.mem->control.sel);
            }
        }
        else{
            uint32_t old_data = p_memory[io.mem->control.addr >> 2];
            uint32_t mask = 0;
            if (io.mem->control.sel & 0b1)
            mask |= 0xFF;
            if (io.mem->control.sel & 0b10)
            mask |= 0xFF00;
            if (io.mem->control.sel & 0b100)
            mask |= 0xFF0000;
            if (io.mem->control.sel & 0b1000)
            mask |= 0xFF000000;

            p_memory[io.mem->control.addr >> 2] = (mask & io.mem->control.wdata) | (~mask & old_data);
        
            if(MEM_LOG){
                printf("store data %08x in %08x(%08x) mask %d\n",p_memory[io.mem->control.addr>>2],io.mem->control.addr,io.mem->control.addr>>2,io.mem->control.sel);
            }
        }
    }

    if(state ==TRANSFER){
        lastreg = data_cnt == (io.mem->control.len+1);
    }else{
        lastreg = false;
    }

    if(state == LATENCY){
        Latency_cnt++;
    }
    else{
        Latency_cnt =0 ;
    }
    if(state == TRANSFER){
        data_cnt ++;
    }else{
        data_cnt =0;
    }
}