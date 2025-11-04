#include "Arbiter.h"
#include <cstdio>

void Arbiter::init(){
    state=0;
}

void Arbiter::comb_in()
{
    if(state==0){
        if(io.cpu_st->control.en==true){
            io.mem->control.en = io.cpu_st->control.en;
            io.mem->control.addr = io.cpu_st->control.addr;
            io.mem->control.wdata = io.cpu_st->control.wdata;
            io.mem->control.wen = io.cpu_st->control.wen;
            io.mem->control.sel = io.cpu_st->control.sel;
            io.mem->control.len=io.cpu_st->control.len;
            io.mem->control.size=io.cpu_st->control.size;
            io.mem->control.last=io.cpu_st->control.last;
        }else if(io.cpu_ld->control.en==true){
            //只有load
            io.mem->control.en = io.cpu_ld->control.en;
            io.mem->control.addr = io.cpu_ld->control.addr;
            io.mem->control.wdata = io.cpu_ld->control.wdata;
            io.mem->control.wen = io.cpu_ld->control.wen;
            io.mem->control.sel = io.cpu_ld->control.sel;
            io.mem->control.len=io.cpu_ld->control.len;
            io.mem->control.size=io.cpu_ld->control.size;
            io.mem->control.last=io.cpu_ld->control.last;
        }
        else{
            io.mem->control.en = false;
            io.mem->control.addr = 0;
            io.mem->control.wdata = 0;
            io.mem->control.wen = 0;
            io.mem->control.sel = 0;
            io.mem->control.len=0;
            io.mem->control.size=0;
            io.mem->control.last=1;
        }
    }
    else if(state==1){
        //优先处理store
        io.mem->control.en = io.cpu_st->control.en;
        io.mem->control.addr = io.cpu_st->control.addr;
        io.mem->control.wdata = io.cpu_st->control.wdata;
        io.mem->control.wen = io.cpu_st->control.wen;
        io.mem->control.sel = io.cpu_st->control.sel;
        io.mem->control.len=io.cpu_st->control.len;
        io.mem->control.size=io.cpu_st->control.size;
        io.mem->control.last=io.cpu_st->control.last;
    }else if(state==2){
        //优先处理load
        io.mem->control.en = io.cpu_ld->control.en;
        io.mem->control.addr = io.cpu_ld->control.addr;
        io.mem->control.wdata = io.cpu_ld->control.wdata;
        io.mem->control.wen = io.cpu_ld->control.wen;
        io.mem->control.sel = io.cpu_ld->control.sel;
        io.mem->control.len=io.cpu_ld->control.len;
        io.mem->control.size=io.cpu_ld->control.size;
        io.mem->control.last=io.cpu_ld->control.last;
    }
}
void Arbiter::comb_out()
{
    if(state==1&&io.mem->data.last==true&&io.cpu_st->control.en==true){
        io.cpu_st->data.data = io.mem->data.data;
        io.cpu_st->data.last = io.mem->data.last;
        io.cpu_st->data.done = io.mem->data.done;

        io.cpu_ld->data.done = false;
        io.cpu_ld->data.last = false;
        io.cpu_ld->data.data = 0;
    }else if(state==2&&io.mem->data.last==true&&io.cpu_ld->control.en==true){ 
        io.cpu_ld->data.data = io.mem->data.data;
        io.cpu_ld->data.last = io.mem->data.last;
        io.cpu_ld->data.done = io.mem->data.done;
        io.cpu_st->data.done = false;
        io.cpu_st->data.last = false;
        io.cpu_st->data.data = 0;
    }else if(state==0){
        io.cpu_ld->data.done = false;
        io.cpu_ld->data.last = false;
        io.cpu_ld->data.data = 0;
        io.cpu_st->data.done = false;
        io.cpu_st->data.last = false;
        io.cpu_st->data.data = 0;
    }
    if (ARB_LOG)
    {
        printf("\narbiter state:%d\n",state);
        printf("cpu_st en:%d addr:%08x wdata:0x%08x wen:%d sel:%d len:%d size:%d last:%d\n",io.cpu_st->control.en,io.cpu_st->control.addr,io.cpu_st->control.wdata,io.cpu_st->control.wen,io.cpu_st->control.sel,io.cpu_st->control.len,io.cpu_st->control.size,io.cpu_st->control.last);
        printf("cpu_ld en:%d addr:%08x wdata:0x%08x wen:%d sel:%d len:%d size:%d last:%d\n",io.cpu_ld->control.en,io.cpu_ld->control.addr,io.cpu_ld->control.wdata,io.cpu_ld->control.wen,io.cpu_ld->control.sel,io.cpu_ld->control.len,io.cpu_ld->control.size,io.cpu_ld->control.last);
        printf("mem en:%d addr:0x%08x wdata:0x%08x wen:%d sel:%d len:%d size:%d last:%d\n",io.mem->control.en,io.mem->control.addr,io.mem->control.wdata,io.mem->control.wen,io.mem->control.sel,io.mem->control.len,io.mem->control.size,io.mem->control.last);
        printf("mem data:0x%08x last:%d done:%d\n",io.mem->data.data,io.mem->data.last,io.mem->data.done);
    }
    
}
void Arbiter::seq()
{
    if(state==0){
        if(io.cpu_st->control.en==true){
            state=1;
        }
        else if(io.cpu_ld->control.en==true){
            state=2;
        }
    }else if(state==1){
        if(io.mem->data.last==true||io.cpu_st->control.en==false ){
            state=0;
        }
    }else if(state==2){
        if(io.mem->data.last==true||io.cpu_ld->control.en==false){
            state=0;
        }
    }
}