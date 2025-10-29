#include "Arbiter.h"
#include <cstdio>

void Arbiter::init(){
    state=0;
}

void Arbiter::comb_in()
{
    if(state==0){
        if(io.cpu_st->req==true){
            io.mem->control.en = io.cpu_st->req;
            io.mem->control.addr = io.cpu_st->addr;
            io.mem->control.wdata = io.cpu_st->wdata;
            io.mem->control.wen = io.cpu_st->wr;
            io.mem->control.sel = io.cpu_st->wstrb;
        }else if(io.cpu_ld->req==true){
            //只有load
            io.mem->control.en = io.cpu_ld->req;
            io.mem->control.addr = io.cpu_ld->addr;
            io.mem->control.wdata = io.cpu_ld->wdata;
            io.mem->control.wen = io.cpu_ld->wr;
            io.mem->control.sel = io.cpu_ld->wstrb;
        }
        else{
            io.mem->control.en = false;
            io.mem->control.addr = 0;
            io.mem->control.wdata = 0;
            io.mem->control.wen = 0;
            io.mem->control.sel = 0;
        }
    }
    else if(state==1){
        //优先处理store
        io.mem->control.en = io.cpu_st->req;
        io.mem->control.addr = io.cpu_st->addr;
        io.mem->control.wdata = io.cpu_st->wdata;
        io.mem->control.wen = io.cpu_st->wr;
        io.mem->control.sel = io.cpu_st->wstrb;
    }else if(state==2){
        //优先处理load
        io.mem->control.en = io.cpu_ld->req;
        io.mem->control.addr = io.cpu_ld->addr;
        io.mem->control.wdata = io.cpu_ld->wdata;
        io.mem->control.wen = io.cpu_ld->wr;
        io.mem->control.sel = io.cpu_ld->wstrb;
    }
    io.mem->control.len=0;
    io.mem->control.size=0;
    io.mem->control.last=1;

}
void Arbiter::comb_out()
{
    if(state==1&&io.mem->data.last==true&&io.cpu_st->req==true){
        io.cpu_st->rdata = io.mem->data.data;
        io.cpu_st->data_ok = io.mem->data.last;
        io.cpu_ld->data_ok = false;
        io.cpu_ld->rdata = 0;
    }else if(state==2&&io.mem->data.last==true&&io.cpu_ld->req==true){ 
        io.cpu_ld->rdata = io.mem->data.data;
        io.cpu_ld->data_ok = io.mem->data.last;
        io.cpu_st->data_ok = false;
        io.cpu_st->rdata = 0;
    }else if(state==0){
        io.cpu_ld->data_ok = false;
        io.cpu_ld->rdata = 0;
        io.cpu_st->data_ok = false;
        io.cpu_st->rdata = 0;
    }
    if (ARB_LOG)
    {
        printf("arbiter state:%d\n",state);
        printf("arbiter mem.en:%d mem.wen:%d mem.addr:%08x mem.wdata:%08x mem.sel:%d mem.len:%d mem.size%d mem.last:%d\n",io.mem->control.en,io.mem->control.wen,io.mem->control.addr,io.mem->control.wdata,io.mem->control.sel,io.mem->control.len,io.mem->control.size,io.mem->control.last);
        printf("arbiter cpu_st.req:%d cpu_st.wr:%d cpu_st.addr:%08x cpu_st.wdata:%08x cpu_st.wstrb:%02d cpu_st.rdata:%08x cpu_st.data_ok:%d\n",io.cpu_st->req,io.cpu_st->wr,io.cpu_st->addr,io.cpu_st->wdata,io.cpu_st->wstrb,io.cpu_st->rdata,io.cpu_st->data_ok);
        printf("arbiter cpu_ld.req:%d cpu_ld.wr:%d cpu_ld.addr:%08x cpu_ld.wdata:%08x cpu_ld.wstrb:%02d cpu_ld.rdata:%08x cpu_ld.data_ok:%d\n",io.cpu_ld->req,io.cpu_ld->wr,io.cpu_ld->addr,io.cpu_ld->wdata,io.cpu_ld->wstrb,io.cpu_ld->rdata,io.cpu_ld->data_ok);
        printf("arbiter mem.data.data:%08x mem.data.last:%d\n\n",io.mem->data.data,io.mem->data.last);


    }
    
}
void Arbiter::seq()
{
    if(state==0){
        if(io.cpu_st->req==true){
            state=1;
        }
        else if(io.cpu_ld->req==true){
            state=2;
        }
    }else if(state==1){
        if(io.mem->data.last==true||io.cpu_st->req==false ){
            state=0;
        }
    }else if(state==2){
        if(io.mem->data.last==true||io.cpu_ld->req==false){
            state=0;
        }
    }
}