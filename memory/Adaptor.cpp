// #include "Adaptor.h"
// #include <cstdio>
// void ADAPTOR::init(){
//     state=IDLE;
//     wflag=false;
//     rflag=false;

//     rdone=false;
//     wdone=false;

//     rlast=false;
//     wlast=false;

//     wvalid=false;

//     awready=false;
//     wready=false;

// }

// void ADAPTOR::comb(){
//     io.axi_r->m.rready_out = true;
//     io.axi_b->m.bready_out = true;
//     io.axi_aw->m.awid_out = 0;
//     io.axi_ar->m.arid_out = 0;
//     io.axi_aw->m.awburst_out = 1;
//     io.axi_ar->m.arburst_out = 1;

//     if(state==ARREADY_WAIT){
//         io.axi_ar->m.arvalid_out = true;
//         io.axi_ar->m.araddr_out =  io.cpu->control.addr;
//         io.axi_ar->m.arsize_out =  io.cpu->control.size;
//         io.axi_ar->m.arlen_out =  io.cpu->control.len;
//     }else{
//         io.axi_ar->m.arvalid_out = false;
//         io.axi_ar->m.araddr_out = 0;
//         io.axi_ar->m.arsize_out = 0;
//         io.axi_ar->m.arlen_out = 0;
//     }
    
//     io.cpu->data.data = state==IDLE&&io.axi_r->m.rvalid_in ? io.axi_r->m.rdata_in : 0;
    
//     if(state==WRITE_WAIT&!awready){
//         io.axi_aw->m.awvalid_out =  true ;
//         io.axi_aw->m.awaddr_out =  io.cpu->control.addr ;
//         io.axi_aw->m.awsize_out =  io.cpu->control.size ;
//         io.axi_aw->m.awlen_out = io.cpu->control.len ;
//     }else{
//         io.axi_aw->m.awvalid_out =  false ;
//         io.axi_aw->m.awaddr_out =  0 ;
//         io.axi_aw->m.awsize_out =  0 ;
//         io.axi_aw->m.awlen_out = 0 ;
//     }
    

//     io.axi_w->m.wvalid_out = wvalid;

//     if((io.cpu->control.en==true&&io.cpu->control.wen==true&&state==idle)||wvalid==false &&state==WRITE_WAIT){
//         wvalid=true;
//     }else if(state==IDLE || (io.axi_w->m.wvalid_out==true&&io.axi_w->m.wready_out==true)){
//         wvalid=false;
//     }
    
//     if(state == WRITE_WAIT){
//         io.axi_w->m.wdata_out = io.cpu->control.wdata;
//         io.axi_w->m.wstrb_out = io.cpu->control.sel;
//         io.axi_w->m.wlast_out = io.cpu->control.last;
//     }else{
//         io.axi_w->m.wdata_out = 0;
//         io.axi_w->m.wstrb_out = 0;
//         io.axi_w->m.wlast_out = 0;
//     }

//     if(io.axi_r->m.rvalid_in&&io.axi_r->m.rvalid_in){
//         rflag=false;
//     }else if(state==ARREADY_WAIT){
//         rflag=true;
//     }

//     if(io.axi_b->m.bvalid_in){
//         wflag=false;
//     }
//     else if(state==WRITE_WAIT){
//         wflag=true;
//     }

// }

// void ADAPTOR::seq(){
//     if(state==IDLE){
//         if(io.cpu->control.en==true&&io.cpu->control.wen==true&&!wflag){
//             state==WRITE_WAIT;
//         }
//         else if(io.cpu->control.en==true&&io.cpu->control.wen==false&&!rflag){
//             state==ARREADY_WAIT;
//         }
//         else{
//             state=IDLE;
//         }
//     }
//     else if(state==WRITE_WAIT){
//         if(io.axi_b->m.bvalid_in){
//             state=IDLE;
//         }
//         else{
//             state=WRITE_WAIT;
//         }
//     }
//     else if(state==ARREADY_WAIT){
//         if(io.axi_ar->m.arready_in){
//             state=IDLE;
//         }
//         else{
//             state=ARREADY_WAIT;
//         }
//     }
//     else {
//         state=IDLE;
//     }
//     rdone = state==IDLE&&io.axi_r->m.rvalid_in ? true : false;
//     io.cpu->data.done=rdone|wdone;
//     wdone = state==WRITE_WAIT&&io.axi_w->m.wvalid_in&&io.axi_w->m.wready_out ? true : false;

//     rlast = state==IDLE&&io.axi_r->m.rvalid_in&&io.axi_r->m.rlast_in ? true : false;
//     wlast = io.axi_b->m.bvalid_in ? true : false;
//     io.cpu->data.last = rlast|wlast;

//     if(state==WRITE_WAIT&&io.axi_w->m.wready_out&&io.axi_w->m.wlast_out){
//         wready=true;
//     }else if(state==IDLE){
//         wready=false;
//     }

//     if(state==ARREADY_WAIT&&io.axi_ar->m.arready_in){
//         awready=true;   
//     }else if(state==IDLE){
//         awready=false;
//     }

// }