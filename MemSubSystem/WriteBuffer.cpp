#include "WriteBuffer.h"
#include <cassert>
#include <cstring>

WriteBufferEntry write_buffer_nxt[WB_ENTRIES];

// ─────────────────────────────────────────────────────────────────────────────
// init
// ─────────────────────────────────────────────────────────────────────────────
void WriteBuffer::init() {
    std::memset(&cur, 0, sizeof(cur));
    std::memset(&nxt, 0, sizeof(nxt));
    in.clear();
    out.clear();
}
// ─────────────────────────────────────────────────────────────────────────────
// comb_outputs — Phase 1: compute full flag and free slot count.
//
// Uses nxt.count so that pushes committed in previous comb_inputs() calls
// (within the same cycle) are already reflected.
// ─────────────────────────────────────────────────────────────────────────────
void WriteBuffer::comb_outputs() {
    out.wbmshr.ready = (nxt.count < WB_ENTRIES);

    for(int i=0;i<LSU_LDU_COUNT;i++){
        out.wbdcache.bypass_resp[i].valid = nxt.bypassvalid[i];
        out.wbdcache.bypass_resp[i].data = nxt.bypassdata[i];
    }

    for(int i=0;i<LSU_STA_COUNT;i++){
        out.wbdcache.merge_resp[i].valid = nxt.mergevalid[i];
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// comb_inputs — Phase 2: process push requests, accept B channel responses,
// and fill axi_out with the next AW+W request.
//
// Reads axi_in (bridged from IC by RealDcache before this call).
// Writes axi_out (bridged to IC by RealDcache after this call).
// ─────────────────────────────────────────────────────────────────────────────
void WriteBuffer::comb_inputs() {
    // Default outputs: nothing to send, always ready to accept a write response.
    out.axi_out.req_valid  = false;
    out.axi_out.resp_ready = true;
    if(in.mshrwb.valid){
        if(cur.count < WB_ENTRIES){
            WriteBufferEntry &e = write_buffer_nxt[nxt.tail];
            e.valid    = true;
            e.send     = false;
            e.addr     = in.mshrwb.addr;
            std::memcpy(e.data, in.mshrwb.data, DCACHE_LINE_WORDS * sizeof(uint32_t));

            nxt.tail  = (nxt.tail + 1) % WB_ENTRIES;
            nxt.count++;
        }
        else{
            assert(false && "WriteBuffer overflow: MSHR is producing evictions faster than WriteBuffer can drain them");
        }
    }

    for(int i=0;i<LSU_LDU_COUNT;i++){
         if(in.dcachewb.bypass_req[i].valid){
            int wb_idx = find_wb_entry(in.dcachewb.bypass_req[i].addr);
            if(wb_idx != -1){
                nxt.bypassvalid[i] = true;
                nxt.bypassdata[i] = write_buffer[wb_idx].data[decode(in.dcachewb.bypass_req[i].addr).word_off]; // For simplicity, we only support bypassing the first word in the cache line. Extend as needed.
            }
            else{
                nxt.bypassvalid[i] = false;
                nxt.bypassdata[i] = 0; // Or some default value if not found in the write buffer
            }
        }
        else{
            nxt.bypassvalid[i] = false;
        }
    }
    
    for(int i=0;i<LSU_STA_COUNT;i++){
         if(in.dcachewb.merge_req[i].valid){
            int wb_idx = find_wb_entry(in.dcachewb.merge_req[i].addr);
            if(wb_idx != -1){ // Only merge if the entry is still in the buffer (not being sent out)
                nxt.mergevalid[i] = true;
                WriteBufferEntry &e = write_buffer_nxt[wb_idx];
                uint32_t word_off = decode(in.dcachewb.merge_req[i].addr).word_off;
                uint32_t strb = in.dcachewb.merge_req[i].strb;
                apply_strobe(e.data[word_off], in.dcachewb.merge_req[i].data, strb);
            }
            else{
                nxt.mergevalid[i] = false;
            }
        }
        else{
            nxt.mergevalid[i] = false;
        }
    }
    


    // ── Accept write response (B channel) ────────────────────────────────────
    if (in.axi_in.resp_valid) {
        WriteBufferEntry &head_e = write_buffer[cur.head];
        if (head_e.valid && head_e.send ) {
            write_buffer_nxt[cur.head].valid = false;
            int new_head = (cur.head + 1) % WB_ENTRIES;
            nxt.head  = new_head;
            nxt.count = cur.count - 1;
        }
        nxt.send = 0; // allow sending the next entry
    }

    // ── Issue write request (AW + W) ─────────────────────────────────────────
    // Walk the FIFO from nxt.head to find the first unsent entry.
    if(cur.send == 0){
        const WriteBufferEntry &ne_check = write_buffer_nxt[cur.head];
        if (ne_check.valid && !ne_check.send){
            out.axi_out.req_valid      = true;
            out.axi_out.req_addr       = ne_check.addr;
            out.axi_out.req_total_size = 31; // 32B cacheline
            out.axi_out.req_id         = 0;
            out.axi_out.req_wstrb      = 0xff;
            for (int w = 0; w < DCACHE_LINE_WORDS; w++)
                out.axi_out.req_wdata[w] = ne_check.data[w];

            if (in.axi_in.req_ready)
                write_buffer_nxt[cur.head].send = true;
            nxt.send = 1;
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// seq
// ─────────────────────────────────────────────────────────────────────────────
void WriteBuffer::seq() {
    cur = nxt;
    memcpy(write_buffer, write_buffer_nxt, sizeof(write_buffer));
    nxt = cur; 
}
