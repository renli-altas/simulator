#include "WriteBuffer.h"
#include "config.h"
#include "types.h"
#include <cassert>
#include <cstdio>
#include <cstring>

WriteBufferEntry write_buffer_nxt[DCACHE_WB_ENTRIES];

namespace {
uint32_t g_wb_head = 0;
uint32_t g_wb_count = 0;

static constexpr uint8_t kCacheLineReqTotalSize =
    static_cast<uint8_t>(DCACHE_LINE_BYTES - 1u);

static bool is_valid(uint32_t head, uint32_t count, uint32_t idx) {
    if (idx >= DCACHE_WB_ENTRIES) {
        return false;
    }
    if (count == 0) {
        return false;
    }
    uint32_t tail = (head + count) % DCACHE_WB_ENTRIES;
    if (head < tail) {
        return idx >= head && idx < tail;
    } else {
        return idx >= head || idx < tail;
    }
}

static constexpr uint64_t full_line_wstrb_mask() {
    uint64_t mask = 0;
    for (uint32_t i = 0; i < DCACHE_LINE_BYTES && i < 64; i++) {
        mask |= (1ull << i);
    }
    return mask;
}

static int find_wb_entry_in_view(const WriteBufferEntry *entries, uint32_t head,
                                 uint32_t count, uint32_t addr) {
    int best_match = -1;
    const uint32_t line_addr = (addr & ~(DCACHE_LINE_BYTES - 1));
    for (uint32_t i = head, cnt = 0; cnt < count;
         i = (i + 1) % DCACHE_WB_ENTRIES, cnt++) {
        if (is_valid(head, count, i) && entries[i].addr == line_addr) {
            best_match = static_cast<int>(i);
        }
    }
    return best_match;
}

static void clear_axi_req(WBOut &out) {
    auto &axi_out = *out.axi_out;
    axi_out.req_valid = false;
    axi_out.req_addr = 0;
    axi_out.req_total_size = 0;
    axi_out.req_id = 0;
    axi_out.req_wstrb = 0;
    for (int w = 0; w < DCACHE_LINE_WORDS; w++) {
        axi_out.req_wdata[w] = 0;
    }
}

static void drive_axi_req_from_head(WBOut &out, uint32_t send, uint32_t head,uint32_t count,
                                    const WriteBufferEntry *entries) {
    clear_axi_req(out);
    auto &axi_out = *out.axi_out;
    axi_out.resp_ready = true;
    if (send != 0) {
        return;
    }
    const WriteBufferEntry &head_e = entries[head];
    if ( !is_valid(head, count, head) || head_e.send) {
        return;
    }
    axi_out.req_valid = true;
    axi_out.req_addr = head_e.addr;
    axi_out.req_total_size = kCacheLineReqTotalSize;
    axi_out.req_id = 0;
    axi_out.req_wstrb = full_line_wstrb_mask();
    for (int w = 0; w < DCACHE_LINE_WORDS; w++) {
        axi_out.req_wdata[w] = head_e.data[w];
    }
}

} // namespace

bool write_buffer_lookup_word(uint32_t addr, uint32_t &data)
{
    const int wb_idx =
        find_wb_entry_in_view(write_buffer, g_wb_head, g_wb_count, addr);
    if (wb_idx < 0) {
        return false;
    }
    data = write_buffer[wb_idx].data[decode(addr).word_off];
    return true;
}

bool write_buffer_entry_live(uint32_t idx)
{
    return is_valid(g_wb_head, g_wb_count, idx);
}

int WriteBuffer::find_wb_entry(uint32_t addr)
{
    return find_wb_entry_in_view(write_buffer, cur.head, cur.count, addr);
}
// ─────────────────────────────────────────────────────────────────────────────
// init
// ─────────────────────────────────────────────────────────────────────────────
void WriteBuffer::init() {
    std::memset(&cur, 0, sizeof(cur));
    std::memset(&nxt, 0, sizeof(nxt));
    std::memset(write_buffer_nxt, 0, sizeof(write_buffer_nxt));
    g_wb_head = 0;
    g_wb_count = 0;
    in.clear();
    clear_outputs();
}

void WriteBuffer::clear_outputs() {
    out.clear();
}
// ─────────────────────────────────────────────────────────────────────────────
// comb_outputs — Phase 1: compute full flag and free slot count.
//
// Uses nxt.count so that pushes committed in previous comb_inputs() calls
// (within the same cycle) are already reflected.
// ─────────────────────────────────────────────────────────────────────────────
void WriteBuffer::comb_outputs() {
    clear_outputs();
    auto &wbmshr = *out.wbmshr;
    auto &wbdcache = *out.wbdcache;

    // MemSubsystem::comb() calls wb_.comb_outputs() again after wb_.comb_inputs()
    // and before mshr_.comb_inputs(). The MSHR therefore samples the refreshed
    // nxt-count view, and producing one victim in the next cycle is safe as
    // long as there is one actual slot left in the FIFO.
    wbmshr.ready = (nxt.count < DCACHE_WB_ENTRIES);

    for(int i=0;i<LSU_LDU_COUNT;i++){
        wbdcache.bypass_resp[i].valid = nxt.bypassvalid[i];
        wbdcache.bypass_resp[i].data = nxt.bypassdata[i];
    }

    for(int i=0;i<LSU_STA_COUNT;i++){
        wbdcache.merge_resp[i].valid = nxt.mergevalid[i];
        wbdcache.merge_resp[i].busy = nxt.mergebusy[i];
    }
    // Drive the request from nxt view so previously accumulated merges are
    // visible to the write beat snapshot.
    drive_axi_req_from_head(out, cur.send, cur.head, cur.count, write_buffer_nxt);

}

// ─────────────────────────────────────────────────────────────────────────────
// comb_inputs — Phase 2: process push requests, accept B channel responses,
// and fill axi_out with the next AW+W request.
//
// Reads axi_in (bridged from IC by RealDcache before this call).
// Writes axi_out (bridged to IC by RealDcache after this call).
// ─────────────────────────────────────────────────────────────────────────────
void WriteBuffer::comb_inputs() {
    clear_outputs();
    auto &mshrwb = *in.mshrwb;
    auto &dcachewb = *in.dcachewb;
    auto &axi_in = *in.axi_in;
    auto &axi_out = *out.axi_out;

    // Default outputs: nothing to send, always ready to accept a write response.
    for(int i=0;i<LSU_LDU_COUNT;i++){
        nxt.bypassvalid[i] = false;
        nxt.bypassdata[i] = 0; // Or some default value if not found in the write buffer
    }
    
    for(int i=0;i<LSU_STA_COUNT;i++){
        nxt.mergevalid[i] = false;
        nxt.mergebusy[i] = false;
         if(dcachewb.merge_req[i].valid){
            int wb_idx = find_wb_entry_in_view(write_buffer_nxt, nxt.head,
                                               nxt.count,
                                               dcachewb.merge_req[i].addr);
            if(wb_idx != -1){
                WriteBufferEntry &e = write_buffer_nxt[wb_idx];
                const bool head_issue_frozen =
                    (wb_idx == static_cast<int>(cur.head)) &&
                    ((cur.issue_pending != 0u) || (nxt.issue_pending != 0u));
                // Ready-first AXI means the interconnect may capture the head
                // entry one cycle before WriteBuffer observes req_accepted.
                // Once that window opens, same-line merges must stop; otherwise
                // the buffer state moves ahead of the payload already captured
                // by the interconnect.
                if (e.send || head_issue_frozen) {
                    nxt.mergebusy[i] = true;
                } else {
                    nxt.mergevalid[i] = true;
                    uint32_t word_off = decode(dcachewb.merge_req[i].addr).word_off;
                    uint32_t strb = dcachewb.merge_req[i].strb;
                    apply_strobe(e.data[word_off], dcachewb.merge_req[i].data, strb);
                }
            }
            else if(cache_line_match(dcachewb.merge_req[i].addr, mshrwb.addr) &&
                    mshrwb.valid){
                nxt.mergevalid[i] = true;
                uint32_t word_off = decode(dcachewb.merge_req[i].addr).word_off;
                uint32_t strb = dcachewb.merge_req[i].strb;
                apply_strobe(mshrwb.data[word_off], dcachewb.merge_req[i].data,
                             strb);
            }
            
        }
    }
    

    if(mshrwb.valid){
        if(nxt.count < DCACHE_WB_ENTRIES){
            WriteBufferEntry &e = write_buffer_nxt[(nxt.head + nxt.count) % DCACHE_WB_ENTRIES];
            e.send     = false;
            e.addr     = mshrwb.addr;
            std::memcpy(e.data, mshrwb.data, DCACHE_LINE_WORDS * sizeof(uint32_t));
            nxt.count++;
        }
        else{
        }
    }

    for(int i=0;i<LSU_LDU_COUNT;i++){
        if(dcachewb.bypass_req[i].valid){
            int wb_idx = find_wb_entry_in_view(write_buffer_nxt, nxt.head,
                                               nxt.count,
                                               dcachewb.bypass_req[i].addr);
            if(wb_idx != -1){
                nxt.bypassvalid[i] = true;
                nxt.bypassdata[i] =
                    write_buffer_nxt[wb_idx]
                        .data[decode(dcachewb.bypass_req[i].addr).word_off];
            }
            else if(cache_line_match(dcachewb.bypass_req[i].addr, mshrwb.addr) &&
                    mshrwb.valid){
                // Bypass from the MSHR fill data if the requested line matches the line being filled by the MSHR.
                nxt.bypassvalid[i] = true;
                nxt.bypassdata[i] =
                    mshrwb.data[decode(dcachewb.bypass_req[i].addr).word_off];
            }
        }
    }

    // Rebuild current-cycle AXI request after same-cycle merges / pushes.
    drive_axi_req_from_head(out, cur.send, cur.head, cur.count, write_buffer_nxt);

    // AXI interconnect uses ready-first timing, but only req.accepted means the
    // request has actually been captured by the interconnect.
    if (cur.send == 0) {
        WriteBufferEntry &head_e = write_buffer_nxt[cur.head];
        const bool can_issue_head = is_valid(cur.head, cur.count,cur.head) && !head_e.send;
        const bool req_payload_matches_head =
            axi_out.req_valid && (axi_out.req_addr == head_e.addr);
        const bool req_handshake = can_issue_head && axi_in.req_accepted &&
                                   req_payload_matches_head;
        if (req_handshake) {
            head_e.send = true;
            nxt.send = 1;
            nxt.issue_pending = 0;
        } else if (can_issue_head) {
            nxt.send = 0;
            nxt.issue_pending =
                (axi_in.req_ready && req_payload_matches_head) ? 1u : 0u;
        } else {
            nxt.issue_pending = 0;
        }
    } else {
        nxt.issue_pending = 0;
    }

    // ── Accept write response (B channel) ────────────────────────────────────
    if (axi_in.resp_valid) {
        WriteBufferEntry &head_e = write_buffer[cur.head];
        if (is_valid(cur.head, cur.count, cur.head) && head_e.send ) {
            write_buffer_nxt[cur.head].send = false;
            int new_head = (cur.head + 1) % DCACHE_WB_ENTRIES;
            nxt.head  = new_head;
            if (nxt.count > 0) {
                nxt.count--;
            }
        } else {
        }
        nxt.send = 0; // allow sending (or retrying) the head entry
        nxt.issue_pending = 0;
    }

    // ── Issue write request (AW + W) ─────────────────────────────────────────
    // Walk the FIFO from nxt.head to find the first unsent entry.
    
}

// ─────────────────────────────────────────────────────────────────────────────
// seq
// ─────────────────────────────────────────────────────────────────────────────
void WriteBuffer::seq() {
    cur = nxt;
    memcpy(write_buffer, write_buffer_nxt, sizeof(write_buffer));
    g_wb_head = cur.head;
    g_wb_count = cur.count;
    nxt = cur; 
}
