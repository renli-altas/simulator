#include "MSHR.h"

#include <cassert>
#include <cstring>

MSHREntry mshr_entries_nxt[MSHR_ENTRIES];

// ─────────────────────────────────────────────────────────────────────────────
// init
// ─────────────────────────────────────────────────────────────────────────────
void MSHR::init()
{
    std::memset(mshr_entries_nxt, 0, sizeof(mshr_entries_nxt));

    std::memset(&out, 0, sizeof(out));
}

// ─────────────────────────────────────────────────────────────────────────────
// comb_outputs — Phase 1: compute lookups, full, and fill delivery from cur.
//
// Must be called BEFORE stage2_comb() so that lookup results and full flag
// are stable when stage2_comb() decides whether to alloc or add_secondary.
// ─────────────────────────────────────────────────────────────────────────────
void MSHR::comb_outputs()
{   
    out.mshr2dcache.free = MSHR_ENTRIES - cur.mshr_count;
    out.replay_resp.replay = cur.fill; // MSHR full replay code
    out.replay_resp.replay_addr = cur.fill_addr;
}

int MSHR::entries_add(int set_idx, int tag)
{
    for (int i = 0; i < MSHR_ENTRIES; i++)
    {
        if (!mshr_entries_nxt[i].valid)
        {
            mshr_entries_nxt[i].valid = true;
            mshr_entries_nxt[i].issued = false;
            mshr_entries_nxt[i].fill = false;
            mshr_entries_nxt[i].index = set_idx;
            mshr_entries_nxt[i].tag = tag;
            nxt.mshr_count++;
            return i;
        }
    }
    return -1;
}

void MSHR::comb_inputs()
{
    // Default AXI outputs.
    out.axi_out.req_valid = false;
    out.axi_out.req_addr = 0;
    out.axi_out.req_total_size = 0;
    out.axi_out.req_id = 0;
    out.axi_out.resp_ready = false;

    // ── Process alloc and secondary requests ─────────────────────────────────
    for (int i = 0; i < LSU_LDU_COUNT; i++)
    {
        const LoadReq &req = in.dcachemshr.load_reqs[i];
        if (!req.valid)
            continue;
        entries_add(decode(req.addr).set_idx, decode(req.addr).tag);
        if (nxt.mshr_count > MSHR_ENTRIES)
        {
            assert(false && "MSHR full");
        }
    }

    for (int i = 0; i < LSU_STA_COUNT; i++)
    {
        const StoreReq &req = in.dcachemshr.store_reqs[i];
        if (!req.valid)
            continue;
        entries_add(decode(req.addr).set_idx, decode(req.addr).tag);
        if (nxt.mshr_count > MSHR_ENTRIES)
        {
            assert(false && "MSHR full");
        }
    }

    // ── Accept R channel response ─────────────────────────────────────────────
    out.mshr2dcache.fill.valid = false; // default: no fill delivery this cycle
    nxt.fill = false;
    if (in.axi_in.resp_valid)
    {
        uint8_t resp_id = in.axi_in.resp_id;
        if (resp_id < MSHR_ENTRIES)
        {
            const MSHREntry &e = mshr_entries[resp_id];
            if (e.valid && e.issued && !e.fill)
            {
                uint32_t lru_idx = choose_lru_victim(mshr_entries[resp_id].index);
                if (dirty_array[mshr_entries[resp_id].index][lru_idx] && !in.wbmshr.ready)
                {
                }
                else
                {
                    if (dirty_array[mshr_entries[resp_id].index][lru_idx] && in.wbmshr.ready)
                    {
                        out.mshrwb.valid = true;
                        out.mshrwb.addr = get_addr(mshr_entries[resp_id].index, tag_array[mshr_entries[resp_id].index][lru_idx], 0);
                        for (int w = 0; w < DCACHE_LINE_WORDS; w++)
                        {
                            out.mshrwb.data[w] = data_array[mshr_entries[resp_id].index][lru_idx][w];
                        }
                    }
                    out.mshr2dcache.fill.valid = true;
                    out.mshr2dcache.fill.way = lru_idx;
                    out.mshr2dcache.fill.addr = get_addr(mshr_entries[resp_id].index, mshr_entries[resp_id].tag, 0);
                    for (int w = 0; w < DCACHE_LINE_WORDS; w++)
                    {
                        out.mshr2dcache.fill.data[w] = in.axi_in.resp_data[w];
                    }
                    mshr_entries_nxt[resp_id].valid = false; // auto-consume the fill response
                    mshr_entries_nxt[resp_id].issued = false;
                    mshr_entries_nxt[resp_id].fill = false;
                    nxt.fill = true;
                    nxt.fill_addr = out.mshr2dcache.fill.addr;
                    nxt.mshr_count--;
                }
            }
            else
            {
            }
        }
        else
        {
            assert(false && "Invalid MSHR response ID");
        }
    }

    // ── Issue next pending AR ─────────────────────────────────────────────────
    for (int i = 0; i < MSHR_ENTRIES; i++)
    {
        const MSHREntry &ce = mshr_entries[i];
        if (!ce.valid || ce.issued)
            continue;

        out.axi_out.req_valid = true;
        out.axi_out.req_addr = get_addr(ce.index, ce.tag, 0);
        out.axi_out.req_total_size = 31;
        out.axi_out.req_id = static_cast<uint8_t>(i);

        if (in.axi_in.req_ready)
        {
            mshr_entries_nxt[i].issued = true;
        }
        break;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// seq — advance state on the clock edge.
//
// 1. Auto-consume the fill that was delivered this cycle (out.fill_idx).
// 2. cur = nxt.
// 3. Retire or promote fill_consumed entries.
// 4. nxt = cur (reset nxt for next-cycle mutations).
// ─────────────────────────────────────────────────────────────────────────────
void MSHR::seq()
{

    cur = nxt;

    memcpy(mshr_entries, mshr_entries_nxt, sizeof(mshr_entries));

    nxt = cur;
}
