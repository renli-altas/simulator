#include "MSHR.h"

#include <cassert>
#include <cstring>


MSHREntry mshr_entries_nxt[DCACHE_MSHR_ENTRIES];

namespace {
static constexpr uint8_t kCacheLineReqTotalSize =
    static_cast<uint8_t>(DCACHE_LINE_BYTES - 1u);

uint32_t count_valid_mshr_entries(const MSHREntry *entries) {
    uint32_t count = 0;
    for (int i = 0; i < DCACHE_MSHR_ENTRIES; i++) {
        if (entries[i].valid) {
            count++;
        }
    }
    return count;
}

bool victim_has_same_cycle_store_hit(const DcacheMSHRIO &dcachemshr,
                                     uint32_t set_idx, uint32_t way_idx) {
    for (int p = 0; p < LSU_STA_COUNT; ++p) {
        const auto &u = dcachemshr.store_hit_updates[p];
        if (!u.valid) {
            continue;
        }
        if (u.set_idx != set_idx || u.way_idx != way_idx) {
            continue;
        }
        return true;
    }
    return false;
}

int find_next_entry_idx(uint32_t set_idx, uint32_t tag) {
    for (int i = 0; i < DCACHE_MSHR_ENTRIES; i++) {
        const MSHREntry &entry = mshr_entries_nxt[i];
        if (entry.valid && entry.index == set_idx && entry.tag == tag) {
            return i;
        }
    }
    return -1;
}

void merge_store_into_entry(MSHREntry &entry, const StoreReq &req) {
    const AddrFields f = decode(req.addr);
    Assert(f.word_off < DCACHE_LINE_WORDS && "Store merge word offset overflow");
    apply_strobe(entry.merged_store_data[f.word_off], req.data, req.strb);
    entry.merged_store_strb[f.word_off] |= req.strb;
    entry.merged_store_dirty = true;
}

}

// ─────────────────────────────────────────────────────────────────────────────
// init
// ─────────────────────────────────────────────────────────────────────────────
void MSHR::init()
{
    std::memset(&cur, 0, sizeof(cur));
    std::memset(&nxt, 0, sizeof(nxt));
    std::memset(mshr_entries_nxt, 0, sizeof(mshr_entries_nxt));

    out = {};
}

// ─────────────────────────────────────────────────────────────────────────────
// comb_outputs — Phase 1: compute lookups, full, and fill delivery from cur.
//
// Must be called BEFORE stage2_comb() so that lookup results and full flag
// are stable when stage2_comb() decides whether to alloc or add_secondary.
// ─────────────────────────────────────────────────────────────────────────────

void MSHR::comb_outputs()
{   
    const uint32_t mshr_count = count_valid_mshr_entries(mshr_entries);
    out.mshr2dcache.free = DCACHE_MSHR_ENTRIES - mshr_count;
    out.replay_resp.replay = cur.fill; // MSHR full replay code
    out.replay_resp.replay_addr = cur.fill_addr;
    out.replay_resp.free_slots = out.mshr2dcache.free;

    // Registered fill output (from previous cycle comb_inputs()).
    out.mshr2dcache.fill.valid = cur.fill_valid;
    out.mshr2dcache.fill.dirty = cur.fill_dirty;
    out.mshr2dcache.fill.way = cur.fill_way;
    out.mshr2dcache.fill.addr = cur.fill_addr;
    std::memcpy(out.mshr2dcache.fill.data, cur.fill_data,
                sizeof(out.mshr2dcache.fill.data));

    // Registered WB-eviction output (from previous cycle comb_inputs()).
    out.mshrwb.valid = cur.wb_valid;
    out.mshrwb.addr = cur.wb_addr;
    std::memcpy(out.mshrwb.data, cur.wb_data, sizeof(out.mshrwb.data));

    // AXI outputs.
    out.axi_out.req_valid = false;
    out.axi_out.req_addr = 0;
    out.axi_out.req_total_size = 0;
    out.axi_out.req_id = 0;
    // Default to ready; may be deasserted when WB backpressure prevents
    // consuming the current-cycle AXI read response.
    out.axi_out.resp_ready = true;

    if (in.axi_in.resp_valid) {
        const uint8_t resp_id = in.axi_in.resp_id;
        if (resp_id < DCACHE_MSHR_ENTRIES) {
            const MSHREntry &e = mshr_entries[resp_id];
            if (e.valid && e.issued && !e.fill) {
                const uint32_t lru_idx = choose_lru_victim(e.index);
                const bool same_cycle_store_hit =
                    victim_has_same_cycle_store_hit(in.dcachemshr, e.index,
                                                    lru_idx);
                const bool need_wb_evict =
                    dirty_array[e.index][lru_idx] || same_cycle_store_hit;
                if (need_wb_evict && !in.wbmshr.ready) {
                    out.axi_out.resp_ready = false;
                }
            }
        }
    }

    for(int i=0; i<DCACHE_MSHR_ENTRIES; i++){
        const MSHREntry &ce = mshr_entries[i];
        if (!ce.valid || ce.issued)
            continue;

        out.axi_out.req_valid = true;
        out.axi_out.req_addr = get_addr(ce.index, ce.tag, 0);
        out.axi_out.req_total_size = kCacheLineReqTotalSize;
        out.axi_out.req_id = static_cast<uint8_t>(i);
        break;
    }


}

int MSHR::entries_add(int set_idx, int tag, uint32_t &mshr_count)
{   
    if (mshr_count >= DCACHE_MSHR_ENTRIES) {
        Assert(0 && "MSHR full");
    }
    int alloc_idx = -1;
    for (int off = 0; off < DCACHE_MSHR_ENTRIES; off++)
    {
        if (!mshr_entries_nxt[off].valid)
        {
            alloc_idx = off;
            break;
        }
    }
    Assert(alloc_idx >= 0 &&
           "MSHR full but per-cycle traversed count says not full");
    mshr_entries_nxt[alloc_idx].valid = true;
    mshr_entries_nxt[alloc_idx].issued = false;
    mshr_entries_nxt[alloc_idx].fill = false;
    mshr_entries_nxt[alloc_idx].index = set_idx;
    mshr_entries_nxt[alloc_idx].tag = tag;
    mshr_entries_nxt[alloc_idx].merged_store_dirty = false;
    std::memset(mshr_entries_nxt[alloc_idx].merged_store_data, 0,
                sizeof(mshr_entries_nxt[alloc_idx].merged_store_data));
    std::memset(mshr_entries_nxt[alloc_idx].merged_store_strb, 0,
                sizeof(mshr_entries_nxt[alloc_idx].merged_store_strb));
    mshr_count++;
    return alloc_idx;
}

void MSHR::comb_inputs()
{
    uint32_t mshr_count = count_valid_mshr_entries(mshr_entries_nxt);

    // One-cycle pulse/state outputs are generated into nxt and exposed by
    // comb_outputs() in the next cycle.
    nxt.fill = false;
    nxt.fill_valid = false;
    nxt.fill_dirty = false;
    nxt.fill_way = 0;
    std::memset(nxt.fill_data, 0, sizeof(nxt.fill_data));
    nxt.wb_valid = false;
    nxt.wb_addr = 0;
    std::memset(nxt.wb_data, 0, sizeof(nxt.wb_data));

    out.axi_out.resp_ready = true;
    if (in.axi_in.resp_valid) {
        const uint8_t rid = in.axi_in.resp_id;
        if (rid < DCACHE_MSHR_ENTRIES) {
            const MSHREntry re = mshr_entries[rid];
            if (re.valid && re.issued && !re.fill) {
                const uint32_t lru_idx = choose_lru_victim(re.index);
                const bool need_wb_evict = dirty_array[re.index][lru_idx];
                if (need_wb_evict && !in.wbmshr.ready) {
                    out.axi_out.resp_ready = false;
                }
            }
        }
    }

    // ── Process alloc and secondary requests ─────────────────────────────────
    for (int i = 0; i < LSU_LDU_COUNT; i++)
    {
        const LoadReq &req = in.dcachemshr.load_reqs[i];
        if (!req.valid)
            continue;
        entries_add(decode(req.addr).set_idx, decode(req.addr).tag, mshr_count);
    }

    for (int i = 0; i < LSU_STA_COUNT; i++)
    {
        const StoreReq &req = in.dcachemshr.store_reqs[i];
        if (!req.valid)
            continue;
        const AddrFields f = decode(req.addr);
        int entry_idx = find_next_entry_idx(f.set_idx, f.tag);
        if (entry_idx < 0) {
            entry_idx = entries_add(f.set_idx, f.tag, mshr_count);
        }
        merge_store_into_entry(mshr_entries_nxt[entry_idx], req);
    }

    // ── Accept R channel response ─────────────────────────────────────────────
    if (in.axi_in.resp_valid)
    {
        uint8_t resp_id = in.axi_in.resp_id;
        if (resp_id < DCACHE_MSHR_ENTRIES)
        {
            const MSHREntry &e_cur = mshr_entries[resp_id];
            if (e_cur.valid && e_cur.issued && !e_cur.fill)
            {
                const uint32_t fill_set = mshr_entries[resp_id].index;
                const uint32_t fill_tag = mshr_entries[resp_id].tag;
                const uint32_t lru_idx = choose_lru_victim(fill_set);
                const uint32_t victim_tag = tag_array[fill_set][lru_idx];
                const bool same_cycle_store_hit =
                    victim_has_same_cycle_store_hit(in.dcachemshr, fill_set,
                                                    lru_idx);
                bool need_wb_evict =
                    dirty_array[fill_set][lru_idx] || same_cycle_store_hit;
                bool can_consume_resp = (!need_wb_evict) || in.wbmshr.ready;
                if (!can_consume_resp)
                {
                }
                else
                {
                    const uint32_t fill_line_addr = get_addr(fill_set, fill_tag, 0);
                    if (need_wb_evict)
                    {
                        nxt.wb_valid = true;
                        nxt.wb_addr = get_addr(fill_set, victim_tag, 0);
                        for (int w = 0; w < DCACHE_LINE_WORDS; w++)
                        {
                            nxt.wb_data[w] = data_array[fill_set][lru_idx][w];
                        }
                        // Preserve same-cycle store-hit updates that have not
                        // reached data_array yet (committed in RealDcache::seq()).
                        // Apply in store-port order to match seq() semantics.
                        for (int p = 0; p < LSU_STA_COUNT; p++)
                        {
                            const auto &u = in.dcachemshr.store_hit_updates[p];
                            if (!u.valid)
                            {
                                continue;
                            }
                            if (u.set_idx != fill_set)
                            {
                                continue;
                            }
                            if (u.way_idx != lru_idx)
                            {
                                continue;
                            }
                            if (u.word_off >= DCACHE_LINE_WORDS)
                            {
                                continue;
                            }
                            apply_strobe(nxt.wb_data[u.word_off], u.data, u.strb);
                        }
                    }
                    // Same-cycle store miss merges were already applied into
                    // mshr_entries_nxt above. Use that snapshot so the refill
                    // line delivered to DCache includes those merged bytes
                    // instead of overwriting them with stale AXI payload.
                    const MSHREntry &e_fill = mshr_entries_nxt[resp_id];
                    nxt.fill_valid = true;
                    nxt.fill_dirty = e_fill.merged_store_dirty;
                    nxt.fill_way = lru_idx;
                    nxt.fill_addr = fill_line_addr;
                    for (int w = 0; w < DCACHE_LINE_WORDS; w++)
                    {
                        nxt.fill_data[w] = in.axi_in.resp_data[w];
                        if (e_fill.merged_store_strb[w] != 0) {
                            apply_strobe(nxt.fill_data[w],
                                         e_fill.merged_store_data[w],
                                         e_fill.merged_store_strb[w]);
                        }
                    }
                    // AXI read-path check: under direct-memory mode, returned
                    // cachelines must match the backing memory. Under LLC
                    // write-back mode, the response may legally come from a
                    // dirty LLC resident line that has not been written back to
                    // p_memory yet, so this comparison is no longer valid.

                    mshr_entries_nxt[resp_id] = {};
                    nxt.fill = true;
                    nxt.fill_addr = fill_line_addr;
                    if (mshr_count > 0)
                    {
                        mshr_count--;
                    }
                }
            }
            else
            {
            }
        }
        else
        {
            Assert(false && "Invalid MSHR response ID");
        }
    }

    // ── Issue next pending AR ─────────────────────────────────────────────────
    // Handshake note:
    // `req_ready` is only a ready-first hint. Use `req_accepted + req_accepted_id`
    // to mark exactly which MSHR slot completed AR handshake.
    if (in.axi_in.req_accepted) {
        const uint8_t acc_id = in.axi_in.req_accepted_id;
        if (acc_id < DCACHE_MSHR_ENTRIES) {
            const MSHREntry &ce = mshr_entries[acc_id];
            if (ce.valid && !ce.issued) {
                mshr_entries_nxt[acc_id].issued = true;
            }
        }
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
