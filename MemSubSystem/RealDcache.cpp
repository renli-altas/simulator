#include "RealDcache.h"
#include "PhysMemory.h"
#include <oracle.h>
#include <cassert>
#include <cstddef>
#include <cstring>

namespace {

constexpr uint8_t kLoadRespSrcSpecial = 1;
constexpr uint8_t kLoadRespSrcMshrFill = 2;
constexpr uint8_t kLoadRespSrcWbBypass = 3;
constexpr uint8_t kLoadRespSrcDcacheHit = 4;
constexpr uint8_t kLoadRespSrcReplayBankConflict = 11;
constexpr uint8_t kLoadRespSrcReplayMshrPendingGuard = 12;
constexpr uint8_t kLoadRespSrcReplayMshrHit = 13;
constexpr uint8_t kLoadRespSrcReplayMshrFull = 14;
constexpr uint8_t kLoadRespSrcReplayFirstAlloc = 15;
constexpr uint8_t kLoadRespSrcReplaySameCycleStore = 16;

struct PendingMissLine {
    bool valid = false;
    uint32_t set_idx = 0;
    uint32_t tag = 0;
};

bool pending_miss_contains(const PendingMissLine *pending_miss_lines,
                           int pending_miss_count, uint32_t set_idx,
                           uint32_t tag) {
    for (int idx = 0; idx < pending_miss_count; idx++) {
        if (!pending_miss_lines[idx].valid) {
            continue;
        }
        if (pending_miss_lines[idx].set_idx == set_idx &&
            pending_miss_lines[idx].tag == tag) {
            return true;
        }
    }
    return false;
}

void pending_miss_add(PendingMissLine *pending_miss_lines,
                      int &pending_miss_count, int pending_miss_capacity,
                      uint32_t set_idx, uint32_t tag) {
    if (pending_miss_contains(pending_miss_lines, pending_miss_count, set_idx,
                              tag)) {
        return;
    }
    // Assert(pending_miss_count < pending_miss_capacity);
    if(pending_miss_count >= pending_miss_capacity) {
        return;
    }
    pending_miss_lines[pending_miss_count].valid = true;
    pending_miss_lines[pending_miss_count].set_idx = set_idx;
    pending_miss_lines[pending_miss_count].tag = tag;
    pending_miss_count++;
}

bool has_same_cycle_store_hazard(const S1S2Reg &pipe, uint32_t load_addr) {
    for (int i = 0; i < LSU_STA_COUNT; ++i) {
        const auto &store = pipe.stores[i];
        if (!store.valid || store.replayed) {
            continue;
        }
        if (cache_line_match(load_addr, store.addr)) {
            return true;
        }
    }
    return false;
}

}

// ─────────────────────────────────────────────────────────────────────────────
// Constructor / init
// ─────────────────────────────────────────────────────────────────────────────
// RealDcache::RealDcache(axi_interconnect::AXI_Interconnect *ic)
//     : ic_(ic) {}
void RealDcache::init() {
    init_dcache();
    s1s2_cur = {};
    s1s2_nxt = {};
    for (auto &pending_write : pending_writes_) {
        pending_write = {};
    }
    for (auto &lru_update : lru_updates_) {
        lru_update = {};
    }
}

bool RealDcache::special_load_addr(uint32_t addr,uint32_t &mem_val,MicroOp &uop){
    // Timer addresses (0x1fd0e000, 0x1fd0e004) are classified as MMIO and
    // should be routed through PeripheralAxi instead of DCache.
    // if(addr != OPENSBI_TIMER_LOW_ADDR && addr != OPENSBI_TIMER_HIGH_ADDR)
    // {
    //     return false;
    // }
    // Assert(addr != OPENSBI_TIMER_LOW_ADDR && addr != OPENSBI_TIMER_HIGH_ADDR &&
    //        "Timer address reached DCache! Should be routed via MMIO path.");
    (void)mem_val;
    uop.dbg.difftest_skip = false;
    return false;
}

RealDcache::CoherentQueryResult
RealDcache::query_coherent_word(uint32_t addr, uint32_t &data) const {
    const AddrFields f = decode(addr);

    for (int w = 0; w < DCACHE_WAYS; w++) {
        if (!valid_array[f.set_idx][w] || tag_array[f.set_idx][w] != f.tag) {
            continue;
        }
        data = data_array[f.set_idx][w][f.word_off];
        for (int p = 0; p < LSU_STA_COUNT; p++) {
            const auto &pw = pending_writes_[p];
            if (!pw.valid || pw.set_idx != f.set_idx ||
                pw.way_idx != static_cast<uint32_t>(w) ||
                pw.word_off != f.word_off) {
                continue;
            }
            apply_strobe(data, pw.data, pw.strb);
        }
        return CoherentQueryResult::Hit;
    }

    if (write_buffer_lookup_word(addr, data)) {
        return CoherentQueryResult::Hit;
    }

    if (find_mshr_entry(f.set_idx, f.tag)) {
        return CoherentQueryResult::Retry;
    }

    return CoherentQueryResult::Miss;
}

// ─────────────────────────────────────────────────────────────────────────────
// Stage 1 — called from comb().
//
// Reads incoming requests from in.lsu2dcache, detects bank conflicts, and
// snapshots the SRAM arrays into s1s2_nxt.
// ─────────────────────────────────────────────────────────────────────────────
void RealDcache::stage1_comb() {
    s1s2_nxt = {};

    bool mshr_fill = in.mshr2dcache->fill.valid;
    AddrFields mshr_f = decode(in.mshr2dcache->fill.addr);
    uint32_t mshr_fill_bank = mshr_fill ? mshr_f.bank : ~0u;         

    struct ReqInfo {
        bool     valid;
        AddrFields f;
        bool     conflict;
        bool     mshr_hit;
    } reqs[LSU_LDU_COUNT + LSU_STA_COUNT] = {};

    for (int i = 0; i < LSU_LDU_COUNT; i++) {
        const LoadReq &req = in.lsu2dcache->req_ports.load_ports[i];
        reqs[i].valid = req.valid;
        if (req.valid)
            reqs[i].f.bank = decode(req.addr).bank;
        reqs[i].f =  decode(req.addr);
    }
    for (int i = 0; i < LSU_STA_COUNT; i++) {
        const StoreReq &req = in.lsu2dcache->req_ports.store_ports[i];
        int idx = LSU_LDU_COUNT + i;
        reqs[idx].valid = req.valid;
        reqs[idx].f = decode(req.addr);
    }

    // ① Inter-request conflicts: each request loses to the first earlier
    //   request on the same bank (priority: lower index wins).
    // for (int i = 1; i < LSU_LDU_COUNT + LSU_STA_COUNT; i++) {
    //     if (!reqs[i].valid) continue;
    //     for (int j = 0; j < i; j++) {
    //         if (reqs[j].valid && reqs[j].f.bank == reqs[i].f.bank) {
    //             reqs[i].conflict = true;
    //             break;
    //         }
    //     }
    // }


    // ② Fill bank conflict: the MSHR fill write occupies one SRAM bank this
    //   cycle.  Any request (including port 0) to the same bank must be
    //   replayed so it reads the correct post-fill SRAM state next cycle.
    for (int i = 0; i < LSU_LDU_COUNT + LSU_STA_COUNT; i++) {
        if (!reqs[i].valid || reqs[i].conflict) continue;
        if (mshr_fill && reqs[i].f.bank == mshr_fill_bank)
            reqs[i].conflict = true;
    }//可优化：MSHR replay

    for (int i = 0; i < LSU_LDU_COUNT + LSU_STA_COUNT; i++) {
        if (!reqs[i].valid || reqs[i].conflict) continue;
        // Only snapshot true MSHR ownership here.
        // Same-cycle miss allocations are tracked later in stage2 after the
        // older request is proven to be a real miss rather than a hit/bypass.
        reqs[i].mshr_hit = find_mshr_entry(reqs[i].f.set_idx, reqs[i].f.tag);
    }

    for (int i = 0; i < LSU_LDU_COUNT; i++) {
        const LoadReq &req = in.lsu2dcache->req_ports.load_ports[i];
        S1S2Reg::LoadSlot &slot = s1s2_nxt.loads[i];
        if (!req.valid) continue;

        slot.valid    = true;
        slot.addr     = req.addr;
        slot.uop      = req.uop;
        slot.req_id   = req.req_id;
        slot.replayed = reqs[i].conflict;
        slot.mshr_hit = reqs[i].mshr_hit;
        AddrFields f  = decode(req.addr);
        slot.set_idx  = f.set_idx;
        if(reqs[i].conflict) continue; // Don't need to snapshot SRAM if we're going to replay due to bank conflict
        
        #if !CONFIG_BSD
        for (int w = 0; w < DCACHE_WAYS; w++) {
            slot.tag_snap[w]   = tag_array[f.set_idx][w];
            slot.valid_snap[w] = valid_array[f.set_idx][w];
            slot.dirty_snap[w] = dirty_array[f.set_idx][w];
            for (int d = 0; d < DCACHE_LINE_WORDS; d++)
                slot.data_snap[w][d] = data_array[f.set_idx][w][d];
        }
        #else
            // For BSD, we don't need to capture the full SRAM snapshot for each load slot,
            // since we will simply replay the load on any potential bank conflict or MSHR hit
            out.bsd_out[i]->valid = true;
            out.bsd_out[i]->set_idx = f.set_idx;
            memcpy(slot.tag_snap, in.bsd_in[i]->tag, sizeof(slot.tag_snap));
            memcpy(slot.valid_snap, in.bsd_in[i]->valid, sizeof(slot.valid_snap));
            memcpy(slot.dirty_snap, in.bsd_in[i]->dirty, sizeof(slot.dirty_snap));
            for (int w = 0; w < DCACHE_WAYS; w++) {
                for (int d = 0; d < DCACHE_LINE_WORDS; d++) {
                    slot.data_snap[w][d] = in.bsd_in[i]->data[w][d];
                }
            }
        #endif
    }

    for (int i = 0; i < LSU_STA_COUNT; i++) {
        const StoreReq &req = in.lsu2dcache->req_ports.store_ports[i];
        S1S2Reg::StoreSlot &slot = s1s2_nxt.stores[i];
        if (!req.valid) continue;

        int idx       = LSU_LDU_COUNT + i;
        slot.valid    = true;
        slot.addr     = req.addr;
        slot.data     = req.data;
        slot.strb     = static_cast<uint8_t>(req.strb);
        slot.uop      = req.uop;
        slot.req_id   = req.req_id;
        slot.replayed = reqs[idx].conflict;
        slot.mshr_hit = reqs[idx].mshr_hit;
        AddrFields f  = decode(req.addr);
        slot.set_idx  = f.set_idx;
        if(reqs[idx].conflict) continue; // Don't need to snapshot SRAM if we're going to replay due to bank conflict
        for (int w = 0; w < DCACHE_WAYS; w++) {
        #if !CONFIG_BSD
            slot.tag_snap[w]   = tag_array[f.set_idx][w];
            slot.valid_snap[w] = valid_array[f.set_idx][w];
            slot.dirty_snap[w] = dirty_array[f.set_idx][w];
            for (int d = 0; d < DCACHE_LINE_WORDS; d++)
                slot.data_snap[w][d] = data_array[f.set_idx][w][d];
        #else
            out.bsd_out[i+LSU_LDU_COUNT]->valid = true;
            out.bsd_out[i+LSU_LDU_COUNT]->set_idx = f.set_idx;
            memcpy(slot.tag_snap, in.bsd_in[i+LSU_LDU_COUNT]->tag, sizeof(slot.tag_snap));
            memcpy(slot.valid_snap, in.bsd_in[i+LSU_LDU_COUNT]->valid, sizeof(slot.valid_snap));
            memcpy(slot.dirty_snap, in.bsd_in[i+LSU_LDU_COUNT]->dirty, sizeof(slot.dirty_snap));
            for (int w = 0; w < DCACHE_WAYS; w++) {
                for (int d = 0; d < DCACHE_LINE_WORDS; d++) {
                    slot.data_snap[w][d] = in.bsd_in[i+LSU_LDU_COUNT]->data[w][d];
                }
            }
            // For BSD, we don't need to capture the full SRAM snapshot for each store slot,
            // since we will simply replay the store on any potential bank conflict or MSHR hit
            
        #endif
        }
    }
}

void RealDcache::prepare_wb_queries_for_next_stage2() {
    // Assert(out.dcache2wb != nullptr && "out.dcache2wb pointer not set");

    for (int i = 0; i < LSU_LDU_COUNT; i++) {
        out.dcache2wb->bypass_req[i] = {};
        const S1S2Reg::LoadSlot &slot = s1s2_nxt.loads[i];
        if (!slot.valid || slot.replayed) {
            continue;
        }
        out.dcache2wb->bypass_req[i].valid = true;
        out.dcache2wb->bypass_req[i].addr = slot.addr;
    }

    for (int i = 0; i < LSU_STA_COUNT; i++) {
        out.dcache2wb->merge_req[i] = {};
        const S1S2Reg::StoreSlot &slot = s1s2_nxt.stores[i];
        if (!slot.valid || slot.replayed) {
            continue;
        }
        out.dcache2wb->merge_req[i].valid = true;
        out.dcache2wb->merge_req[i].addr = slot.addr;
        out.dcache2wb->merge_req[i].data = slot.data;
        out.dcache2wb->merge_req[i].strb = slot.strb;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Stage 2 — called from comb().
//
// Processes s1s2_cur (the pipeline register from the previous cycle):
//   - Tag comparison → hit or miss.
//   - Hit: extract data, form response.
//   - Miss: set mshr_.in / wb_.in signals; replay if resources are full.
//
// Reads mshr_.out (populated by comb_outputs()) and wb_.out.
// Sets mshr_.in.ports[] and wb_.in.push_ports[] signals to be processed
// by mshr_.comb_inputs() / wb_.comb_inputs() called afterwards.
// ─────────────────────────────────────────────────────────────────────────────
void RealDcache::stage2_comb() {
    
    out.dcache2lsu->resp_ports.clear();
    for (auto &pending_write : pending_writes_) {
        pending_write = {};
    }
    for (auto &lru_update : lru_updates_) {
        lru_update = {};
    }

    uint32_t mshr_free_entries = in.mshr2dcache->free;
    AddrFields mshr_f = decode(in.mshr2dcache->fill.addr);
    PendingMissLine pending_miss_lines[LSU_LDU_COUNT + LSU_STA_COUNT] = {};
    int pending_miss_count = 0;

    // ── Load ports ────────────────────────────────────────────────────────────
    for (int i = 0; i < LSU_LDU_COUNT; i++) {
        out.dcache2mshr->load_reqs[i].valid = false; // Default to no load request; set to true on load miss
        const S1S2Reg::LoadSlot &slot = s1s2_cur.loads[i];
        LoadResp &resp = out.dcache2lsu->resp_ports.load_resps[i];

        if (!slot.valid) continue;

        if (slot.replayed) {
            resp.valid  = true;
            resp.replay = 3;
            resp.req_id = slot.req_id;
            resp.uop    = slot.uop;
            // resp.debug_addr = slot.addr;
            // resp.debug_src = kLoadRespSrcReplayBankConflict;
            continue;
        }

        AddrFields f          = decode(slot.addr);
        uint32_t tag_expected = f.tag;
        const bool mshr_pending_line =
            pending_miss_contains(pending_miss_lines, pending_miss_count,
                                  slot.set_idx, tag_expected) ||
            slot.mshr_hit || find_mshr_entry(slot.set_idx, tag_expected);
        const bool same_cycle_store_hazard =
            has_same_cycle_store_hazard(s1s2_cur, slot.addr);

        uint32_t mem_val = 0;
        MicroOp response_uop = slot.uop;

        bool is_special = special_load_addr(slot.addr, mem_val, response_uop);

        int hit_way = -1;
        for (int w = 0; w < DCACHE_WAYS; w++) {
            if (slot.valid_snap[w] && slot.tag_snap[w] == tag_expected) {
                hit_way = w;
                break;
            }
        }
        const bool mshr_fill_match =
            in.mshr2dcache->fill.valid && mshr_f.set_idx == slot.set_idx &&
            mshr_f.tag == tag_expected;

        if(is_special){
            resp.valid = true;
            resp.data = mem_val;
            resp.uop = response_uop;
            resp.replay = 0;
            resp.req_id = slot.req_id;
            // resp.debug_addr = slot.addr;
            // resp.debug_src = kLoadRespSrcSpecial;
        }
        else if (same_cycle_store_hazard) {
            // Stores issued from STQ in the same cycle are older than any
            // speculative load still in LDQ, but their cache-state update does
            // not reach SRAM until seq(). Without this guard, a same-cycle load
            // can observe the stale s1 snapshot instead of the older store.
            resp.valid = true;
            resp.replay = 3;
            resp.req_id = slot.req_id;
            resp.uop = slot.uop;
            // resp.debug_addr = slot.addr;
            // resp.debug_src = kLoadRespSrcReplaySameCycleStore;
        }
        else if (mshr_pending_line && mshr_fill_match) {
            resp.valid = true;
            resp.data = in.mshr2dcache->fill.data[f.word_off];
            resp.uop = slot.uop;
            resp.replay = 0;
            resp.req_id = slot.req_id;
            // resp.debug_addr = slot.addr;
            // resp.debug_src = kLoadRespSrcMshrFill;
        }
        else if (mshr_pending_line) {
            // Once a line has an active MSHR/fill in flight, the cache snapshot
            // seen in s1s2_cur can be stale relative to the eventual refill or
            // same-line store merges. Returning a plain hit here lets PTW/LSU
            // observe transient old PTE/data values and commit false faults.
            resp.valid = true;
            resp.replay = 2;
            resp.req_id = slot.req_id;
            resp.uop = slot.uop;
            // resp.debug_addr = slot.addr;
            // resp.debug_src = kLoadRespSrcReplayMshrPendingGuard;
        }
        else if (in.wb2dcache->bypass_resp[i].valid) {
            // Same-line dirty victims in WriteBuffer are newer than a clean
            // line that may have been refilled into DCache before WB drains.
            resp.valid = true;
            resp.replay = 0;
            resp.data = in.wb2dcache->bypass_resp[i].data;
            resp.uop = slot.uop;
            resp.req_id = slot.req_id;
            // resp.debug_addr = slot.addr;
            // resp.debug_src = kLoadRespSrcWbBypass;
        }
        else if (hit_way >= 0 ) {
            // ── Cache Hit ────────────────────────────────────────────────────
            resp.valid  = true;
            resp.replay = 0;
            resp.data   = slot.data_snap[hit_way][f.word_off];
            resp.uop    = slot.uop;
            resp.req_id = slot.req_id;
            // resp.debug_addr = slot.addr;
            // resp.debug_src = kLoadRespSrcDcacheHit;
            lru_updates_[i] = {true, slot.set_idx, (wire<DCACHE_WAY_BITS_PLUS>)(hit_way)};
        } else {
            if(mshr_fill_match){
                resp.valid = true;
                resp.data = in.mshr2dcache->fill.data[f.word_off];
                resp.uop = slot.uop;
                resp.replay = 0; // waiting for fill to complete, replay next cycle
                resp.req_id = slot.req_id;
                // resp.debug_addr = slot.addr;
                // resp.debug_src = kLoadRespSrcMshrFill;
            }
            else if (mshr_pending_line) {
                resp.valid = true;
                resp.replay = 2; // MSHR full, replay later
                resp.req_id = slot.req_id;
                resp.uop = slot.uop;
                // resp.debug_addr = slot.addr;
                // resp.debug_src = kLoadRespSrcReplayMshrHit;
            }
            else if(mshr_free_entries == 0){
                    resp.valid = true;
                    resp.replay = 1; // MSHR full, replay later
                    resp.req_id = slot.req_id;
                    resp.uop = slot.uop;
                    // resp.debug_addr = slot.addr;
                    // resp.debug_src = kLoadRespSrcReplayMshrFull;
            }
            else{
                out.dcache2mshr->load_reqs[i].valid = true;
                out.dcache2mshr->load_reqs[i].addr  = slot.addr;
                out.dcache2mshr->load_reqs[i].uop   = slot.uop;
                out.dcache2mshr->load_reqs[i].req_id = slot.req_id;
                pending_miss_add(pending_miss_lines, pending_miss_count,
                                 LSU_LDU_COUNT + LSU_STA_COUNT, slot.set_idx,
                                 tag_expected);
                // First miss allocation must still return a replay response,
                // otherwise LSU keeps waiting forever for a one-shot request.
                resp.valid = true;
                resp.replay = 2;
                resp.req_id = slot.req_id;
                resp.uop = slot.uop;
                // resp.debug_addr = slot.addr;
                // resp.debug_src = kLoadRespSrcReplayFirstAlloc;
                mshr_free_entries = mshr_free_entries - 1;
            }
        } 
    }

    // ── Store ports ───────────────────────────────────────────────────────────
    for (int i = 0; i < LSU_STA_COUNT; i++) {
        out.dcache2mshr->store_reqs[i].valid = false; // Default to no store request; set to true on store miss
        out.dcache2mshr->store_hit_updates[i].valid = false;
        const S1S2Reg::StoreSlot &slot = s1s2_cur.stores[i];
        StoreResp &resp = out.dcache2lsu->resp_ports.store_resps[i];

        if (!slot.valid) continue;

        if (slot.replayed) {
            resp.valid  = true;
            resp.replay = 3;
            resp.req_id = slot.req_id;
            continue;
        }

        AddrFields f          = decode(slot.addr);
        uint32_t tag_expected = f.tag;

        int hit_way = -1;
        for (int w = 0; w < DCACHE_WAYS; w++) {
            if (slot.valid_snap[w] && slot.tag_snap[w] == tag_expected) {
                hit_way = w;
                break;
            }
        }

        if (hit_way >= 0) {
            // ── Store Hit ────────────────────────────────────────────────────

            // The line has been reallocated in DCache, but an older eviction of
            // the same line is already on the AXI write path and can no longer
            // absorb merges. Accepting the hit here would let cache state move
            // ahead of the in-flight writeback snapshot, so force a replay
            // until the older WB entry drains.
            if (in.wb2dcache->merge_resp[i].busy) {
                resp.valid = true;
                resp.replay = 3;
                resp.req_id = slot.req_id;
            }

            // If fill writes the same set/way in this cycle, this hit-line is
            // being replaced at seq(). A store-hit "success" here would be
            // overwritten by fill commit, so force replay.
            else if (in.mshr2dcache->fill.valid &&
                mshr_f.set_idx == slot.set_idx &&
                static_cast<int>(in.mshr2dcache->fill.way) == hit_way) {
                resp.valid = true;
                resp.replay = 3;
                resp.req_id = slot.req_id;
            }else {
                resp.valid  = true;
                resp.replay = 0;
                resp.req_id = slot.req_id;

                pending_writes_[i] = {
                    true,
                    slot.set_idx,
                    static_cast<wire<DCACHE_WAY_BITS_PLUS>>(hit_way),
                    static_cast<wire<DCACHE_OFFSET_BITS>>(f.word_off),
                    slot.data,
                    slot.strb
                };
                out.dcache2mshr->store_hit_updates[i].valid = true;
                out.dcache2mshr->store_hit_updates[i].set_idx = slot.set_idx;
                out.dcache2mshr->store_hit_updates[i].way_idx =
                    static_cast<wire<DCACHE_WAY_BITS_PLUS>>(hit_way);
                out.dcache2mshr->store_hit_updates[i].word_off = static_cast<wire<DCACHE_OFFSET_BITS>>(f.word_off);
                out.dcache2mshr->store_hit_updates[i].data = slot.data;
                out.dcache2mshr->store_hit_updates[i].strb = slot.strb;
                lru_updates_[LSU_LDU_COUNT + i] = {true, slot.set_idx, static_cast<wire<DCACHE_WAY_BITS_PLUS>>(hit_way)};
            }
        } else {
            if(in.wb2dcache->merge_resp[i].valid){
                resp.valid = true;
                resp.replay = 0;
                resp.req_id = slot.req_id;
            }
            else if (in.wb2dcache->merge_resp[i].busy) {
                // The matching writeback line has already been issued to AXI.
                // Do not ack the store and do not allocate a fresh MSHR for the
                // same line; simply retry after the in-flight WB drains.
                resp.valid = true;
                resp.replay = 3;
                resp.req_id = slot.req_id;
            }
            else if(in.mshr2dcache->fill.valid && mshr_f.set_idx == slot.set_idx && mshr_f.tag == tag_expected){
                resp.valid = true;
                resp.replay = 2; // waiting for fill to complete, replay next cycle
                resp.req_id = slot.req_id;
            }
            else if (pending_miss_contains(pending_miss_lines,
                                           pending_miss_count, slot.set_idx,
                                           tag_expected) ||
                     slot.mshr_hit || find_mshr_entry(slot.set_idx, tag_expected)) {
                out.dcache2mshr->store_reqs[i].valid = true;
                out.dcache2mshr->store_reqs[i].addr  = slot.addr;
                out.dcache2mshr->store_reqs[i].data  = slot.data;
                out.dcache2mshr->store_reqs[i].strb  = slot.strb;
                out.dcache2mshr->store_reqs[i].uop   = slot.uop;
                out.dcache2mshr->store_reqs[i].req_id = slot.req_id;
                resp.valid = true;
                resp.replay = 0;
                resp.req_id = slot.req_id;
            }
            else if(mshr_free_entries == 0){
                resp.valid = true;
                resp.replay = 1; // MSHR full, replay later
                resp.req_id = slot.req_id;
            }
            else {
                out.dcache2mshr->store_reqs[i].valid = true;
                out.dcache2mshr->store_reqs[i].addr  = slot.addr;
                out.dcache2mshr->store_reqs[i].data  = slot.data;
                out.dcache2mshr->store_reqs[i].strb  = slot.strb;
                out.dcache2mshr->store_reqs[i].uop   = slot.uop;
                out.dcache2mshr->store_reqs[i].req_id = slot.req_id;
                pending_miss_add(pending_miss_lines, pending_miss_count,
                                 LSU_LDU_COUNT + LSU_STA_COUNT, slot.set_idx,
                                 tag_expected);
                resp.valid = true;
                resp.replay = 0;
                resp.req_id = slot.req_id;
                resp.is_cache_miss = true;
                mshr_free_entries = mshr_free_entries - 1;
            }
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// comb — top-level combinational evaluation for one cycle.
//
// Ordering:
//   1. Set mshr_.in lookups — provide s1s2_cur line addresses (stable from seq)
//   2. mshr_.comb_outputs() — compute lookup results, fill delivery, full flag
//      MUST precede stage1_comb() so fill_set_idx is known for bank-conflict
//      detection against the current cycle's new requests.
//   3. wb_.comb_outputs()   — compute full / free_count
//   4. stage1_comb()        — snapshot SRAMs, detect bank conflicts
//      (inter-request + fill-bank conflicts both handled here)
//   5. stage2_comb()        — tag compare, responses, set alloc/secondary/push
//   6. Bridge IC outputs → mshr_.axi_in, wb_.axi_in
//   7. mshr_.comb_inputs()  — process alloc/secondary, fill axi_out
//   8. wb_.comb_inputs()    — process pushes, fill axi_out
//   9. Bridge mshr_.axi_out, wb_.axi_out → IC inputs
// ─────────────────────────────────────────────────────────────────────────────
void RealDcache::comb() {
    // assert(in.lsu2dcache  != nullptr && "in.lsu2dcache pointer not set");
    // assert(out.dcache2lsu  != nullptr && "out.dcache2lsu pointer not set");

    stage1_comb();
    prepare_wb_queries_for_next_stage2();

    stage2_comb();


    // out.dcache2lsu->resp_ports.mshr_replay = mshr_.out.mshr_replay;
}

// ─────────────────────────────────────────────────────────────────────────────
// seq — advance all state on the simulated clock edge.
// ─────────────────────────────────────────────────────────────────────────────
void RealDcache::seq() {
    // 1. Advance pipeline register.
    s1s2_cur = s1s2_nxt;

    for (int i = 0; i < LSU_STA_COUNT; i++) {
        const PendingWrite &pw = pending_writes_[i];
        if (!pw.valid) continue;
        apply_strobe(data_array[pw.set_idx][pw.way_idx][pw.word_off],
                     pw.data, pw.strb);
        dirty_array[pw.set_idx][pw.way_idx] = true;
    }


    // 2. Apply store hits.
    if (in.mshr2dcache->fill.valid) {
        write_dcache_line(decode(in.mshr2dcache->fill.addr).set_idx,
                          in.mshr2dcache->fill.way,
                          decode(in.mshr2dcache->fill.addr).tag,
                          in.mshr2dcache->fill.data);
        if (in.mshr2dcache->fill.dirty) {
            dirty_array[decode(in.mshr2dcache->fill.addr).set_idx]
                       [in.mshr2dcache->fill.way] = true;
        }
    }

    // 4. Apply LRU updates from hit accesses this cycle.
    for (int i = 0; i < LSU_LDU_COUNT + LSU_STA_COUNT; i++) {
        const LruUpdate &u = lru_updates_[i];
        if (!u.valid || u.way == DCACHE_WAYS) continue;
        lru_reset(u.set_idx, static_cast<uint32_t>(u.way));
    }

}
