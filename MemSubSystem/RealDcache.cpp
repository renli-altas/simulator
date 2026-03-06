#include "RealDcache.h"

#include <cassert>
#include <cstring>

// ─────────────────────────────────────────────────────────────────────────────
// Constructor / init
// ─────────────────────────────────────────────────────────────────────────────
// RealDcache::RealDcache(axi_interconnect::AXI_Interconnect *ic)
//     : ic_(ic) {}

void RealDcache::init() {
    init_dcache();
    std::memset(&s1s2_cur,   0, sizeof(s1s2_cur));
    std::memset(&s1s2_nxt,   0, sizeof(s1s2_nxt));
    std::memset(pending_writes_, 0, sizeof(pending_writes_));
    std::memset(lru_updates_,    0, sizeof(lru_updates_));

}
bool RealDcache::special_load_addr(uint32_t addr,uint32_t &mem_val,MicroOp &uop){
    if (addr == 0x1fd0e000) {
#ifdef CONFIG_BPU
    mem_val = sim_time;
#else
    mem_val = get_oracle_timer();
#endif
    uop.difftest_skip = true;
    return 1;
  } else if (addr == 0x1fd0e004) {
    mem_val = 0;
    uop.difftest_skip = true;
    return 1;
  } else {
    uop.difftest_skip = false;
    return 0;
  }
  return 0;
}

// ─────────────────────────────────────────────────────────────────────────────
// Stage 1 — called from comb().
//
// Reads incoming requests from lsu2dcache, detects bank conflicts, and
// snapshots the SRAM arrays into s1s2_nxt.
// ─────────────────────────────────────────────────────────────────────────────
void RealDcache::stage1_comb() {
    std::memset(&s1s2_nxt, 0, sizeof(s1s2_nxt));

    bool mshr_fill = mshr2dcache->fill.valid;
    AddrFields mshr_f = decode(mshr2dcache->fill.addr);
    uint32_t mshr_fill_bank = mshr_fill ? mshr_f.bank : ~0u;         

    struct ReqInfo {
        bool     valid;
        AddrFields f;
        bool     conflict;
        bool     mshr_hit;
    } reqs[LSU_LDU_COUNT + LSU_STA_COUNT] = {};

    for (int i = 0; i < LSU_LDU_COUNT; i++) {
        const LoadReq &req = lsu2dcache->req_ports.load_ports[i];
        reqs[i].valid = req.valid;
        if (req.valid)
            reqs[i].f.bank = decode(req.addr).bank;
        reqs[i].f =  decode(req.addr);
        dcache2wb->bypass_req[i].valid = req.valid;
        dcache2wb->bypass_req[i].addr = req.addr;
    }
    for (int i = 0; i < LSU_STA_COUNT; i++) {
        const StoreReq &req = lsu2dcache->req_ports.store_ports[i];
        int idx = LSU_LDU_COUNT + i;
        reqs[idx].valid = req.valid;
        if (req.valid)
            reqs[idx].f.bank = decode(req.addr).bank;
        reqs[idx].f = decode(req.addr);
    }

    // ① Inter-request conflicts: each request loses to the first earlier
    //   request on the same bank (priority: lower index wins).
    for (int i = 1; i < LSU_LDU_COUNT + LSU_STA_COUNT; i++) {
        if (!reqs[i].valid) continue;
        for (int j = 0; j < i; j++) {
            if (reqs[j].valid && reqs[j].f.bank == reqs[i].f.bank) {
                reqs[i].conflict = true;
                break;
            }
        }
    }

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
        bool mshr_hit = find_mshr_entry(reqs[i].f.set_idx, reqs[i].f.tag);
        if(mshr_hit)continue;
        for(int j = 0; j < i; j++){
            if(reqs[j].valid&&!reqs[j].conflict){
                if(reqs[j].f.set_idx == reqs[i].f.set_idx && reqs[j].f.tag == reqs[i].f.tag){
                    mshr_hit = true;
                    break;
                }   
            }
        }
        reqs[i].mshr_hit = mshr_hit;
    }

    for(int i = 0;i<LSU_STA_COUNT;i++){
        int idx = LSU_LDU_COUNT + i;
        if(reqs[idx].valid && !reqs[idx].conflict ){
            dcache2wb->merge_req[i].valid = true;
            dcache2wb->merge_req[i].addr = get_addr(reqs[idx].f.set_idx, reqs[idx].f.tag, reqs[idx].f.word_off);
            dcache2wb->merge_req[i].data = lsu2dcache->req_ports.store_ports[i].data;
            dcache2wb->merge_req[i].strb = lsu2dcache->req_ports.store_ports[i].strb;
        }
        else{
            dcache2wb->merge_req[i].valid = false;
        }
    }

    for (int i = 0; i < LSU_LDU_COUNT; i++) {
        const LoadReq &req = lsu2dcache->req_ports.load_ports[i];
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
        for (int w = 0; w < DCACHE_WAYS; w++) {
            slot.tag_snap[w]   = tag_array[f.set_idx][w];
            slot.valid_snap[w] = valid_array[f.set_idx][w];
            slot.dirty_snap[w] = dirty_array[f.set_idx][w];
            for (int d = 0; d < DCACHE_LINE_WORDS; d++)
                slot.data_snap[w][d] = data_array[f.set_idx][w][d];
        }
    }

    for (int i = 0; i < LSU_STA_COUNT; i++) {
        const StoreReq &req = lsu2dcache->req_ports.store_ports[i];
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
            slot.tag_snap[w]   = tag_array[f.set_idx][w];
            slot.valid_snap[w] = valid_array[f.set_idx][w];
            slot.dirty_snap[w] = dirty_array[f.set_idx][w];
            for (int d = 0; d < DCACHE_LINE_WORDS; d++)
                slot.data_snap[w][d] = data_array[f.set_idx][w][d];
        }
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
    std::memset(pending_writes_, 0, sizeof(pending_writes_));
    std::memset(lru_updates_,    0, sizeof(lru_updates_));

    uint32_t mshr_free_entries = mshr2dcache->free;
    AddrFields mshr_f = decode(mshr2dcache->fill.addr);

    // ── Load ports ────────────────────────────────────────────────────────────
    for (int i = 0; i < LSU_LDU_COUNT; i++) {
        const S1S2Reg::LoadSlot &slot = s1s2_cur.loads[i];
        LoadResp &resp = dcache2lsu->resp_ports.load_resps[i];

        if (!slot.valid) continue;

        if (slot.replayed) {
            resp.valid  = true;
            resp.replay = 3;
            resp.req_id = slot.req_id;
            continue;
        }

        AddrFields f          = decode(slot.addr);
        uint32_t tag_expected = f.tag;

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

        if(is_special){
            resp.valid = true;
            resp.data = mem_val;
            resp.uop = response_uop;
            resp.replay = 0;
            resp.req_id = slot.req_id;
        }
        else if (hit_way >= 0 ) {
            // ── Cache Hit ────────────────────────────────────────────────────
            resp.valid  = true;
            resp.replay = 0;
            resp.data   = slot.data_snap[hit_way][f.word_off];
            resp.uop    = slot.uop;
            resp.req_id = slot.req_id;
            lru_updates_[i] = {true, slot.set_idx, hit_way};

        } else {
            if(wb2dcache->bypass_resp[i].valid){
                resp.valid = true;
                resp.replay = 0;
                resp.data = wb2dcache->bypass_resp[i].data;
                resp.uop = slot.uop;
                resp.req_id = slot.req_id;
            }
            else if(mshr2dcache->fill.valid && mshr_f.set_idx == slot.set_idx && mshr_f.tag == tag_expected){
                resp.valid = true;
                resp.data = mshr2dcache->fill.data[f.word_off];
                resp.uop = slot.uop;
                resp.replay = 0; // waiting for fill to complete, replay next cycle
                resp.req_id = slot.req_id;
            }
            else if(slot.mshr_hit){
                resp.valid = true;
                resp.replay = 2; // MSHR full, replay later
                resp.req_id = slot.req_id;
            }
            else if(mshr_free_entries == 0){
                    resp.valid = true;
                    resp.replay = 1; // MSHR full, replay later
                    resp.req_id = slot.req_id;
            }
            else{
                dcache2mshr->load_reqs[i].valid = true;
                dcache2mshr->load_reqs[i].addr  = slot.addr;
                dcache2mshr->load_reqs[i].uop   = slot.uop;
                dcache2mshr->load_reqs[i].req_id = slot.req_id;
                mshr_free_entries = mshr_free_entries - 1;
            }
        } 
    }

    // ── Store ports ───────────────────────────────────────────────────────────
    for (int i = 0; i < LSU_STA_COUNT; i++) {
        const S1S2Reg::StoreSlot &slot = s1s2_cur.stores[i];
        StoreResp &resp = dcache2lsu->resp_ports.store_resps[i];
        int p = LSU_LDU_COUNT + i;

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
            resp.valid  = true;
            resp.replay = 0;
            resp.req_id = slot.req_id;

            pending_writes_[i] = {
                true,
                slot.set_idx,
                static_cast<uint32_t>(hit_way),
                f.word_off,
                slot.data,
                slot.strb
            };
            lru_updates_[LSU_LDU_COUNT + i] = {true, slot.set_idx, hit_way};

        } else {
            if(wb2dcache->merge_resp[i].valid){
                resp.valid = true;
                resp.replay = 0;
                resp.req_id = slot.req_id;
            }
            else if(mshr2dcache->fill.valid && mshr_f.set_idx == slot.set_idx && mshr_f.tag == tag_expected){
                resp.valid = true;
                resp.replay = 2; // waiting for fill to complete, replay next cycle
                resp.req_id = slot.req_id;
            }
            else if(slot.mshr_hit){
                resp.valid = true;
                resp.replay = 2; // MSHR full, replay later
                resp.req_id = slot.req_id;
            }
            else if(mshr_free_entries == 0){
                resp.valid = true;
                resp.replay = 1; // MSHR full, replay later
                resp.req_id = slot.req_id;
            }
            else {
                dcache2mshr->store_reqs[i].valid = true;
                dcache2mshr->store_reqs[i].addr  = slot.addr;
                dcache2mshr->store_reqs[i].data  = slot.data;
                dcache2mshr->store_reqs[i].strb  = slot.strb;
                dcache2mshr->store_reqs[i].uop   = slot.uop;
                dcache2mshr->store_reqs[i].req_id = slot.req_id;
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
    assert(lsu2dcache  != nullptr && "lsu2dcache pointer not set");
    assert(dcache2lsu  != nullptr && "dcache2lsu pointer not set");

    dcache2lsu->resp_ports.clear();

    stage1_comb();

    stage2_comb();


    // dcache2lsu->resp_ports.mshr_replay = mshr_.out.mshr_replay;
}

// ─────────────────────────────────────────────────────────────────────────────
// seq — advance all state on the simulated clock edge.
// ─────────────────────────────────────────────────────────────────────────────
void RealDcache::seq() {
    // 1. Advance pipeline register.
    s1s2_cur = s1s2_nxt;

    // 2. Apply store hits.
    for (int i = 0; i < LSU_STA_COUNT; i++) {
        const PendingWrite &pw = pending_writes_[i];
        if (!pw.valid) continue;
        apply_strobe(data_array[pw.set_idx][pw.way_idx][pw.word_off],
                     pw.data, pw.strb);
        dirty_array[pw.set_idx][pw.way_idx] = true;
    }

    write_dcache_line(decode(mshr2dcache->fill.addr).set_idx, mshr2dcache->fill.way, decode(mshr2dcache->fill.addr).tag, mshr2dcache->fill.data);


    // 4. Apply LRU updates from hit accesses this cycle.
    for (int i = 0; i < LSU_LDU_COUNT + LSU_STA_COUNT; i++) {
        const LruUpdate &u = lru_updates_[i];
        if (!u.valid || u.way < 0) continue;
        uint32_t s = u.set_idx;
        uint8_t max_lru = 0;
        for (int ww = 0; ww < DCACHE_WAYS; ww++)
            if (lru_state[s][ww] > max_lru) max_lru = lru_state[s][ww];
        lru_state[s][u.way] = max_lru + 1;
    }

}
