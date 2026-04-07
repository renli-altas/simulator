#pragma once

#include "AbstractDcache.h"
#include "AbstractLsu.h"
#include "MSHR.h"
#include "WriteBuffer.h"
#include "config.h"
#include "types.h"
#include "DcacheConfig.h"   
#include "IO.h"
#include <cstddef>
#include <cstdint>

#define DCACHE_WAY_BITS_PLUS (DCACHE_WAY_BITS + 1) // Extra bit for encoding "no way matched" in LRU updates
// ─────────────────────────────────────────────────────────────────────────────
// Pending store write (hit path) — records a store hit so that seq() can apply
// the byte-merge to the data SRAM.
// ─────────────────────────────────────────────────────────────────────────────
struct PendingWrite {
    wire<1>     valid    = false;
    wire<DCACHE_SET_BITS> set_idx  = 0;
    wire<DCACHE_WAY_BITS> way_idx  = 0;
    wire<DCACHE_OFFSET_BITS> word_off = 0;
    wire<32> data     = 0;
    wire<4>  strb     = 0;   // byte-enable (4 bits used for a 32-bit word)
};

struct FillWrite{
    wire<1>     valid    = false;
    wire<DCACHE_SET_BITS> set_idx  = 0;
    wire<DCACHE_WAY_BITS> way_idx  = 0;
    wire<32> data[DCACHE_LINE_WORDS] = {};
};
// ─────────────────────────────────────────────────────────────────────────────
// S1S2Reg — pipeline register between Stage 1 (SRAM read) and Stage 2 (hit check)
// ─────────────────────────────────────────────────────────────────────────────
struct S1S2Reg {
    // Load slots
    struct LoadSlot {
        wire<1>     valid    = false;
        wire<1>     replayed = false; // bank conflict — do not process in S2
        wire<32> addr     = 0;
        MicroOp  uop;
        wire<32>   req_id   = 0;

        // SRAM snapshot captured in S1 (index already decoded)
        wire<DCACHE_SET_BITS> set_idx  = 0;
        wire<32> tag_snap [DCACHE_WAYS] = {};
        wire<1>  valid_snap[DCACHE_WAYS] = {};
        wire<1>  dirty_snap[DCACHE_WAYS] = {};
        wire<32> data_snap [DCACHE_WAYS][DCACHE_LINE_WORDS] = {};

        wire<1> mshr_hit = false; // for load hits, whether the MSHR entry is the primary one (for write-allocate ordering)
    } loads[LSU_LDU_COUNT];

    // Store slots
    struct StoreSlot {
        wire<1>     valid    = false;
        wire<1>     replayed = false;
        wire<32> addr     = 0;
        wire<32> data     = 0; // word to write
        wire<4>  strb     = 0; // byte-enable
        StqEntry uop;
        wire<32>   req_id   = 0;

        wire<DCACHE_SET_BITS> set_idx  = 0;
        wire<32> tag_snap [DCACHE_WAYS] = {};
        wire<1>  valid_snap[DCACHE_WAYS] = {};
        wire<1>  dirty_snap[DCACHE_WAYS] = {};
        wire<32> data_snap [DCACHE_WAYS][DCACHE_LINE_WORDS] = {};
        wire<1> mshr_hit = false; // for store misses, whether there's a primary MSHR entry matching this line (for write-allocate decision)
    } stores[LSU_STA_COUNT];
};

// ─────────────────────────────────────────────────────────────────────────────
// RealDcache — non-blocking, write-allocate, write-back D-Cache.
//
// Geometry (configurable via config.h):
//   DCACHE_SETS sets × DCACHE_WAYS ways × 32-byte cachelines.
//
// Pipeline: 2 stages.
//   Stage 1 (S1): index decode + SRAM array read → feeds S1S2 pipeline register.
//   Stage 2 (S2): tag compare, hit/miss decision, MSHR/WB interaction → outputs.
//
// All SRAM state is updated in seq() only; comb() is pure-read + output logic.
// ─────────────────────────────────────────────────────────────────────────────


class RealDcache : public AbstractDcache {
public:
    enum class CoherentQueryResult : uint8_t {
        Miss = 0,
        Retry = 1,
        Hit = 2,
    };

    void init() override;
    void comb() override;
    void seq()  override;
    CoherentQueryResult query_coherent_word(uint32_t addr,
                                            uint32_t &data) const;

    // IO ports — set by the owning module (e.g. MemSubsystem) before init().
    DcacheINIO in;
    DcacheOUTIO out;

    // Stage 1: decode incoming requests, detect bank conflicts, snapshot SRAMs.
    void stage1_comb();

    // Drive WB bypass/merge queries for the requests just captured by stage1
    // into s1s2_nxt. WB responses may return one cycle later, so queries must
    // be issued one cycle ahead of the S2 evaluation that will consume them.
    void prepare_wb_queries_for_next_stage2();

    // Stage 2: evaluate S1S2 pipeline register, generate responses.
    void stage2_comb();
private:
    // // Sub-modules
    // MSHR        mshr_;
    // WriteBuffer wb_;

    // ── SRAM arrays (updated only in seq()) ──────────────────────────────────
    

    // ── Pipeline registers ────────────────────────────────────────────────────
    S1S2Reg s1s2_cur; // latched at start of cycle
    S1S2Reg s1s2_nxt; // computed by comb(); committed by seq()

    // ── Pending store hits (recorded by comb(), applied by seq()) ─────────────
    // One pending write per store port.
    PendingWrite pending_writes_[LSU_STA_COUNT];

    // ── LRU update log (recorded by comb(), applied by seq()) ─────────────────
    struct LruUpdate {
        wire<1>     valid   = false;
        wire<DCACHE_SET_BITS> set_idx = 0;
        wire<DCACHE_WAY_BITS_PLUS>  way     = 0;
    } lru_updates_[LSU_LDU_COUNT + LSU_STA_COUNT];


    bool special_load_addr(uint32_t addr,uint32_t& mem_val,MicroOp &uop);
};
