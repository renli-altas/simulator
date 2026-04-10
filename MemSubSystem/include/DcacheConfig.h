#pragma once
#include <cmath>
#include <cstdint>
#include <config.h>
#include "IO.h"
#define DCACHE_BANKS              4
#define DCACHE_SET_BITS           (__builtin_ctz(DCACHE_SETS))
#define DCACHE_TAG_BITS           (32 - DCACHE_SET_BITS - DCACHE_OFFSET_BITS)
#define DCACHE_WAY_BITS           (__builtin_ctz(DCACHE_WAYS))
#define DCACHE_PLRU_BITS          (DCACHE_WAYS - 1)
#define DCACHE_WB_BITS            (__builtin_ctz(DCACHE_WB_ENTRIES))
#define DCACHE_WB_BITS_PLUS       (__builtin_ctz(DCACHE_WB_ENTRIES + 1))
#define DCACHE_MSHR_BITS          (__builtin_ctz(DCACHE_MSHR_ENTRIES))
#define DCACHE_MSHR_BITS_PLUS     (__builtin_ctz(DCACHE_MSHR_ENTRIES + 1))

#define CONFIG_BSD 1


#define DCACHE_WAY_BITS_PLUS (DCACHE_WAY_BITS + 1) 

static_assert(is_power_of_two_u64(DCACHE_WAYS),
              "Tree-PLRU requires DCACHE_WAYS to be a power of two");

extern uint32_t tag_array  [DCACHE_SETS][DCACHE_WAYS];
extern uint32_t data_array [DCACHE_SETS][DCACHE_WAYS][DCACHE_LINE_WORDS];
extern bool     valid_array[DCACHE_SETS][DCACHE_WAYS];
extern bool     dirty_array[DCACHE_SETS][DCACHE_WAYS];


// Tree-PLRU bits per set. Each internal node records which subtree should be
// chosen next as the replacement candidate.
extern uint8_t  plru_state[DCACHE_SETS][DCACHE_PLRU_BITS];

struct PendingWrite {
    wire<1>     valid    = false;
    wire<DCACHE_SET_BITS> set_idx  = 0;
    wire<DCACHE_WAY_BITS> way_idx  = 0;
    wire<DCACHE_OFFSET_BITS> word_off = 0;
    wire<32> data     = 0;
    wire<4>  strb     = 0;   // byte-enable (4 bits used for a 32-bit word)
};

struct MSHRWrite{
    wire<1> valid = false;
    wire<DCACHE_SET_BITS> set_idx = 0;
    wire<DCACHE_WAY_BITS> way = 0;
    wire<DCACHE_TAG_BITS> tag = 0;
    wire<32> data[DCACHE_LINE_WORDS] = {};
    wire<1> dirty = false;
};
struct LruUpdate {
    wire<1>     valid   = false;
    wire<DCACHE_SET_BITS> set_idx = 0;
    wire<DCACHE_WAY_BITS_PLUS>  way     = 0;
};

struct BSDOUT{
    wire<1> valid;
    wire<DCACHE_SET_BITS> set_idx;
};


struct BSDIN{
    wire<1> valid[DCACHE_WAYS];
    wire<DCACHE_TAG_BITS> tag[DCACHE_WAYS];
    wire<1> dirty[DCACHE_WAYS];
    wire<32> data[DCACHE_WAYS][DCACHE_LINE_WORDS];
};


struct MSHR_FILL{
    wire<1> valid;
    wire<1> dirty;
    wire<32> way;
    wire<32> addr;
    wire<32> data[DCACHE_LINE_WORDS];

};
struct MSHRDcacheIO {
    MSHR_FILL fill;
    wire<DCACHE_MSHR_BITS_PLUS> free;
};

struct DcacheMSHRIO {
    LoadReq load_reqs[LSU_LDU_COUNT];
    StoreReq store_reqs[LSU_STA_COUNT];
    struct StoreHitUpdate {
        wire<1> valid = false;
        wire<DCACHE_SET_BITS> set_idx = 0;
        wire<DCACHE_WAY_BITS> way_idx = 0;
        wire<DCACHE_OFFSET_BITS> word_off = 0;
        wire<32> data = 0;
        wire<8> strb = 0;
    } store_hit_updates[LSU_STA_COUNT];
    wire<1> fill_ack; // MSHR响应：填充完成
};

struct WBMSHRIO {
    wire<1> ready;
};

struct MSHRWBIO{
    wire<1> valid;
    wire<32> addr;
    wire<32> data[DCACHE_LINE_WORDS];
};

struct BypassReq {
    wire<1> valid;
    wire<32> addr;

    BypassReq() : valid(false), addr(0) {}
};

struct BypassResp {
    wire<1> valid;
    wire<32> data;

    BypassResp() : valid(false), data(0) {}
};

struct MergeReq {
    wire<1> valid;
    wire<32> addr;
    wire<32> data;
    wire<8> strb;

    MergeReq() : valid(false), addr(0), data(0), strb(0) {}
};

struct MergeResp {
    wire<1> valid;
    wire<1> busy;

    MergeResp() : valid(false), busy(false) {}
};

struct DcacheWBIO{
    BypassReq bypass_req[LSU_LDU_COUNT];
    MergeReq merge_req[LSU_STA_COUNT];
};

struct WBDcacheIO{
    BypassResp bypass_resp[LSU_LDU_COUNT];
    MergeResp merge_resp[LSU_STA_COUNT];
};

struct DcacheCoherenInIO {
    wire<32>addr;
};
struct DcacheCoherenOutIO {
    wire<32> data;
    wire<2> result; // 0: miss, 1: retry, 2: hit
};

struct DcacheINIO {
    LsuDcacheIO  *lsu2dcache  = nullptr;  // LSU → DCache requests
    MSHRDcacheIO *mshr2dcache = nullptr;  // MSHR fill/free → DCache
    WBDcacheIO   *wb2dcache   = nullptr;  // WB bypass/merge resp → DCache
    #if CONFIG_BSD
    BSDIN *bsd_in[LSU_LDU_COUNT + LSU_STA_COUNT]; // For BSD: output the chosen victim for debugging
    DcacheCoherenInIO *coherence_in; // For coherence queries from DCache to coherence module
     // For tracking pending misses for the store queue and load queue
    #endif
};

struct DcacheOUTIO {
    DcacheLsuIO  *dcache2lsu  = nullptr;  // DCache → LSU responses
    DcacheMSHRIO *dcache2mshr = nullptr;  // DCache miss alloc → MSHR
    DcacheWBIO   *dcache2wb   = nullptr;  // DCache bypass/merge req → WB
    #if CONFIG_BSD
    BSDOUT *bsd_out[LSU_LDU_COUNT + LSU_STA_COUNT] ; // For BSD: output the chosen victim for debugging
    PendingWrite *pendingwrite[LSU_LDU_COUNT + LSU_STA_COUNT]; // For tracking pending misses for the store queue and load queue
    LruUpdate *lru_updates[LSU_LDU_COUNT + LSU_STA_COUNT]; // For tracking PLRU updates for the store queue and load queue
    MSHRWrite *mshr_write; // For tracking MSHR fill responses for the store queue and load queue
    DcacheCoherenOutIO *coherence_out; // For coherence query responses
    #endif
};

// ─────────────────────────────────────────────────────────────────────────────
// Address field decomposition helper
// ─────────────────────────────────────────────────────────────────────────────
struct AddrFields {
    wire<DCACHE_TAG_BITS> tag;
    wire<DCACHE_SET_BITS> set_idx;
    wire<DCACHE_OFFSET_BITS> word_off; // which 32-bit word within the cacheline [4:2]
    uint32_t bank;     // set_idx & (DCACHE_BANKS - 1)
};

struct MSHREntry {
    wire<1> valid;
    wire<1> issued;
    wire<1> fill;
    wire<DCACHE_SET_BITS> index;
    wire<DCACHE_TAG_BITS> tag;
    wire<1> merged_store_dirty;
    wire<32> merged_store_data[DCACHE_LINE_WORDS];
    uint8_t merged_store_strb[DCACHE_LINE_WORDS];
};

struct WriteBufferEntry {
    reg<1> send;
    reg<32> addr;
    reg<32> data[DCACHE_LINE_WORDS];
};

extern MSHREntry mshr_entries[DCACHE_MSHR_ENTRIES];
extern WriteBufferEntry write_buffer[DCACHE_WB_ENTRIES];

void init_dcache();
AddrFields decode(uint32_t addr);
bool find_mshr_entry(uint32_t index, uint32_t tag);
uint32_t get_addr(uint32_t set_idx, uint32_t tag, uint32_t word_off);
int choose_lru_victim(uint32_t set_idx); 
void lru_reset(uint32_t set_idx, uint32_t way);

void write_dcache_line(uint32_t set_idx, uint32_t way,uint32_t tag, uint32_t data[DCACHE_LINE_WORDS]);
void apply_strobe(uint32_t &dst, uint32_t src, uint8_t strb);

bool cache_line_match(uint32_t addr1, uint32_t addr2);
