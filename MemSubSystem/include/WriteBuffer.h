#pragma once

#include "DcacheConfig.h"
#include "IO.h"
#include <cstdint>

class SimContext;

extern WriteBufferEntry write_buffer_nxt[DCACHE_WB_ENTRIES];
bool write_buffer_entry_live(uint32_t idx);
bool write_buffer_lookup_word(uint32_t addr, uint32_t &data);

struct WBState {
    wire<DCACHE_WB_BITS> count; // number of valid entries in the buffer
    wire<DCACHE_WB_BITS> head;  // index of the oldest entry (next to evict)
    wire<1> send;  // flag to indicate if a request is currently being sent
    wire<1> issue_pending; // saw req_ready hint, waiting real acceptance

    wire<32> bypassdata[LSU_LDU_COUNT];
    wire<1> bypassvalid[LSU_LDU_COUNT];

    wire<1> mergevalid[LSU_STA_COUNT];
    wire<1> mergebusy[LSU_STA_COUNT];
};

// AXI write-channel interface signals (IC's write_ports[MASTER_DCACHE_W]).
// axi_in  — inputs from IC to WriteBuffer (driven by RealDcache bridge).
// axi_out — outputs from WriteBuffer to IC (consumed by RealDcache bridge).
struct WbAxiIn {
    wire<1> req_ready  = false;  // IC is ready to accept the current request
    wire<1> req_accepted = false; // one-cycle pulse when IC actually accepts it
    wire<1> resp_valid = false;  // B response available
};

struct WbAxiOut {
    wire<1>     req_valid      = false;  // AW+W request to IC
    wire<32> req_addr       = 0;
    wire<8>  req_total_size = 0;
    wire<8>  req_id         = 0;
    wire<64> req_wstrb      = 0;
    wire<32> req_wdata[DCACHE_LINE_WORDS] = {};
    wire<1>     resp_ready     = false;  // ready to accept B response
};


struct WBIn {
    MSHRWBIO mshrwb;
    DcacheWBIO dcachewb;
    WbAxiIn axi_in;

    void clear() {
        *this = {};
    }
};
struct WBOut {
    WBMSHRIO wbmshr;
    WBDcacheIO wbdcache;
    WbAxiOut axi_out;

    void clear() {
        *this = {};
    }
};
// ─────────────────────────────────────────────────────────────────────────────
// WriteBuffer — queues dirty evictions and drains them over the AXI write port.
//
// Correct usage per cycle:
//   1. wb_.comb_outputs()   ← sets out.full / out.free_count (from nxt.count)
//   2. stage2_comb()        ← reads out.full/free_count, sets in.push_ports[]
//   3. Bridge IC outputs → wb_.axi_in
//   4. wb_.comb_inputs()    ← processes pushes + B channel + fills axi_out
//   5. Bridge wb_.axi_out → IC inputs; interconnect.comb_inputs()
//   6. wb_.seq()            ← cur = nxt, reset nxt
// ─────────────────────────────────────────────────────────────────────────────
class WriteBuffer {
public:
    WriteBuffer() = default;
    void bind_context(SimContext *c) { ctx = c; }
    int find_wb_entry(uint32_t addr);

    void init();

    // Phase 1: compute out.full and out.free_count from current nxt.count.
    void comb_outputs();

    // Phase 2: process push requests from in.push_ports[], accept B channel
    // responses, and fill axi_out with the next AW+W request.
    // Reads axi_in (set by caller before invoking); writes axi_out.
    void comb_inputs();

    void seq();

    // Input / output signal ports (public for direct access by RealDcache).
    WBIn  in;
    WBOut out;

    WBState cur, nxt;
    SimContext *ctx = nullptr;
};
