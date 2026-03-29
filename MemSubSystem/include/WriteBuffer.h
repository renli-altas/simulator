#pragma once

#include "DcacheConfig.h"
#include "IO.h"
#include <cstdint>

struct WBState {
    reg<1> send;  // in-flight write exists (accepted, waiting B response)
    // Issue-hold valid: we have selected/snapshotted a head payload and keep
    // driving it until req_accepted.
    reg<1> issue_pending;
    reg<32> issue_addr;
    reg<32> issue_data[DCACHE_LINE_WORDS];
    reg<1> pending_push_valid;
    WriteBufferEntry pending_push_entry;

    reg<32> bypassdata[LSU_LDU_COUNT];
    reg<1> bypassvalid[LSU_LDU_COUNT];

    reg<1> mergevalid[LSU_STA_COUNT];
    reg<1> mergebusy[LSU_STA_COUNT];
};

static constexpr int WB_FIND_PORTS = LSU_STA_COUNT + LSU_LDU_COUNT;

struct FIFOIN {
    wire<1> writebuffer_head_valid = false;
    WriteBufferEntry writebuffer_head = {};

    wire<1> writebuffer_find_valid[WB_FIND_PORTS] = {};
    wire<DCACHE_WB_BITS> writebuffer_find_index[WB_FIND_PORTS] = {};
    WriteBufferEntry writebuffer_find_entry[WB_FIND_PORTS] = {};

    wire<1> writebuffer_full = false;
};

struct FIFOOUT {
    wire<1> pop = false;

    wire<1> push_valid = false;
    WriteBufferEntry push_entry = {};

    wire<1> writebuffer_update_head = false;
    WriteBufferEntry writebuffer_update_head_entry = {};

    wire<1> writebuffer_find_req[WB_FIND_PORTS] = {};
    wire<32> writebuffer_find_addr[WB_FIND_PORTS] = {};

    wire<1> writebuffer_update_valid[WB_FIND_PORTS] = {};
    wire<DCACHE_WB_BITS> writebuffer_update_index[WB_FIND_PORTS] = {};
    WriteBufferEntry writebuffer_update_entry[WB_FIND_PORTS] = {};
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
    wire<32>    req_addr       = 0;
    wire<8>     req_total_size = 0;
    wire<8>     req_id         = 0;
    wire<64>    req_wstrb      = 0;
    wire<32>    req_wdata[DCACHE_LINE_WORDS] = {};
    wire<1>     resp_ready     = false;  // ready to accept B response
};


struct WBIn {
    MSHRWBIO mshrwb;
    DcacheWBIO dcachewb;
    WbAxiIn axi_in;
    FIFOIN fifo_in;

    void clear() {
        *this = {};
    }
};
struct WBOut {
    WBMSHRIO wbmshr;
    WBDcacheIO wbdcache;
    WbAxiOut axi_out;
    FIFOOUT fifo_out;

    void clear() {
        *this = {};
    }
};
// ─────────────────────────────────────────────────────────────────────────────
// WriteBuffer — queues dirty evictions and drains them over the AXI write port.
//
// Correct usage per cycle:
//   1. wb_.comb_outputs()   ← sets out.ready from FIFO status
//   2. stage2_comb()        ← reads out.full/free_count, sets in.push_ports[]
//   3. Bridge IC outputs → wb_.axi_in
//   4. wb_.comb_inputs()    ← processes pushes + B channel + fills axi_out
//   5. Bridge wb_.axi_out → IC inputs; interconnect.comb_inputs()
//   6. wb_.seq()            ← cur = nxt, reset nxt
// ─────────────────────────────────────────────────────────────────────────────
class WriteBuffer {
public:
    WriteBuffer() = default;
    int find_wb_entry(uint32_t addr);

    void init();

    // Phase 1: compute output readiness from current FIFO status.
    void comb_mshr_outputs();
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
};
