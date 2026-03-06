#include "MemSubsystem.h"

// ─────────────────────────────────────────────────────────────────────────────
// PTW port adapters (unchanged from previous version)
// ─────────────────────────────────────────────────────────────────────────────
class MemSubsystemPtwMemPortAdapter : public PtwMemPort {
public:
  MemSubsystemPtwMemPortAdapter(MemSubsystem *owner, MemSubsystem::PtwClient c)
      : owner(owner), client(c) {}

  bool send_read_req(uint32_t paddr) override {
    return owner->ptw_mem_send_read_req(client, paddr);
  }
  bool resp_valid() const override { return owner->ptw_mem_resp_valid(client); }
  uint32_t resp_data() const override { return owner->ptw_mem_resp_data(client); }
  void consume_resp() override { owner->ptw_mem_consume_resp(client); }

private:
  MemSubsystem *owner = nullptr;
  MemSubsystem::PtwClient client = MemSubsystem::PtwClient::DTLB;
};

class MemSubsystemPtwWalkPortAdapter : public PtwWalkPort {
public:
  MemSubsystemPtwWalkPortAdapter(MemSubsystem *owner, MemSubsystem::PtwClient c)
      : owner(owner), client(c) {}

  bool send_walk_req(const PtwWalkReq &req) override {
    return owner->ptw_walk_send_req(client, req);
  }
  bool resp_valid() const override { return owner->ptw_walk_resp_valid(client); }
  PtwWalkResp resp() const override { return owner->ptw_walk_resp(client); }
  void consume_resp() override { owner->ptw_walk_consume_resp(client); }
  void flush_client() override { owner->ptw_walk_flush(client); }

private:
  MemSubsystem *owner = nullptr;
  MemSubsystem::PtwClient client = MemSubsystem::PtwClient::DTLB;
};

// ─────────────────────────────────────────────────────────────────────────────
// PTW helper implementations
// ─────────────────────────────────────────────────────────────────────────────
MemPtwBlock::Client MemSubsystem::to_block_client(PtwClient c) {
  return (c == MemSubsystem::PtwClient::DTLB) ? MemPtwBlock::Client::DTLB
                                               : MemPtwBlock::Client::ITLB;
}

void MemSubsystem::refresh_ptw_client_outputs() {
  for (size_t i = 0; i < kPtwClientCount; i++) {
    PtwClient client = static_cast<PtwClient>(i);
    ptw_mem_resp_ios[i].valid =
        ptw_block.client_resp_valid(to_block_client(client));
    ptw_mem_resp_ios[i].data =
        ptw_block.client_resp_data(to_block_client(client));
    ptw_walk_resp_ios[i].valid =
        ptw_block.walk_client_resp_valid(to_block_client(client));
    ptw_walk_resp_ios[i].resp =
        ptw_block.walk_client_resp(to_block_client(client));
  }
}

bool MemSubsystem::ptw_mem_send_read_req(PtwClient client, uint32_t paddr) {
  bool fire = ptw_block.client_send_read_req(to_block_client(client), paddr);
  refresh_ptw_client_outputs();
  return fire;
}

bool MemSubsystem::ptw_mem_resp_valid(PtwClient client) const {
  return ptw_mem_resp_ios[ptw_client_idx(client)].valid;
}

uint32_t MemSubsystem::ptw_mem_resp_data(PtwClient client) const {
  return ptw_mem_resp_ios[ptw_client_idx(client)].data;
}

void MemSubsystem::ptw_mem_consume_resp(PtwClient client) {
  ptw_block.client_consume_resp(to_block_client(client));
  refresh_ptw_client_outputs();
}

bool MemSubsystem::ptw_walk_send_req(PtwClient client, const PtwWalkReq &req) {
  bool fire = ptw_block.walk_client_send_req(to_block_client(client), req);
  refresh_ptw_client_outputs();
  return fire;
}

bool MemSubsystem::ptw_walk_resp_valid(PtwClient client) const {
  return ptw_walk_resp_ios[ptw_client_idx(client)].valid;
}

PtwWalkResp MemSubsystem::ptw_walk_resp(PtwClient client) const {
  return ptw_walk_resp_ios[ptw_client_idx(client)].resp;
}

void MemSubsystem::ptw_walk_consume_resp(PtwClient client) {
  ptw_block.walk_client_consume_resp(to_block_client(client));
  refresh_ptw_client_outputs();
}

void MemSubsystem::ptw_walk_flush(PtwClient client) {
  ptw_block.walk_client_flush(to_block_client(client));
  refresh_ptw_client_outputs();
}

// ─────────────────────────────────────────────────────────────────────────────
// Constructor / Destructor
// ─────────────────────────────────────────────────────────────────────────────
MemSubsystem::MemSubsystem(SimContext *ctx) : ctx(ctx) {
  ptw_block.bind_context(ctx);
  dtlb_ptw_port_inst =
      std::make_unique<MemSubsystemPtwMemPortAdapter>(this, PtwClient::DTLB);
  itlb_ptw_port_inst =
      std::make_unique<MemSubsystemPtwMemPortAdapter>(this, PtwClient::ITLB);
  dtlb_walk_port_inst =
      std::make_unique<MemSubsystemPtwWalkPortAdapter>(this, PtwClient::DTLB);
  itlb_walk_port_inst =
      std::make_unique<MemSubsystemPtwWalkPortAdapter>(this, PtwClient::ITLB);
  dtlb_ptw_port  = dtlb_ptw_port_inst.get();
  itlb_ptw_port  = itlb_ptw_port_inst.get();
  dtlb_walk_port = dtlb_walk_port_inst.get();
  itlb_walk_port = itlb_walk_port_inst.get();
}

MemSubsystem::~MemSubsystem() = default;

// ─────────────────────────────────────────────────────────────────────────────
// init — connect wires and initialise all sub-modules.
// ─────────────────────────────────────────────────────────────────────────────
void MemSubsystem::init() {
  Assert(lsu2dcache != nullptr && "MemSubsystem: lsu2dcache is not connected");
  Assert(dcache2lsu  != nullptr && "MemSubsystem: dcache2lsu is not connected");
  Assert(csr    != nullptr && "MemSubsystem: csr is not connected");
  Assert(memory != nullptr && "MemSubsystem: memory is not connected");

  // ── PeripheralModel ────────────────────────────────────────────────────────
  peripheral.csr    = csr;
  peripheral.memory = memory;
  peripheral.init();

  // ── Wire RealDcache IO ports ───────────────────────────────────────────────
  // External LSU interface (pointers owned by caller; stable after init).
  dcache_.lsu2dcache  = lsu2dcache;
  dcache_.dcache2lsu  = dcache2lsu;

  // Internal MSHR ↔ DCache wires: RealDcache reads/writes MSHR IO structs
  // directly via pointers, keeping the connection zero-copy.
  dcache_.mshr2dcache = &mshr_.out.mshr2dcache;  // MSHR output → DCache input
  dcache_.dcache2mshr = &mshr_.in.dcachemshr;    // DCache output → MSHR input

  // Internal WriteBuffer ↔ DCache wires.
  dcache_.wb2dcache   = &wb_.out.wbdcache;       // WB output → DCache input
  dcache_.dcache2wb   = &wb_.in.dcachewb;        // DCache output → WB input

  // ── Initialise sub-modules ─────────────────────────────────────────────────
  mshr_.init();
  wb_.init();
  dcache_.init();

  ptw_block.init();
  ptw_mem_resp_ios  = {};
  ptw_walk_resp_ios = {};
  refresh_ptw_client_outputs();

  mshr_axi_in  = {};
  mshr_axi_out = {};
  wb_axi_in    = {};
  wb_axi_out   = {};
}

// ─────────────────────────────────────────────────────────────────────────────
// on_commit_store — side-effects when a store retires from the ROB.
// ─────────────────────────────────────────────────────────────────────────────
void MemSubsystem::on_commit_store(uint32_t paddr, uint32_t data,
                                   uint8_t func3) {
  peripheral.on_commit_store(paddr, data, func3);
}

// ─────────────────────────────────────────────────────────────────────────────
// comb — combinational evaluation for one cycle.
//
// Ordering:
//   1. PTW: resolve pending page-table reads directly from memory[].
//   2. MSHR Phase-1  : comb_outputs()  — fill / free count from cur state.
//   3. WB   Phase-1  : comb_outputs()  — full flag / bypass results.
//   4. RealDcache    : comb()          — reads MSHR/WB outputs, writes inputs.
//   5. Bridge AXI + WB→MSHR ready signal; run mshr_.comb_inputs().
//   6. Bridge MSHR eviction → WB + AXI write channel; run wb_.comb_inputs().
//   7. Export updated AXI outputs for the caller.
// ─────────────────────────────────────────────────────────────────────────────
void MemSubsystem::comb() {
  // ── PTW ───────────────────────────────────────────────────────────────────
  ptw_block.comb_select_walk_owner();
  ptw_block.count_wait_cycles();

  // Resolve pending single-address PTW mem reads directly from memory[].
  // (PTW bypasses RealDcache to keep the interface simple; page-table data
  //  is always coherent in memory[] because stores go through on_commit_store.)
  for (int i = 0; i < static_cast<int>(MemPtwBlock::Client::NUM_CLIENTS); i++) {
    auto client = static_cast<MemPtwBlock::Client>(i);
    if (ptw_block.has_pending_mem_req(client)) {
      uint32_t paddr = ptw_block.pending_mem_addr(client);
      uint32_t data  = memory[paddr >> 2];
      ptw_block.on_mem_read_granted(client);
      ptw_block.on_mem_resp_client(client, data);
    }
  }

  // Resolve PTW walk reads (L1 → optionally L2) directly from memory[].
  // The loop completes at most two iterations (one per page-table level).
  {
    uint32_t walk_addr = 0;
    while (ptw_block.walk_read_req(walk_addr)) {
      uint32_t data = memory[walk_addr >> 2];
      ptw_block.on_walk_read_granted();
      ptw_block.on_walk_mem_resp(data);
    }
  }

  refresh_ptw_client_outputs();

  // ── RealDcache + MSHR + WriteBuffer ──────────────────────────────────────
  // Phase 1: compute stable outputs from cur state so DCache can read them.
  mshr_.comb_outputs();
  wb_.comb_outputs();

  // Phase 2: RealDcache pipeline (S1 + S2).
  //   Reads : mshr_.out.mshr2dcache, wb_.out.wbdcache  (via dcache_ pointers)
  //   Writes: mshr_.in.dcachemshr,   wb_.in.dcachewb   (via dcache_ pointers)
  dcache_.comb();

  // Phase 3a: inject AXI read-channel inputs and WB→MSHR ready signal, then
  //           run MSHR comb_inputs (may generate a fill → dcache and an
  //           eviction push → WriteBuffer).
  mshr_.in.axi_in  = mshr_axi_in;
  mshr_.in.wbmshr  = wb_.out.wbmshr;   // WB ready flag (set in Phase 1)
  mshr_.comb_inputs();

  // Phase 3b: inject AXI write-channel inputs and MSHR eviction, then run
  //           WriteBuffer comb_inputs (drains evictions onto AXI).
  wb_.in.axi_in   = wb_axi_in;
  wb_.in.mshrwb   = mshr_.out.mshrwb;  // eviction push (set in Phase 3a)
  wb_.comb_inputs();

  // Phase 4: expose updated AXI outputs to the caller.
  mshr_axi_out = mshr_.out.axi_out;
  wb_axi_out   = wb_.out.axi_out;
}

// ─────────────────────────────────────────────────────────────────────────────
// seq — advance all state on the simulated clock edge.
// ─────────────────────────────────────────────────────────────────────────────
void MemSubsystem::seq() {
  dcache_.seq();
  mshr_.seq();
  wb_.seq();
}
