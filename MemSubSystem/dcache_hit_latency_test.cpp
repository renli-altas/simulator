#include "RealDcache.h"

#include <cstdio>
#include <cstdlib>

long long sim_time = 0;
uint32_t *p_memory = nullptr;

bool pmem_init() { return true; }
void pmem_release() {}
void pmem_clear_all() {}
bool pmem_is_ram_addr(uint32_t, uint32_t) { return false; }
uint32_t pmem_read(uint32_t) { return 0; }
void pmem_write(uint32_t, uint32_t) {}
void pmem_memcpy_to_ram(uint32_t, const void *, size_t) {}
void pmem_memcpy_from_ram(void *, uint32_t, size_t) {}
uint32_t *pmem_ram_ptr() { return nullptr; }

namespace {

[[noreturn]] void fail(const char *msg) {
    std::fprintf(stderr, "dcache_hit_latency_test: %s\n", msg);
    std::exit(1);
}

void run_comb_cycle(RealDcache &dcache, MSHR &mshr, WriteBuffer &wb) {
    wb.in.axi_in = {};
    mshr.in.axi_in = {};

    wb.comb_outputs();
    mshr.in.wbmshr = wb.out.wbmshr;
    mshr.comb_outputs();
    dcache.stage1_comb();
    dcache.prepare_wb_queries_for_stage2();

    wb.in.mshrwb = mshr.out.mshrwb;
    wb.comb_inputs();
    wb.comb_outputs();
    mshr.in.wbmshr = wb.out.wbmshr;

    dcache.stage2_comb();
}

} // namespace

int main() {
    SimContext ctx;
    RealDcache dcache;
    MSHR mshr;
    WriteBuffer wb;

    LsuDcacheIO lsu2dcache{};
    DcacheLsuIO dcache2lsu{};

    dcache.lsu2dcache = &lsu2dcache;
    dcache.dcache2lsu = &dcache2lsu;
    dcache.mshr2dcache = &mshr.out.mshr2dcache;
    dcache.dcache2mshr = &mshr.in.dcachemshr;
    dcache.wb2dcache = &wb.out.wbdcache;
    dcache.dcache2wb = &wb.in.dcachewb;
    dcache.bind_context(&ctx);
    mshr.bind_context(&ctx);
    wb.bind_context(&ctx);

    mshr.init();
    wb.init();
    dcache.init();

    const uint32_t addr = 0x80001040u;
    const AddrFields f = decode(addr);
    uint32_t line_data[DCACHE_LINE_WORDS] = {};
    for (int i = 0; i < DCACHE_LINE_WORDS; ++i) {
        line_data[i] = 0x1000u + static_cast<uint32_t>(i);
    }
    write_dcache_line(f.set_idx, 0, f.tag, line_data);

    lsu2dcache.req_ports.load_ports[0].valid = true;
    lsu2dcache.req_ports.load_ports[0].addr = addr;
    lsu2dcache.req_ports.load_ports[0].req_id = 7;
    lsu2dcache.req_ports.load_ports[0].uop.rob_idx = 7;
    lsu2dcache.req_ports.load_ports[0].uop.dest_preg = 3;
    lsu2dcache.req_ports.load_ports[0].uop.rob_flag = 0;
    lsu2dcache.req_ports.load_ports[0].uop.op = UOP_LOAD;

    run_comb_cycle(dcache, mshr, wb);

    const LoadResp &load_hit = dcache2lsu.resp_ports.load_resps[0];
    if (!load_hit.valid) {
        fail("load hit did not respond in the request cycle");
    }
    if (load_hit.replay != 0) {
        fail("load hit unexpectedly replayed");
    }
    if (static_cast<uint32_t>(load_hit.data) != line_data[f.word_off]) {
        fail("load hit returned wrong data");
    }
    const uint32_t first_load_data = static_cast<uint32_t>(load_hit.data);

    dcache.seq();
    mshr.seq();
    wb.seq();
    ++sim_time;

    lsu2dcache = {};
    dcache2lsu = {};

    const uint32_t store_data = 0xdeadbeefu;
    lsu2dcache.req_ports.store_ports[0].valid = true;
    lsu2dcache.req_ports.store_ports[0].addr = addr;
    lsu2dcache.req_ports.store_ports[0].data = store_data;
    lsu2dcache.req_ports.store_ports[0].strb = 0xFu;
    lsu2dcache.req_ports.store_ports[0].req_id = 9;
    lsu2dcache.req_ports.store_ports[0].uop.rob_idx = 9;
    lsu2dcache.req_ports.store_ports[0].uop.rob_flag = 0;

    run_comb_cycle(dcache, mshr, wb);

    const StoreResp &store_hit = dcache2lsu.resp_ports.store_resps[0];
    if (!store_hit.valid) {
        fail("store hit did not ack in the request cycle");
    }
    if (store_hit.replay != 0) {
        fail("store hit unexpectedly replayed");
    }

    dcache.seq();
    mshr.seq();
    wb.seq();
    ++sim_time;

    lsu2dcache = {};
    dcache2lsu = {};
    lsu2dcache.req_ports.load_ports[0].valid = true;
    lsu2dcache.req_ports.load_ports[0].addr = addr;
    lsu2dcache.req_ports.load_ports[0].req_id = 10;
    lsu2dcache.req_ports.load_ports[0].uop.rob_idx = 10;
    lsu2dcache.req_ports.load_ports[0].uop.dest_preg = 4;
    lsu2dcache.req_ports.load_ports[0].uop.rob_flag = 0;
    lsu2dcache.req_ports.load_ports[0].uop.op = UOP_LOAD;

    run_comb_cycle(dcache, mshr, wb);

    const LoadResp &load_after_store = dcache2lsu.resp_ports.load_resps[0];
    if (!load_after_store.valid) {
        fail("load after committed store hit did not respond in-cycle");
    }
    if (load_after_store.replay != 0) {
        fail("load after store unexpectedly replayed");
    }
    if (static_cast<uint32_t>(load_after_store.data) != store_data) {
        fail("load after store did not observe committed store-hit data");
    }
    const uint32_t post_store_load_data =
        static_cast<uint32_t>(load_after_store.data);

    if (ctx.perf.l1d_load_hit_return_samples != 2) {
        fail("unexpected load-hit latency sample count");
    }
    if (ctx.perf.l1d_store_hit_return_samples != 1) {
        fail("unexpected store-hit latency sample count");
    }
    if (ctx.perf.l1d_load_hit_return_total_cycles != 0) {
        fail("same-cycle load hits should accumulate zero-cycle return latency");
    }
    if (ctx.perf.l1d_store_hit_return_total_cycles != 0) {
        fail("same-cycle store hits should accumulate zero-cycle return latency");
    }

    std::printf("same_cycle_load_hit_data=0x%08x\n",
                first_load_data);
    std::printf("same_cycle_store_hit_ack=1\n");
    std::printf("post_store_load_hit_data=0x%08x\n",
                post_store_load_data);
    std::printf("load_hit_return_samples=%llu total_cycles=%llu\n",
                static_cast<unsigned long long>(
                    ctx.perf.l1d_load_hit_return_samples),
                static_cast<unsigned long long>(
                    ctx.perf.l1d_load_hit_return_total_cycles));
    std::printf("store_hit_return_samples=%llu total_cycles=%llu\n",
                static_cast<unsigned long long>(
                    ctx.perf.l1d_store_hit_return_samples),
                static_cast<unsigned long long>(
                    ctx.perf.l1d_store_hit_return_total_cycles));
    std::printf("dcache_hit_latency_test: PASS\n");
    return 0;
}
