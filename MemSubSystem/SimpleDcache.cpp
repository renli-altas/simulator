#include "SimpleDcache.h"
#include "DcacheConfig.h"
#include "PhysMemory.h"
#include "types.h"
#include <cassert>

namespace {

inline uint32_t word_aligned_addr(uint32_t addr) { return addr & ~0x3u; }

inline uint32_t line_aligned_addr(uint32_t addr) {
  return addr & ~(static_cast<uint32_t>(DCACHE_LINE_BYTES) - 1u);
}

int find_hit_way(uint32_t addr) {
  const AddrFields f = decode(addr);
  for (int w = 0; w < DCACHE_WAYS; w++) {
    if (valid_array[f.set_idx][w] && tag_array[f.set_idx][w] == f.tag) {
      return w;
    }
  }
  return -1;
}

void fill_cache_line_from_pmemory(uint32_t addr) {
  const AddrFields f = decode(addr);
  uint32_t line_data[DCACHE_LINE_WORDS] = {};
  const uint32_t base = line_aligned_addr(addr);
  for (int i = 0; i < DCACHE_LINE_WORDS; i++) {
    line_data[i] = pmem_read(base + static_cast<uint32_t>(i) * 4u);
  }
  const int victim = choose_lru_victim(f.set_idx);
  write_dcache_line(f.set_idx, static_cast<uint32_t>(victim), f.tag, line_data);
}

} // namespace

void SimpleDcache::init() {
  init_dcache();
  cycle_ = 0;
  pending_loads_.clear();
  pending_stores_.clear();
}

SimpleDcache::CoherentQueryResult
SimpleDcache::query_coherent_word(uint32_t addr, uint32_t &data) const {
  (void)addr;
  data = 0;
  return CoherentQueryResult::Miss;
}

void SimpleDcache::comb() {
  assert(lsu2dcache != nullptr && "lsu2dcache pointer not set");
  assert(dcache2lsu != nullptr && "dcache2lsu pointer not set");

  dcache2lsu->resp_ports.clear();

  for (int i = 0; i < LSU_LDU_COUNT; i++) {
    const auto &req = lsu2dcache->req_ports.load_ports[i];
    if (!req.valid) {
      continue;
    }
    PendingLoad p{};
    const bool hit = (find_hit_way(req.addr) >= 0);
    p.ready_cycle = cycle_ + (hit ? kHitLatency : kMissLatency);
    p.addr = req.addr;
    p.uop = req.uop;
    p.req_id = req.req_id;
    p.was_miss = !hit;
    pending_loads_.push_back(p);
    if (ctx != nullptr) {
      ctx->perf.dcache_access_num++;
      if (!hit) {
        ctx->perf.dcache_miss_num++;
      }
    }
  }

  for (int i = 0; i < LSU_STA_COUNT; i++) {
    const auto &req = lsu2dcache->req_ports.store_ports[i];
    if (!req.valid) {
      continue;
    }
    PendingStore p{};
    const bool hit = (find_hit_way(req.addr) >= 0);
    p.ready_cycle = cycle_ + (hit ? kHitLatency : kMissLatency);
    p.addr = req.addr;
    p.data = req.data;
    p.strb = static_cast<uint8_t>(req.strb);
    p.req_id = req.req_id;
    p.was_miss = !hit;
    pending_stores_.push_back(p);
    if (ctx != nullptr) {
      ctx->perf.dcache_access_num++;
      if (!hit) {
        ctx->perf.dcache_miss_num++;
      }
    }
  }

  for (int i = 0; i < LSU_STA_COUNT; i++) {
    PendingStore head{};
    bool found_ready = false;
    for (auto it = pending_stores_.begin(); it != pending_stores_.end(); ++it) {
      if (it->ready_cycle > cycle_) {
        continue;
      }
      head = *it;
      pending_stores_.erase(it);
      found_ready = true;
      break;
    }
    if (!found_ready) {
      break;
    }
    int way = find_hit_way(head.addr);
    if (way < 0) {
      fill_cache_line_from_pmemory(head.addr);
      way = find_hit_way(head.addr);
    }
    const AddrFields f = decode(head.addr);
    if (way >= 0) {
      apply_strobe(data_array[f.set_idx][way][f.word_off], head.data, head.strb);
      dirty_array[f.set_idx][way] = false; // write-through model
      lru_reset(f.set_idx, static_cast<uint32_t>(way));
    }

    const uint32_t paddr_word = word_aligned_addr(head.addr);
    uint32_t mem_word = pmem_read(paddr_word);
    apply_strobe(mem_word, head.data, head.strb);
    pmem_write(paddr_word, mem_word);

    auto &resp = dcache2lsu->resp_ports.store_resps[i];
    resp = {};
    resp.valid = true;
    resp.replay = 0;
    resp.req_id = head.req_id;
    resp.is_cache_miss = head.was_miss;
  }

  for (int i = 0; i < LSU_LDU_COUNT; i++) {
    PendingLoad head{};
    bool found_ready = false;
    for (auto it = pending_loads_.begin(); it != pending_loads_.end(); ++it) {
      if (it->ready_cycle > cycle_) {
        continue;
      }
      head = *it;
      pending_loads_.erase(it);
      found_ready = true;
      break;
    }
    if (!found_ready) {
      break;
    }
    int way = find_hit_way(head.addr);
    if (way < 0) {
      fill_cache_line_from_pmemory(head.addr);
      way = find_hit_way(head.addr);
    }
    uint32_t data = pmem_read(word_aligned_addr(head.addr));
    if (way >= 0) {
      const AddrFields f = decode(head.addr);
      data = data_array[f.set_idx][way][f.word_off];
      lru_reset(f.set_idx, static_cast<uint32_t>(way));
    }

    auto &resp = dcache2lsu->resp_ports.load_resps[i];
    resp = {};
    resp.valid = true;
    resp.replay = 0;
    resp.data = data;
    resp.uop = head.uop;
    resp.req_id = head.req_id;
  }
}

void SimpleDcache::seq() { cycle_++; }
