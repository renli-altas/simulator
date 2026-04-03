#pragma once

#include "AbstractDcache.h"
#include "IO.h"
#include <cstddef>
#include <cstdint>
#include <deque>

class SimContext;

class SimpleDcache : public AbstractDcache {
public:
  enum class CoherentQueryResult : uint8_t {
    Miss = 0,
    Retry = 1,
    Hit = 2,
  };

  void bind_context(SimContext *c) { ctx = c; }
  void init() override;
  void comb() override;
  void seq() override;
  void dump_debug_state() const;
  CoherentQueryResult query_coherent_word(uint32_t addr, uint32_t &data) const;

  // IO ports - keep only LSU <-> DCache interfaces.
  LsuDcacheIO *lsu2dcache = nullptr;
  DcacheLsuIO *dcache2lsu = nullptr;

private:
  static constexpr uint64_t kHitLatency =
      (SIMPLE_DCACHE_HIT_LATENCY > 0)
          ? static_cast<uint64_t>(SIMPLE_DCACHE_HIT_LATENCY)
          : 1ull;
  static constexpr uint64_t kMissLatency =
      (SIMPLE_DCACHE_MISS_LATENCY > 0)
          ? static_cast<uint64_t>(SIMPLE_DCACHE_MISS_LATENCY)
          : 1ull;

  struct PendingLoad {
    uint64_t ready_cycle = 0;
    uint32_t addr = 0;
    MicroOp uop = {};
    size_t req_id = 0;
    bool was_miss = false;
    uint8_t replay = 0;
    uint8_t resp_detail = 0;
  };

  struct PendingStore {
    uint64_t ready_cycle = 0;
    uint32_t addr = 0;
    uint32_t data = 0;
    uint8_t strb = 0;
    StqEntry uop = {};
    size_t req_id = 0;
    bool was_miss = false;
    uint8_t replay = 0;
    uint8_t resp_detail = 0;
  };

  uint64_t cycle_ = 0;
  std::deque<PendingLoad> pending_loads_{};
  std::deque<PendingStore> pending_stores_{};
  SimContext *ctx = nullptr;
};
