#include "RealLsu.h"
#include "DeadlockDebug.h"
#include "DcacheConfig.h"
#include "PhysMemory.h"
#include "TlbMmu.h"
#include "config.h"
#include "util.h"
#include <chrono>
#include <cstdio>
#include <cstdint>
#include <memory>
static constexpr int64_t REQ_WAIT_RETRY = 0x7FFFFFFFFFFFFFFF;
static constexpr int64_t REQ_WAIT_SEND = 0x7FFFFFFFFFFFFFFD;
static constexpr int64_t REQ_WAIT_RESP = 0x7FFFFFFFFFFFFFFE;
static constexpr int64_t REQ_WAIT_EXEC = 0x7FFFFFFFFFFFFFFC;
static constexpr uint64_t LD_RESP_STUCK_RETRY_CYCLES = 150;
static constexpr uint64_t LD_KILLED_GC_CYCLES = 2;
static inline bool is_amo_lr_uop(const MicroOp &uop) {
  return ((uop.dbg.instruction & 0x7Fu) == 0x2Fu) &&
         ((uop.func7 >> 2) == AmoOp::LR);
}
namespace {
using LsuClock = std::chrono::steady_clock;

struct ScopedPerfNs {
  uint64_t *total_ns = nullptr;
  LsuClock::time_point start = LsuClock::now();

  explicit ScopedPerfNs(uint64_t *dst) : total_ns(dst) {}

  ~ScopedPerfNs() {
    if (total_ns == nullptr) {
      return;
    }
    *total_ns += static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            LsuClock::now() - start)
            .count());
  }
};

constexpr size_t kStqTokenFlagShift = STQ_IDX_WIDTH;
constexpr uint32_t kDebugLoadPc = 0xc03679e0;
constexpr uint32_t kDebugLoadInst = 0x02c12083;
constexpr uint32_t kDebugLoadPaddr = 0xffffffffu;
constexpr uint32_t kDebugStorePaddr = 0x80833ddc;
constexpr uint32_t kDebugStoreData = 0xc0369acc;

inline size_t encode_stq_token(uint32_t stq_idx, uint32_t stq_flag) {
  return static_cast<size_t>(stq_idx) |
         (static_cast<size_t>(stq_flag) << kStqTokenFlagShift);
}

inline uint32_t decode_stq_token_idx(size_t token) {
  return static_cast<uint32_t>(token & ((1u << STQ_IDX_WIDTH) - 1u));
}

inline uint32_t decode_stq_token_flag(size_t token) {
  return static_cast<uint32_t>((token >> kStqTokenFlagShift) & 0x1u);
}

inline std::pair<uint32_t, uint32_t>
advance_stq_slot(uint32_t stq_idx, uint32_t stq_flag, std::size_t delta) {
  const std::size_t raw = static_cast<std::size_t>(stq_idx) + delta;
  const uint32_t next_idx = static_cast<uint32_t>(raw % STQ_SIZE);
  const uint32_t next_flag =
      static_cast<uint32_t>(stq_flag ^ ((raw / STQ_SIZE) & 0x1u));
  return {next_idx, next_flag};
}

inline bool is_debug_load_uop(const MicroOp &uop) {
  return uop.dbg.pc == kDebugLoadPc;
}

inline bool is_debug_store_data(uint32_t data) {
  return data == kDebugStoreData;
}

inline bool is_debug_store_addr(uint32_t paddr) {
  return paddr == kDebugStorePaddr;
}

inline bool commit_store_is_older_than_load(const StqCommitEntry &entry,
                                            const MicroOp &load_uop) {
  MicroOp store_uop{};
  store_uop.rob_idx = entry.rob_idx;
  store_uop.rob_flag = entry.rob_flag;
  return cmp_inst_age(load_uop, store_uop);
}

template <typename QueueT>
inline auto *find_done_stq_entry_by_token(QueueT &queue, size_t token,
                                          PerfCount *perf = nullptr) {
  if (perf != nullptr) {
    perf->lsu_find_done_token_calls++;
  }
  const uint32_t stq_idx = decode_stq_token_idx(token);
  const uint32_t stq_flag = decode_stq_token_flag(token);
  for (std::size_t slot = 0; slot < STQ_SIZE; ++slot) {
    if (perf != nullptr) {
      perf->lsu_find_done_token_entries_scanned++;
    }
    if (!queue.is_valid(slot)) {
      continue;
    }
    auto &entry = queue.slot(slot);
    if (entry.stq_idx == stq_idx && entry.stq_flag == stq_flag) {
      return &entry;
    }
  }
  return static_cast<typename std::remove_reference_t<decltype(queue.slot(0))> *>(nullptr);
}

template <typename QueueT>
inline bool queue_contains_stq_token(const QueueT &queue, uint32_t stq_idx,
                                     uint32_t stq_flag,
                                     PerfCount *perf = nullptr) {
  if (perf != nullptr) {
    perf->lsu_store_alloc_token_check_calls++;
  }
  for (std::size_t slot = 0; slot < STQ_SIZE; ++slot) {
    if (perf != nullptr) {
      perf->lsu_store_alloc_token_check_entries_scanned++;
    }
    if (!queue.is_valid(slot)) {
      continue;
    }
    const auto &entry = queue.slot(slot);
    if (entry.stq_idx == stq_idx && entry.stq_flag == stq_flag) {
      return true;
    }
  }
  return false;
}

RealLsu *g_deadlock_lsu_instance = nullptr;

void dump_lsu_deadlock_state() {
  if (g_deadlock_lsu_instance != nullptr) {
    g_deadlock_lsu_instance->dump_debug_state();
  }
}

template <typename EntryT>
inline bool stq_entry_matches_uop(const EntryT &entry, const MicroOp &uop) {
  return entry.valid && entry.rob_idx == uop.rob_idx &&
         entry.rob_flag == uop.rob_flag;
}

template <typename QueueT, typename ValueT>
inline void fifo_push_or_assert(QueueT &queue, ValueT &&value,
                                const char *queue_name) {
  if (!queue.push(std::forward<ValueT>(value))) {
    LSU_MEM_DBG_PRINTF("[LSU FIFO OVERFLOW] cyc=%lld queue=%s\n",
                       (long long)sim_time, queue_name);
    Assert(false && "RealLsu FIFO overflow");
  }
}

template <typename T, std::size_t Capacity, typename Pred>
inline void fifo_erase_if(FIFO<T, Capacity> &queue, Pred pred,
                          const char *queue_name) {
  const std::size_t original_size = queue.size();
  for (std::size_t i = 0; i < original_size; ++i) {
    T item = std::move(queue.front());
    queue.pop();
    if (!pred(item)) {
      fifo_push_or_assert(queue, std::move(item), queue_name);
    }
  }
}

template <typename T, std::size_t Capacity, typename Pred>
inline void fifo_flag_erase_if(FIFO_FLAG<T, Capacity> &queue, Pred pred,
                               const char *queue_name) {
  const std::size_t original_size = queue.size();
  for (std::size_t i = 0; i < original_size; ++i) {
    T item = std::move(queue.front());
    queue.pop();
    if (!pred(item)) {
      fifo_push_or_assert(queue, std::move(item), queue_name);
    }
  }
}

int get_mem_width(int func3) {
  switch (func3 & 0b11) {
  case 0b00:
    return 1;
  case 0b01:
    return 2;
  case 0b10:
    return 4;
  default:
    return 4;
  }
}
uint32_t extract_data(uint32_t raw_mem_val, uint32_t addr, int func3) {
  int bit_offset = (addr & 0x3) * 8;
  uint32_t result = 0;
  uint32_t shifted = raw_mem_val >> bit_offset;

  switch (func3) {
  case 0b000:
    result = shifted & 0xFF;
    if (result & 0x80)
      result |= 0xFFFFFF00;
    break;
  case 0b001:
    result = shifted & 0xFFFF;
    if (result & 0x8000)
      result |= 0xFFFF0000;
    break;
  case 0b010:
    result = shifted;
    break;
  case 0b100:
    result = shifted & 0xFF;
    break;
  case 0b101:
    result = shifted & 0xFFFF;
    break;
  default:
    result = shifted;
    break;
  }
  return result;
}
uint32_t merge_data_to_word(uint32_t old_word, uint32_t new_data,
                            uint32_t addr, int func3) {
  int bit_offset = (addr & 0x3) * 8;
  uint32_t mask = 0;

  switch (func3 & 0b11) {
  case 0b00:
    mask = 0xFF;
    break;
  case 0b01:
    mask = 0xFFFF;
    break;
  default:
    mask = 0xFFFFFFFF;
    break;
  }

  uint32_t clear_mask = ~(mask << bit_offset);
  uint32_t result = old_word & clear_mask;
  result |= ((new_data & mask) << bit_offset);
  return result;
}
} // namespace

RealLsu::RealLsu(SimContext *ctx) : ctx(ctx) {
  g_deadlock_lsu_instance = this;
  deadlock_debug::register_lsu_dump_cb(dump_lsu_deadlock_state);
  // Initialize MMU
#ifdef CONFIG_TLB_MMU
  mmu = std::make_unique<TlbMmu>(ctx, nullptr, nullptr, DTLB_ENTRIES);
#else
  mmu = std::make_unique<SimpleMmu>(ctx, this);
#endif
  init();
}

void RealLsu::init() {
  cur = LSUState{};
  nxt = LSUState{};
  lookup_ = LookupTables{};
  mmu->flush();
  // memset(&out, 0, sizeof(LsuOut));
}

void RealLsu::clear_commit_lookup() {
  for (int flag = 0; flag < 2; ++flag) {
    for (int idx = 0; idx < ROB_NUM; ++idx) {
      lookup_.commit_lookup_valid[flag][idx] = false;
      lookup_.commit_lookup_slot[flag][idx] = 0;
    }
  }
}

void RealLsu::clear_done_lookup() {
  for (int flag = 0; flag < 2; ++flag) {
    for (int idx = 0; idx < ROB_NUM; ++idx) {
      lookup_.done_lookup_valid[flag][idx] = false;
      lookup_.done_lookup_slot[flag][idx] = 0;
    }
    for (int idx = 0; idx < STQ_SIZE; ++idx) {
      lookup_.done_token_lookup_valid[flag][idx] = false;
      lookup_.done_token_lookup_slot[flag][idx] = 0;
    }
  }
}

void RealLsu::rebuild_commit_lookup() {
  clear_commit_lookup();
  for (std::size_t slot = 0; slot < STQ_SIZE; ++slot) {
    if (!nxt.commit_stq.is_valid(slot)) {
      continue;
    }
    const auto &entry = nxt.commit_stq.slot(slot);
    const int rob_flag = static_cast<int>(entry.rob_flag);
    const int rob_idx = static_cast<int>(entry.rob_idx);
    lookup_.commit_lookup_valid[rob_flag][rob_idx] = true;
    lookup_.commit_lookup_slot[rob_flag][rob_idx] =
        static_cast<wire<STQ_IDX_WIDTH>>(slot);
  }
}

std::pair<uint32_t, uint32_t>
RealLsu::next_dispatch_stq_token(const LSUState &state) const {
  const std::size_t done_count = state.done_stq.size();
  const std::size_t commit_count = state.commit_stq.size();
  const std::size_t live_count = done_count + commit_count;

  uint32_t base_idx = static_cast<uint32_t>(state.done_stq.get_tail());
  uint32_t base_flag = static_cast<uint32_t>(state.done_stq.get_tail_flag());
  if (!state.done_stq.empty()) {
    const auto &head = state.done_stq.front();
    base_idx = head.stq_idx;
    base_flag = head.stq_flag;
  } else if (!state.commit_stq.empty()) {
    const auto &head = state.commit_stq.front();
    base_idx = head.stq_idx;
    base_flag = head.stq_flag;
  }
  return advance_stq_slot(base_idx, base_flag, live_count);
}

// =========================================================
// 1. Dispatch 阶段: STQ 分配反馈
// =========================================================

wire<LDQ_IDX_WIDTH> RealLsu::LDQ_Count() {
  wire<LDQ_IDX_WIDTH> count = 0;
  for (int i = 0; i < LDQ_SIZE; i++) {
    if (cur.ldq[i].valid) {
      count++;
    }
  }
  return count;
}

void RealLsu::comb_lsu2dis_info() {
  ScopedPerfNs scoped(ctx ? &ctx->perf.lsu_host_time_comb_lsu2dis_ns : nullptr);
  Assert(out.lsu2dis != nullptr && "RealLsu::comb_lsu2dis_info: lsu2dis is null");
  *out.lsu2dis = {};

  // Global STQ order is:
  //   done_stq [head, commit) + commit_stq [commit, tail)
  // Tokens must not be reused until both queues have retired them. Derive the
  // dispatch-visible tail from the oldest live token plus total live
  // occupancy, instead of using commit_stq.tail() alone.
  const std::size_t done_count = cur.done_stq.size();
  const std::size_t commit_count = cur.commit_stq.size();
  const std::size_t live_count = done_count + commit_count;
  Assert(live_count <= STQ_SIZE &&
         "RealLsu: combined STQ occupancy overflow / token reuse detected");

  const auto [tail_idx, tail_flag] = next_dispatch_stq_token(cur);
  out.lsu2dis->stq_tail = tail_idx;
  out.lsu2dis->stq_tail_flag = tail_flag;

  // Leave one full commit row of visible headroom. Without this slack, oracle
  // mode can overfill store-side speculation, which hurts Dhrystone IPC even
  // though the total STQ capacity is technically not exhausted.
  const int visible_stq_free_raw =
      STQ_SIZE - static_cast<int>(live_count) - COMMIT_WIDTH;
  out.lsu2dis->stq_free = visible_stq_free_raw > 0 ? visible_stq_free_raw : 0;
  out.lsu2dis->ldq_free = LDQ_SIZE - LDQ_Count();

  int produced = 0;
  for (int scan_pos = 0; scan_pos < LDQ_SIZE && produced < MAX_LDQ_DISPATCH_WIDTH;
      scan_pos++) {
    if (!cur.ldq[scan_pos].valid) {
      out.lsu2dis->ldq_alloc_idx[produced] = scan_pos;
      out.lsu2dis->ldq_alloc_valid[produced] = true;
      produced++;
    }
  }


  // SFENCE.VMA should wait only for older stores that have already committed
  // from ROB but have not drained from the LSU yet. Younger speculative or
  // not-yet-committed stores in commit_stq must not block ROB head progress,
  // otherwise SFENCE and those younger stores can deadlock each other.
  out.lsu2rob->committed_store_pending = !cur.done_stq.empty();

#ifndef CONFIG_BSD
  // Populate miss_mask (Phase 4)
  uint64_t mask = 0;
  for (int i = 0; i < LDQ_SIZE; i++ ) {
    const auto &entry = cur.ldq[i];
    if (cur.ldq[i].valid && !entry.killed && entry.uop.tma.is_cache_miss) {
      mask |= (1ULL << entry.uop.rob_idx);
    }
  }
  out.lsu2rob->tma.miss_mask = mask;
#endif
}
StqDoneEntry RealLsu::get_done_stq_entry(int idx, uint32_t rob_idx,
                                         uint32_t rob_flag) const {
  if (ctx != nullptr) {
    ctx->perf.lsu_get_done_entry_calls++;
  }
  if (lookup_.done_lookup_valid[rob_flag][rob_idx]) {
    if (ctx != nullptr) {
      ctx->perf.lsu_get_done_entry_entries_scanned++;
    }
    const std::size_t slot = lookup_.done_lookup_slot[rob_flag][rob_idx];
    if (nxt.done_stq.is_valid(slot)) {
      const auto &entry = nxt.done_stq.slot(slot);
      if (entry.stq_idx == idx && entry.rob_idx == rob_idx &&
          entry.rob_flag == rob_flag) {
        return entry;
      }
    }
  }

  // Commit/difftest queries can arrive in Rename::comb_fire() before this
  // cycle's LSU comb_pipeline() promotes the just-committed store from
  // commit_stq into done_stq. Fall back to ROB identity so same-cycle
  // sideband lookup does not depend on commit_stq's physical slot numbering.
  if (const auto *src = find_commit_stq_entry_by_rob(rob_idx, rob_flag)) {
    StqDoneEntry dst{};
    dst.done = false;
    dst.inflight = false;
    dst.is_mmio = src->is_mmio;
    dst.suppress_write = src->suppress_write;
    dst.replay_priority = 0;
    dst.p_addr = src->p_addr;
    dst.data = src->data;
    dst.func3 = src->func3;
    dst.rob_idx = src->rob_idx;
    dst.rob_flag = src->rob_flag;
    dst.stq_idx = src->stq_idx;
    dst.stq_flag = src->stq_flag;
    return dst;
  }

  return StqDoneEntry{};
}
// =========================================================
// 2. Execute 阶段: 接收 AGU/SDU 请求 (多端口轮询)
// =========================================================
void RealLsu::comb_recv() {
  ScopedPerfNs scoped(ctx ? &ctx->perf.lsu_host_time_comb_recv_ns : nullptr);
  // 顶层当前采用直接变量赋值连线；这里每拍将端口硬连到 MMU。
  Assert(out.lsu2dcache != nullptr && "RealLsu::comb_recv: lsu2dcache is null");
  Assert(out.peripheral_req != nullptr &&
         "RealLsu::comb_recv: peripheral_req is null");
  *out.lsu2dcache = {};
  *out.peripheral_req = {};

  PeripheralReqIO mmio_req = {};
  bool mmio_req_used = false;
  const PeripheralRespIO peripheral_resp = *in.peripheral_resp ;

  if (cur.pending_mmio_valid) {
    if (peripheral_resp.ready) {
        *out.peripheral_req = cur.pending_mmio_req;
      // MMIO bridge is ready: hand off exactly once, then let inflight
      // tracking rely on LDQ waiting_resp / STQ inflight state instead of
      // re-driving the same pending request forever.
      nxt.pending_mmio_valid = false;
      nxt.pending_mmio_req = {};
    } else {
      // Keep the request pending until the bridge becomes ready.
    }
    mmio_req_used = true;
  } else if (!peripheral_resp.ready) {
    // Bridge is still busy with a previously accepted MMIO transaction.
    mmio_req_used = true;
  }

  // Retry STA address translations that previously returned MMU::RETRY.
  progress_pending_sta_addr();

  // 1. 优先级：Store Data (来自 SDU)
  // 确保在消费者检查之前数据就绪
  for (int i = 0; i < LSU_SDU_COUNT; i++) {
    if (in.exe2lsu->sdu_req[i].valid) {
      handle_store_data(in.exe2lsu->sdu_req[i].uop.to_micro_op());
    }
  }

  // 2. 优先级：Store Addr (来自 AGU)
  // 确保地址对于别名检查有效
  for (int i = 0; i < LSU_AGU_COUNT; i++) {
    if (in.exe2lsu->agu_req[i].valid) {
      const auto &uop = in.exe2lsu->agu_req[i].uop;
      if (uop.rob_idx == 133 && uop.rob_flag == 0) {
        TEMP_BUG_TRACE_PRINTF(
            "[LSU TRACE] recv agu rob=%u/%u port=%d op=%d result=0x%08x stq=%u/%u cyc=%lld\n",
            (unsigned)uop.rob_idx, (unsigned)uop.rob_flag, i, (int)uop.op,
            (uint32_t)uop.result, (unsigned)uop.stq_idx,
            (unsigned)uop.stq_flag, (long long)sim_time);
      }
      if (uop.op == UOP_STA) {
        handle_store_addr(uop.to_micro_op());
      }
    }
  }

  // 3. 优先级：Loads (来自 AGU)
  // 最后处理 Load，使其能看到本周期最新的 Store (STLF)
  for (int i = 0; i < LSU_AGU_COUNT; i++) {
    if (in.exe2lsu->agu_req[i].valid) {
      const auto &uop = in.exe2lsu->agu_req[i].uop;
      if (uop.op == UOP_LOAD) {
        handle_load_req(uop.to_micro_op());
      }
    }
  }

  for (int i = 0; i < LSU_LDU_COUNT; i++) {
    out.lsu2dcache->req_ports.load_ports[i].valid = false;
  }
  for (int i = 0; i < LSU_STA_COUNT; i++) {
    out.lsu2dcache->req_ports.store_ports[i].valid = false;
  }

  // Lost-response recovery:
  // 1) non-killed load waits too long -> retry
  // 2) killed load waits too long -> drop the slot to avoid LDQ leak/deadlock
  for (int i = 0; i < LDQ_SIZE; i++) {
    auto &entry = nxt.ldq[i];
    if (!entry.valid || !entry.sent || !entry.waiting_resp) {
      continue;
    }
    if (entry.uop.cplt_time != REQ_WAIT_RESP) {
      continue;
    }
    if (sim_time < 0) {
      continue;
    }

    const uint64_t sim_time_u64 = static_cast<uint64_t>(sim_time);
    if (sim_time_u64 < entry.wait_resp_since) {
      continue;
    }
    const uint64_t wait_cycles = sim_time_u64 - entry.wait_resp_since;

    if (entry.killed) {
      if (wait_cycles >= LD_KILLED_GC_CYCLES) {
        entry.sent = false;
        entry.waiting_resp = false;
        entry.wait_resp_since = 0;
        entry.uop.cplt_time = sim_time;
        entry.replay_priority = 0;
      }
      continue;
    }

    if (entry.is_mmio_wait || is_mmio_addr(entry.uop.diag_val)) {
      continue;
    }

    if (wait_cycles >= LD_RESP_STUCK_RETRY_CYCLES) {
      entry.sent = false;
      entry.waiting_resp = false;
      entry.wait_resp_since = 0;
      entry.uop.cplt_time = REQ_WAIT_SEND;
      entry.replay_priority = 3;
    }
  }


  // Rebuild MSHR replay pressure from queue state every cycle.
  // This avoids stale counters after flush/mispredict/recovery paths.
  int mshr_replay_count_ldq = 0;
  int mshr_replay_count_stq = 0;
  for (int i = 0; i < LDQ_SIZE; i++) {
    const auto &e = nxt.ldq[i];
    if (!e.valid || e.killed || e.sent || e.waiting_resp) {
      continue;
    }
    if (e.replay_priority == 1) {
      mshr_replay_count_ldq++;
    }
  }
  for (int i = 0; i < nxt.done_stq.size(); i++) {
    const auto &e = nxt.done_stq[i];
    if ( e.done || e.inflight) {
      continue;
    }
    if (e.replay_priority == 1) {
      mshr_replay_count_stq++;
    }
  }

  if (mshr_replay_count_stq > REPLAY_STORE_COUNT_UPPER_BOUND &&
      cur.replay_type == 0) {
    nxt.replay_type = 1;
  } else if (mshr_replay_count_stq < REPLAY_STORE_COUNT_LOWER_BOUND &&
             cur.replay_type == 1) {
    nxt.replay_type = 0;
  }

  if (mshr_replay_count_ldq == 0)
    nxt.replay_type = 1;

  bool has_replay = false;
  auto has_mmio_inflight = [&]() {
    if (nxt.pending_mmio_valid) {
      return true;
    }
    for (int idx = 0; idx < LDQ_SIZE; idx++) {
      const auto &e = nxt.ldq[idx];
      if (e.valid && e.waiting_resp && !e.killed && !e.is_mmio_wait &&
          is_mmio_addr(e.uop.diag_val)) {
        return true;
      }
    }
    for (int idx = 0; idx < nxt.done_stq.size(); idx++) {
      const auto &e = nxt.done_stq[idx];
      if (e.is_mmio && e.inflight && !e.done) {
        return true;
      }
    }
    return false;
  };

  const bool fill_wakeup = in.dcache2lsu->resp_ports.replay_resp.replay;
  const bool mshr_has_free =
      (in.dcache2lsu->resp_ports.replay_resp.free_slots > 0);

  if (fill_wakeup || mshr_has_free) {
    for (int i = 0; i < LDQ_SIZE; i++) {
      auto &entry = nxt.ldq[i];
      if (!entry.valid || entry.killed || entry.sent || entry.waiting_resp) {
        continue;
      }
      if (fill_wakeup && entry.replay_priority == 2 &&
          cache_line_match(entry.uop.diag_val,
                           in.dcache2lsu->resp_ports.replay_resp.replay_addr)) {
        entry.replay_priority = 5;
      }
      if (mshr_has_free && entry.replay_priority == 1 && nxt.replay_type == 0 &&
          !has_replay) {
        entry.replay_priority = 4;
        has_replay = true;
      }
    }
    for (int i = 0; i < nxt.done_stq.size(); i++) {
      auto &entry = nxt.done_stq[i];
      if ( entry.done || entry.inflight) {
        continue;
      }
      if (fill_wakeup && entry.replay_priority == 2 &&
          cache_line_match(entry.p_addr,
                           in.dcache2lsu->resp_ports.replay_resp.replay_addr)) {
        entry.replay_priority = 0;
      }
      if (mshr_has_free && entry.replay_priority == 1 && nxt.replay_type == 1 &&
          !has_replay) {
        entry.replay_priority = 0;
        has_replay = true;
      }
    }
  }

  for (int i = 0; i < LSU_LDU_COUNT; i++) {
    int max_idx = -1;
    int max_priority = -1;
    int best_age = ROB_NUM + 1;
    for (int j = 0; j < LDQ_SIZE; j++) {
      auto &entry = nxt.ldq[j];
      if (!entry.valid || entry.killed || entry.sent || entry.waiting_resp) {
        continue;
      }
      // Only issue loads whose address translation / forwarding stage has
      // finished and explicitly marked them ready to send.
      if (entry.uop.cplt_time != REQ_WAIT_SEND) {
        continue;
      }
      // replay=1(mshr_full) and replay=2(mshr_hit) both wait for explicit
      // wakeup from MSHR fill events.
      if (entry.replay_priority == 1 || entry.replay_priority == 2) {
        continue;
      }
      // MMIO load 必须等到成为 ROB 当前最老的未提交指令后才发送，
      // 并且这条 load 之前的 store 已经从 STQ 排空，这样前面更老指令的
      // 提交副作用（尤其是 MMIO store）一定已经生效。
      if (entry.is_mmio_wait) {
        const bool mmio_can_issue =
            !mmio_req_used && !has_mmio_inflight() && in.rob_bcast->head_valid &&
            !has_older_store_pending(entry.uop) &&
            entry.uop.rob_idx == (uint32_t)in.rob_bcast->head_rob_idx &&
            peripheral_resp.ready;
        if (!mmio_can_issue) {
          continue;
        }
        // MMIO response path expects uop.rob_idx to carry an LDQ-local
        // token, not architectural ROB index.
        MicroOp mmio_uop = entry.uop;
        mmio_uop.rob_idx = j;
        mmio_req.is_mmio = 1;
        mmio_req.wen = 0; // Load 没有写使能
        mmio_req.mmio_addr = entry.uop.diag_val;
        mmio_req.mmio_wdata = 0; // Load 没有写数据
        mmio_req.uop = mmio_uop;
        mmio_req_used = true;
        nxt.pending_mmio_valid = true;
        nxt.pending_mmio_req = mmio_req;
        entry.is_mmio_wait = false; // 已发出请求，重置等待标志
        entry.sent = true;
        entry.waiting_resp = true;
        entry.wait_resp_since = sim_time;
        entry.uop.cplt_time = REQ_WAIT_RESP;
        break;
      }
      int rob_age = 0;
      if (in.rob_bcast->head_valid) {
        rob_age = (static_cast<int>(entry.uop.rob_idx) -
                   static_cast<int>(in.rob_bcast->head_rob_idx) + ROB_NUM) %
                  ROB_NUM;
      }
      if (entry.replay_priority > max_priority ||
          (entry.replay_priority == max_priority && rob_age < best_age)) {
        max_priority = entry.replay_priority;
        max_idx = j;
        best_age = rob_age;
      }
    }
    if (max_idx != -1) {
      MicroOp req_uop = nxt.ldq[max_idx].uop;
      req_uop.rob_idx = max_idx; // Local token: LDQ index
      if (is_debug_load_uop(nxt.ldq[max_idx].uop) ||
          nxt.ldq[max_idx].uop.diag_val == kDebugLoadPaddr) {
        TEMP_BUG_TRACE_PRINTF(
            "[LSU LOAD DBG][send] pc=0x%08x inst=0x%08x ldq=%d paddr=0x%08x replay=%u cyc=%lld\n",
            (uint32_t)nxt.ldq[max_idx].uop.dbg.pc,
            (uint32_t)nxt.ldq[max_idx].uop.dbg.instruction, max_idx,
            (uint32_t)nxt.ldq[max_idx].uop.diag_val,
            (unsigned)nxt.ldq[max_idx].replay_priority,
            (long long)sim_time);
      }
      out.lsu2dcache->req_ports.load_ports[i].valid = true;
      out.lsu2dcache->req_ports.load_ports[i].addr = nxt.ldq[max_idx].uop.diag_val;
      out.lsu2dcache->req_ports.load_ports[i].req_id = max_idx;
      out.lsu2dcache->req_ports.load_ports[i].uop = req_uop;
      nxt.ldq[max_idx].sent = true;
      nxt.ldq[max_idx].waiting_resp = true;
      nxt.ldq[max_idx].wait_resp_since = sim_time;
      nxt.ldq[max_idx].uop.cplt_time = REQ_WAIT_RESP;
      if (nxt.ldq[max_idx].replay_priority >= 4) {
        // replay_priority=4: replay=1(mshr_full) wakeup by free-slot.
        // replay_priority=5: replay=2(mshr_hit) wakeup by fill-match.
        nxt.ldq[max_idx].replay_priority = 0;
      }
    }
  }

  // Per cycle, each STA port can issue at most one real store request.
  // Scan the wait_done queue from head and pick the oldest issuable entries.
  int issued_sta = 0;
  for (int i = 0; i < nxt.done_stq.size() && issued_sta < LSU_STA_COUNT; i++) {
    auto &entry = nxt.done_stq[i];

    if (entry.suppress_write) {
      continue;
    }
    if (entry.done || entry.inflight || entry.replay_priority) {
      continue;
    }
    bool continue_flag = false;
    for (int j = 0; j < i; j++) {
      auto &older_entry = nxt.done_stq[j];
      if (older_entry.done || older_entry.suppress_write) {
        continue;
      }
      // Preserve program order for same-address stores until the older
      // store is fully acknowledged. Otherwise a bank-conflict replay can
      // let an older store reissue after a younger one and overwrite the
      // newer value.
      if (older_entry.p_addr == entry.p_addr) {
        continue_flag = true;
        break;
      }
    }
    if (continue_flag) {
      continue;
    }
    if (entry.is_mmio) {
      if (mmio_req_used) {
        continue;
      }
      if (has_mmio_inflight()) {
        continue;
      }
      // MMIO store only needs STQ ordering. Once it is the oldest
      // committed/ready store reachable from stq_head, it can issue
      // even if the ROB head has already advanced past it.
      if (!peripheral_resp.ready) {
        continue;
      }
      mmio_req.is_mmio = 1;
      mmio_req.wen = 1; // Store 有写使能
      mmio_req.mmio_addr = entry.p_addr;
      mmio_req.mmio_wdata = entry.data;
      mmio_req.uop = {};
      mmio_req.uop.op = UOP_STA;
      // MMIO response path uses uop.rob_idx as STQ slot token.
      mmio_req.uop.stq_idx = entry.stq_idx;
      mmio_req.uop.stq_flag = entry.stq_flag;
      mmio_req.uop.func3 = entry.func3;
      mmio_req_used = true;
      nxt.pending_mmio_valid = true;
      nxt.pending_mmio_req = mmio_req;
      entry.inflight = true;
      issued_sta++;
      continue;
    }
    change_store_info(entry, issued_sta,
                      static_cast<int>(encode_stq_token(entry.stq_idx,
                                                        entry.stq_flag)));
    entry.inflight = true; // Track request until resp/replay returns.
    issued_sta++;
  }

  // MMIO request is sent only in the cycle-begin pending path above.
  // Do not re-drive here, otherwise a newly enqueued pending request can be
  // sent twice (once at end of this cycle, once again next cycle).
}

// =========================================================
// 3. Writeback 阶段: 输出 Load 结果 (多端口写回)
// =========================================================
void RealLsu::comb_load_res() {
  ScopedPerfNs scoped(ctx ? &ctx->perf.lsu_host_time_comb_load_res_ns : nullptr);
  // 1. 先清空所有写回端口
  Assert(out.lsu2exe != nullptr && "RealLsu::comb_load_res: lsu2exe is null");
  *out.lsu2exe = {};

  for (int i = 0; i < LSU_LDU_COUNT; i++) {
    if (in.dcache2lsu->resp_ports.load_resps[i].valid) {
      uint32_t idx =
          static_cast<uint32_t>(in.dcache2lsu->resp_ports.load_resps[i].req_id);
      if (idx >= 0 && idx < LDQ_SIZE) {
        auto &entry = nxt.ldq[idx];
        if (entry.valid && entry.sent && entry.waiting_resp) {
          const auto &resp_uop = in.dcache2lsu->resp_ports.load_resps[i].uop;
          if (is_debug_load_uop(entry.uop) || entry.uop.diag_val == kDebugLoadPaddr ||
              resp_uop.dbg.pc == kDebugLoadPc ||
              resp_uop.dbg.instruction == kDebugLoadInst) {
            TEMP_BUG_TRACE_PRINTF(
                "[LSU LOAD DBG][resp] pc=0x%08x inst=0x%08x ldq=%u replay=%u raw=0x%08x paddr=0x%08x cyc=%lld\n",
                (uint32_t)entry.uop.dbg.pc,
                (uint32_t)entry.uop.dbg.instruction, (unsigned)idx,
                (unsigned)in.dcache2lsu->resp_ports.load_resps[i].replay,
                (uint32_t)in.dcache2lsu->resp_ports.load_resps[i].data,
                (uint32_t)entry.uop.diag_val, (long long)sim_time);
          }
          
          if (!entry.killed) {
            if (in.dcache2lsu->resp_ports.load_resps[i].replay == 0) {
              uint32_t raw_data = in.dcache2lsu->resp_ports.load_resps[i].data;
              uint32_t extracted =
                  extract_data(raw_data, entry.uop.diag_val, entry.uop.func3);
              if (is_amo_lr_uop(entry.uop)) {
                nxt.reserve_addr = entry.uop.diag_val;
                nxt.reserve_valid = true;
              }
              entry.uop.result = extracted;
            #ifndef CONFIG_BSD
              entry.uop.dbg.difftest_skip =
                  in.dcache2lsu->resp_ports.load_resps[i].uop.dbg.difftest_skip;
              entry.uop.cplt_time = sim_time;
              entry.uop.tma.is_cache_miss =
                  !in.dcache2lsu->resp_ports.load_resps[i]
                       .uop.tma.is_cache_miss;
            #endif
              entry.replay_priority = 0;
              fifo_push_or_assert(nxt.finished_loads, entry.uop,
                                  "finished_loads");
              entry.valid = false;
            } else {
              // Handle load replay if needed (e.g., due to MSHR eviction)
              entry.replay_priority =
                  in.dcache2lsu->resp_ports.load_resps[i].replay;
              // replay=1(resource full) waits for a free-slot wakeup.
              // replay=2(mshr_hit) waits for matching line fill wakeup.
              entry.sent = false;
              entry.waiting_resp = false;
              entry.wait_resp_since = 0;
              entry.uop.cplt_time = REQ_WAIT_SEND;
            }
          } else {
            entry.valid = false;
          }
        }
      } else {
      #ifndef CONFIG_BSD
        Assert(false && "Invalid LDQ index in load response");
      #endif
      }
    }
  }
  const PeripheralRespIO &peripheral_resp = *in.peripheral_resp;

  if (peripheral_resp.is_mmio && peripheral_resp.uop.op == UOP_LOAD) {
    int idx = peripheral_resp.uop.rob_idx;
    if (idx >= 0 && idx < LDQ_SIZE) {
      auto &entry = nxt.ldq[idx];
      if (entry.valid && entry.sent && entry.waiting_resp) {
        if (!entry.killed) {
          entry.uop.result = peripheral_resp.mmio_rdata;
          entry.uop.dbg.difftest_skip = peripheral_resp.uop.dbg.difftest_skip;
          entry.uop.cplt_time = sim_time;
          entry.uop.tma.is_cache_miss = false; // MMIO 访问不算 Cache Miss
          fifo_push_or_assert(nxt.finished_loads, entry.uop, "finished_loads");
        }
      }
      entry.valid = false;
    } else {
    #ifndef CONFIG_BSD
      Assert(false && "Invalid LDQ index in MMIO load response");
    #endif
    }
  }

  for (int i = 0; i < LSU_STA_COUNT; i++) {
    if (in.dcache2lsu->resp_ports.store_resps[i].valid) {
      const size_t token = in.dcache2lsu->resp_ports.store_resps[i].req_id;
      const uint32_t stq_idx = decode_stq_token_idx(token);
      const uint32_t stq_flag = decode_stq_token_flag(token);
      if (ctx != nullptr) {
        ctx->perf.lsu_find_done_token_calls++;
      }
      if (lookup_.done_token_lookup_valid[stq_flag][stq_idx]) {
        if (ctx != nullptr) {
          ctx->perf.lsu_find_done_token_entries_scanned++;
        }
        auto &entry =
            nxt.done_stq.slot(lookup_.done_token_lookup_slot[stq_flag][stq_idx]);
        if (is_debug_store_addr(entry.p_addr) || is_debug_store_data(entry.data)) {
          TEMP_BUG_TRACE_PRINTF(
              "[LSU STORE DBG][resp] rob=%u/%u stq=%u/%u replay=%u done=%d inflight=%d paddr=0x%08x data=0x%08x cyc=%lld\n",
              (unsigned)entry.rob_idx, (unsigned)entry.rob_flag,
              (unsigned)entry.stq_idx, (unsigned)entry.stq_flag,
              (unsigned)in.dcache2lsu->resp_ports.store_resps[i].replay,
              (int)entry.done, (int)entry.inflight,
              (uint32_t)entry.p_addr, (uint32_t)entry.data,
              (long long)sim_time);
        }
        if (!entry.done && entry.inflight) {
          if (in.dcache2lsu->resp_ports.store_resps[i].replay == 0) {
            entry.done = true;
            entry.replay_priority = 0;
            entry.inflight = false;
          } else {
            // replay=3 is bank-conflict: it should be retried directly
            // on the next cycle and must not freeze the STQ head.
            entry.replay_priority =
                (in.dcache2lsu->resp_ports.store_resps[i].replay == 3)
                    ? 0
                    : in.dcache2lsu->resp_ports.store_resps[i].replay;
            entry.inflight = false; // 等待下次重新发送
          }
        }
      } else {
      #ifndef CONFIG_BSD
        Assert(false && "Invalid STQ index in store response");
      #endif
      }
    }
  }

  if (peripheral_resp.is_mmio && peripheral_resp.uop.op == UOP_STA) {
    const uint32_t stq_idx = peripheral_resp.uop.stq_idx;
    const uint32_t stq_flag = peripheral_resp.uop.stq_flag;
    const bool matched = lookup_.done_token_lookup_valid[stq_flag][stq_idx];
    if (matched) {
      auto &entry =
          nxt.done_stq.slot(lookup_.done_token_lookup_slot[stq_flag][stq_idx]);
      if (!entry.done && entry.inflight) {
        entry.done = true;
        entry.inflight = false;
      }
    }
    if (!matched) {
    #ifndef CONFIG_BSD
      Assert(false && "Invalid STQ index in MMIO store response");
    #endif
    }
  }

  // 2. 从完成队列填充端口 (Load)
  for (int i = 0; i < LSU_LOAD_WB_WIDTH; i++) {
    if (!nxt.finished_loads.empty()) {
      out.lsu2exe->wb_req[i].valid = true;
      out.lsu2exe->wb_req[i].uop =
          LsuExeIO::LsuExeRespUop::from_micro_op(nxt.finished_loads.front());
      nxt.finished_loads.pop();
    } else {
      break;
    }
  }

  // 3. 从完成队列填充端口 (STA)
  for (int i = 0; i < LSU_STA_COUNT; i++) {
    if (!nxt.finished_sta_reqs.empty()) {
      out.lsu2exe->sta_wb_req[i].valid = true;
      out.lsu2exe->sta_wb_req[i].uop =
          LsuExeIO::LsuExeRespUop::from_micro_op(nxt.finished_sta_reqs.front());
      nxt.finished_sta_reqs.pop();
    } else {
      out.lsu2exe->sta_wb_req[i].valid = false;
    }
  }
}

void RealLsu::handle_load_req(const MicroOp &inst) {
  int ldq_idx = inst.ldq_idx;
  Assert(ldq_idx >= 0 && ldq_idx < LDQ_SIZE);
  if (!nxt.ldq[ldq_idx].valid || nxt.ldq[ldq_idx].killed) {
    return;
  }

  MicroOp task = inst;
  const bool debug_target = is_debug_load_uop(task);
  task.tma.is_cache_miss = false;
  uint32_t p_addr;
  auto mmu_ret = mmu->translate(p_addr, task.result, 1, in.csr_status);

  if (debug_target) {
    TEMP_BUG_TRACE_PRINTF(
        "[LSU LOAD DBG][agu] pc=0x%08x inst=0x%08x ldq=%u vaddr=0x%08x mmu_ret=%d paddr=0x%08x cyc=%lld\n",
        (uint32_t)task.dbg.pc, (uint32_t)task.dbg.instruction,
        (unsigned)task.ldq_idx, (uint32_t)task.result, (int)mmu_ret,
        (uint32_t)p_addr, (long long)sim_time);
  }

  if (mmu_ret == AbstractMmu::Result::RETRY) {
    task.cplt_time = REQ_WAIT_EXEC;
    nxt.ldq[ldq_idx].tlb_retry = true;
    nxt.ldq[ldq_idx].uop = task;
    return;
  }

  if (mmu_ret == AbstractMmu::Result::FAULT) {
    task.page_fault_load = true;
    task.diag_val = task.result;
    task.cplt_time = sim_time + 1;
  } else {
    task.diag_val = p_addr;
    nxt.ldq[ldq_idx].is_mmio_wait = is_mmio_addr(p_addr);

    nxt.ldq[ldq_idx].uop = task;
    nxt.ldq[ldq_idx].tlb_retry = false;
    update_load_ready_state(nxt.ldq[ldq_idx], p_addr, sim_time);
    return;
  }

  nxt.ldq[ldq_idx].tlb_retry = false;
  nxt.ldq[ldq_idx].uop = task;
}

void RealLsu::handle_store_addr(const MicroOp &inst) {
  Assert(inst.stq_idx >= 0 && inst.stq_idx < STQ_SIZE);
  if (inst.rob_idx == 133 && inst.rob_flag == 0) {
    TEMP_BUG_TRACE_PRINTF(
        "[LSU TRACE] handle store addr rob=%u/%u stq=%u/%u vaddr=0x%08x cyc=%lld\n",
        (unsigned)inst.rob_idx, (unsigned)inst.rob_flag,
        (unsigned)inst.stq_idx, (unsigned)inst.stq_flag,
        (uint32_t)inst.result, (long long)sim_time);
  }
  if (!finish_store_addr_once(inst)) {
    fifo_push_or_assert(nxt.pending_sta_addr_reqs, inst, "pending_sta_addr_reqs");
  }
}

void RealLsu::handle_store_data(const MicroOp &inst) {
  Assert(inst.stq_idx >= 0 && inst.stq_idx < STQ_SIZE);
  StqCommitEntry *entry = find_commit_stq_entry_by_rob(inst.rob_idx, inst.rob_flag);
  if (entry == nullptr) {
    LSU_MEM_DBG_PRINTF("[LSU WARN] missing commit_stq entry for store data rob=%u/%u stq=%u/%u cyc=%lld\n",
                       (unsigned)inst.rob_idx, (unsigned)inst.rob_flag,
                       (unsigned)inst.stq_idx, (unsigned)inst.stq_flag,
                       (long long)sim_time);
    return;
  }
  entry->data = inst.result;
  entry->data_valid = true;
  if (is_debug_store_data(inst.result) || is_debug_store_addr(entry->p_addr)) {
    TEMP_BUG_TRACE_PRINTF(
        "[LSU STORE DBG][data] rob=%u/%u stq=%u/%u data=0x%08x addr_v=%d paddr=0x%08x cyc=%lld\n",
        (unsigned)inst.rob_idx, (unsigned)inst.rob_flag,
        (unsigned)inst.stq_idx, (unsigned)inst.stq_flag,
        (uint32_t)inst.result, (int)entry->addr_valid,
        (uint32_t)entry->p_addr, (long long)sim_time);
  }
  if (inst.rob_idx == 133 && inst.rob_flag == 0) {
    TEMP_BUG_TRACE_PRINTF(
        "[LSU TRACE] store data rob=%u/%u stq=%u/%u data=0x%08x valid=%d cyc=%lld\n",
        (unsigned)inst.rob_idx, (unsigned)inst.rob_flag,
        (unsigned)inst.stq_idx, (unsigned)inst.stq_flag,
        (uint32_t)inst.result, (int)nxt.commit_stq.is_valid(inst.stq_idx),
        (long long)sim_time);
  }
}

bool RealLsu::reserve_stq_entry(mask_t br_mask, uint32_t rob_idx,
                                uint32_t rob_flag, uint32_t stq_idx,
                                uint32_t stq_flag, uint32_t func3) {
  if (ctx != nullptr) {
    ctx->perf.lsu_store_alloc_token_check_calls++;
    ctx->perf.lsu_store_alloc_token_check_entries_scanned++;
  }
  const auto [expect_idx, expect_flag] = next_dispatch_stq_token(nxt);
  Assert(expect_idx == stq_idx && expect_flag == stq_flag &&
         "RealLsu: unexpected STQ token / token reuse detected");
  const std::size_t alloc_slot = nxt.commit_stq.get_tail();
  const bool alloc_flag = nxt.commit_stq.get_tail_flag();
  StqCommitEntry entry = StqCommitEntry{};
  entry.br_mask = br_mask;
  entry.rob_idx = rob_idx;
  entry.rob_flag = rob_flag;
  entry.stq_idx = stq_idx;
  entry.stq_flag = stq_flag;
  entry.func3 = func3;
  nxt.commit_stq.push(entry);
  lookup_.commit_lookup_valid[rob_flag][rob_idx] = true;
  lookup_.commit_lookup_slot[rob_flag][rob_idx] =
      static_cast<wire<STQ_IDX_WIDTH>>(alloc_slot);
  if (rob_idx == 133 && rob_flag == 0) {
    TEMP_BUG_TRACE_PRINTF(
        "[LSU TRACE] alloc stq rob=%u/%u phys=%zu/%d token=%u/%u cyc=%lld\n",
        (unsigned)rob_idx, (unsigned)rob_flag, alloc_slot, (int)alloc_flag,
        (unsigned)stq_idx, (unsigned)stq_flag, (long long)sim_time);
  }
  return true;
}

void RealLsu::consume_stq_alloc_reqs() {
  for (int i = 0; i < MAX_STQ_DISPATCH_WIDTH; i++) {
    if (!in.dis2lsu->alloc_req[i]) {
      continue;
    }
    bool ok = reserve_stq_entry(in.dis2lsu->br_mask[i], in.dis2lsu->rob_idx[i],
                                in.dis2lsu->rob_flag[i],
                                in.dis2lsu->stq_idx[i],
                                in.dis2lsu->stq_flag[i],
                                in.dis2lsu->func3[i]);
    Assert(ok && "STQ allocate overflow");
  }
}

bool RealLsu::reserve_ldq_entry(int idx, mask_t br_mask, uint32_t rob_idx,
                                uint32_t rob_flag) {
  Assert(idx >= 0 && idx < LDQ_SIZE);
  if (nxt.ldq[idx].valid) {
    return false;
  }
  LdqEntry &entry = nxt.ldq[idx];
  entry = LdqEntry{};
  entry.valid = true;
  entry.uop.br_mask = br_mask;
  entry.uop.rob_idx = rob_idx;
  entry.uop.rob_flag = rob_flag;
  entry.uop.ldq_idx = idx;
  entry.uop.cplt_time = REQ_WAIT_EXEC;
  return true;
}

void RealLsu::consume_ldq_alloc_reqs() {
  for (int i = 0; i < MAX_LDQ_DISPATCH_WIDTH; i++) {
    if (!in.dis2lsu->ldq_alloc_req[i]) {
      continue;
    }
    bool ok = reserve_ldq_entry(
        in.dis2lsu->ldq_idx[i], in.dis2lsu->ldq_br_mask[i],
        in.dis2lsu->ldq_rob_idx[i], in.dis2lsu->ldq_rob_flag[i]);
    Assert(ok && "LDQ allocate collision");
  }
}

bool RealLsu::is_mmio_addr(uint32_t paddr) const {
  return ((paddr & UART_ADDR_MASK) == UART_ADDR_BASE) ||
         ((paddr & PLIC_ADDR_MASK) == PLIC_ADDR_BASE) ||
         (paddr == OPENSBI_TIMER_LOW_ADDR) ||
         (paddr == OPENSBI_TIMER_HIGH_ADDR);
}

void RealLsu::change_store_info(const StqDoneEntry &head, int port,
                                int store_index) {
  uint32_t alignment_mask = (head.func3 & 0x3) == 0   ? 0
                            : (head.func3 & 0x3) == 1 ? 1
                                                      : 3;
  Assert((head.p_addr & alignment_mask) == 0 &&
         "DUT: Store address misaligned at commit!");

  uint32_t byte_off = head.p_addr & 0x3;
  uint32_t wstrb = 0;
  uint32_t wdata = 0;
  switch (head.func3 & 0x3) {
  case 0:
    wstrb = (1u << byte_off);
    wdata = (head.data & 0xFFu) << (byte_off * 8);
    break;
  case 1:
    wstrb = (0x3u << byte_off);
    wdata = (head.data & 0xFFFFu) << (byte_off * 8);
    break;
  default:
    wstrb = 0xFu;
    wdata = head.data;
    break;
  }

  out.lsu2dcache->req_ports.store_ports[port].valid = true;
  out.lsu2dcache->req_ports.store_ports[port].addr = head.p_addr;
  out.lsu2dcache->req_ports.store_ports[port].strb = wstrb;
  out.lsu2dcache->req_ports.store_ports[port].data = wdata;
  out.lsu2dcache->req_ports.store_ports[port].req_id = store_index;
  if (is_debug_store_addr(head.p_addr) || is_debug_store_data(head.data)) {
    TEMP_BUG_TRACE_PRINTF(
        "[LSU STORE DBG][send] rob=%u/%u stq=%u/%u port=%d paddr=0x%08x data=0x%08x strb=0x%x req=%d cyc=%lld\n",
        (unsigned)head.rob_idx, (unsigned)head.rob_flag,
        (unsigned)head.stq_idx, (unsigned)head.stq_flag, port,
        (uint32_t)head.p_addr, (uint32_t)head.data, (unsigned)wstrb,
        store_index, (long long)sim_time);
  }
}

void RealLsu::handle_global_flush() {
  if (nxt.pending_mmio_valid && nxt.pending_mmio_req.uop.op == UOP_STA) {
    const uint32_t stq_idx = nxt.pending_mmio_req.uop.stq_idx;
    const uint32_t stq_flag = nxt.pending_mmio_req.uop.stq_flag;
    if (lookup_.done_token_lookup_valid[stq_flag][stq_idx]) {
      auto &entry =
          nxt.done_stq.slot(lookup_.done_token_lookup_slot[stq_flag][stq_idx]);
      entry.inflight = false;
    }
  }
  nxt.commit_stq.clear();
  clear_commit_lookup();
  nxt.pending_sta_addr_reqs.clear();
  nxt.pending_mmio_valid = false;
  nxt.pending_mmio_req = {};
  nxt.reserve_addr = 0;
  nxt.reserve_valid = false;
}

void RealLsu::handle_mispred(mask_t mask) {
  auto is_killed = [&](const MicroOp &u) { return (u.br_mask & mask) != 0; };

  for (int i = 0; i < LDQ_SIZE; i++) {
    if (!nxt.ldq[i].valid) {
      continue;
    }
    if (is_killed(nxt.ldq[i].uop)) {
      if (nxt.ldq[i].sent) {

        nxt.ldq[i].killed = true;
      } else {

        free_ldq_entry(i);
      }
    }
  }

  fifo_erase_if(nxt.finished_sta_reqs, is_killed, "finished_sta_reqs");
  fifo_erase_if(nxt.finished_loads, is_killed, "finished_loads");
  fifo_erase_if(nxt.pending_sta_addr_reqs, is_killed, "pending_sta_addr_reqs");

  if (nxt.pending_mmio_valid && (nxt.pending_mmio_req.uop.br_mask & mask) != 0) {
    nxt.pending_mmio_valid = false;
    nxt.pending_mmio_req = {};
  }

  fifo_flag_erase_if(
      nxt.commit_stq,
      [&](const StqCommitEntry &entry) { return (entry.br_mask & mask) != 0; },
      "commit_stq");
  rebuild_commit_lookup();
}


void RealLsu::retire_stq_head_if_ready() {
  if(nxt.done_stq.empty()) {
    return;
  }
  StqDoneEntry head = nxt.done_stq[0];
  while (head.done || head.suppress_write) {
    // Process the head entry
    lookup_.done_lookup_valid[head.rob_flag][head.rob_idx] = false;
    lookup_.done_token_lookup_valid[head.stq_flag][head.stq_idx] = false;
    nxt.done_stq.pop();
    if(nxt.done_stq.empty()) {
      break;
    }
    head= nxt.done_stq[0];
  }
  return ;
}

void RealLsu::commit_stores_from_rob() {
  bool head_not_ready_blocked = false;
  bool done_stq_full_blocked = false;
  for (int i = 0; i < COMMIT_WIDTH; i++) {
    if (!in.rob_commit->commit_entry[i].valid) {
      continue;
    }
    const auto &commit_uop = in.rob_commit->commit_entry[i].uop;
    if (!is_store(commit_uop)) {
      continue;
    }
    
    const StqCommitEntry src = nxt.commit_stq.front();
    if(!src.addr_valid || !src.data_valid) {
      // 地址或数据未准备好，等待下一周期继续检查
      head_not_ready_blocked = true;
      continue;
    }
    if (nxt.done_stq.full()) {
      // Keep the older committed store at the head of commit_stq and stop
      // consuming younger stores this cycle. This preserves store age order
      // and avoids overflowing done_stq.
      done_stq_full_blocked = true;
      break;
    }
    StqDoneEntry dst{};
    dst.done = src.suppress_write;
    dst.inflight = false;
    dst.is_mmio = src.is_mmio;
    dst.suppress_write = src.suppress_write;
    dst.replay_priority = 0;
    dst.p_addr = src.p_addr;
    dst.data = src.data;
    dst.func3 = src.func3;
    dst.rob_idx = src.rob_idx;
    dst.rob_flag = src.rob_flag;
    dst.stq_idx = src.stq_idx;
    dst.stq_flag = src.stq_flag;
    if (is_debug_store_addr(dst.p_addr) || is_debug_store_data(dst.data)) {
      TEMP_BUG_TRACE_PRINTF(
          "[LSU STORE DBG][commit] rob=%u/%u stq=%u/%u paddr=0x%08x data=0x%08x cyc=%lld\n",
          (unsigned)dst.rob_idx, (unsigned)dst.rob_flag,
          (unsigned)dst.stq_idx, (unsigned)dst.stq_flag,
          (uint32_t)dst.p_addr, (uint32_t)dst.data, (long long)sim_time);
    }
    nxt.commit_stq.pop();
    lookup_.commit_lookup_valid[src.rob_flag][src.rob_idx] = false;
    const std::size_t done_slot = nxt.done_stq.get_tail();
    const bool ok = nxt.done_stq.push(std::move(dst));
    Assert(ok && "done_stq push failed unexpectedly");
    lookup_.done_lookup_valid[src.rob_flag][src.rob_idx] = true;
    lookup_.done_lookup_slot[src.rob_flag][src.rob_idx] =
        static_cast<wire<STQ_IDX_WIDTH>>(done_slot);
    lookup_.done_token_lookup_valid[src.stq_flag][src.stq_idx] = true;
    lookup_.done_token_lookup_slot[src.stq_flag][src.stq_idx] =
        static_cast<wire<STQ_IDX_WIDTH>>(done_slot);

  }
  if (ctx != nullptr) {
    if (head_not_ready_blocked) {
      ctx->perf.lsu_commit_blocked_head_not_ready_cycles++;
    }
    if (done_stq_full_blocked) {
      ctx->perf.lsu_commit_blocked_done_stq_full_cycles++;
    }
  }
}

void RealLsu::comb_pipeline(){
  ScopedPerfNs scoped(ctx ? &ctx->perf.lsu_host_time_comb_pipeline_ns : nullptr);
  bool is_flush = in.rob_bcast->flush;
  bool is_mispred = in.dec_bcast->mispred;

  if (is_flush) {
    mmu->flush();
    handle_global_flush();
    return;
  }

  if (is_mispred) {
    mmu->flush();
    handle_mispred(in.dec_bcast->br_mask);
  }

  // 清除已解析分支的 br_mask bit（在 flush 之后，只影响存活条目）
  mask_t clear = in.dec_bcast->clear_mask;
  if (clear) {
    for (int i = 0; i < LDQ_SIZE; i++) {
      if (nxt.ldq[i].valid)
        nxt.ldq[i].uop.br_mask &= ~clear;
    }
    for (int i = 0; i < nxt.commit_stq.size(); i++) {
      nxt.commit_stq[i].br_mask &= ~clear;
    }
    for (std::size_t i = 0; i < nxt.finished_sta_reqs.size(); ++i)
      nxt.finished_sta_reqs[i].br_mask &= ~clear;
    for (std::size_t i = 0; i < nxt.finished_loads.size(); ++i)
      nxt.finished_loads[i].br_mask &= ~clear;
    for (std::size_t i = 0; i < nxt.pending_sta_addr_reqs.size(); ++i)
      nxt.pending_sta_addr_reqs[i].br_mask &= ~clear;
  }

  if (is_mispred) {
    return;
  }

  if (in.rob_bcast->fence) {
    mmu->flush();
  }

  consume_stq_alloc_reqs();
  consume_ldq_alloc_reqs();
  commit_stores_from_rob();

  progress_ldq_entries();

  // Retire after load progress so same-cycle completed stores can still
  // participate in store-to-load forwarding.
  retire_stq_head_if_ready();
}

void RealLsu::progress_ldq_entries() {
  for (int i = 0; i < LDQ_SIZE; i++) {
    auto &entry = nxt.ldq[i];
    if (!entry.valid) {
      continue;
    }
    const bool debug_target = is_debug_load_uop(entry.uop);
    if (entry.killed && !entry.sent) {

      free_ldq_entry(i);
      continue;
    }

    if (entry.waiting_resp || entry.uop.cplt_time == REQ_WAIT_EXEC) {
      if (!entry.tlb_retry) {
        continue;
      }
      uint32_t p_addr = 0;
      auto mmu_ret = mmu->translate(p_addr, entry.uop.result, 1, in.csr_status);
      if (mmu_ret == AbstractMmu::Result::RETRY) {
        continue;
      }
      entry.tlb_retry = false;
      if (mmu_ret == AbstractMmu::Result::FAULT) {
        entry.uop.page_fault_load = true;
        entry.uop.diag_val = entry.uop.result;
        entry.uop.cplt_time = sim_time + 1;
      } else {
        update_load_ready_state(entry, p_addr, sim_time);
      }
      continue;
    }

    if (entry.uop.cplt_time == REQ_WAIT_RETRY) {
      if (debug_target) {
        TEMP_BUG_TRACE_PRINTF(
            "[LSU LOAD DBG][retry] pc=0x%08x ldq=%d paddr=0x%08x replay=%u sent=%d wait=%d cyc=%lld\n",
            (uint32_t)entry.uop.dbg.pc, i, (uint32_t)entry.uop.diag_val,
            (unsigned)entry.replay_priority, (int)entry.sent,
            (int)entry.waiting_resp, (long long)sim_time);
      }
      auto fwd_res = check_store_forward(entry.uop.diag_val, entry.uop);
      if (fwd_res.state == StoreForwardState::Hit) {
        entry.uop.result = fwd_res.data;
        entry.uop.cplt_time = sim_time;
      } else if (fwd_res.state == StoreForwardState::NoHit) {
        entry.uop.cplt_time = REQ_WAIT_SEND;
      }
    }

    if (entry.uop.cplt_time <= sim_time) {
      if (debug_target) {
        TEMP_BUG_TRACE_PRINTF(
            "[LSU LOAD DBG][finish] pc=0x%08x ldq=%d result=0x%08x cplt=%lld cyc=%lld\n",
            (uint32_t)entry.uop.dbg.pc, i, (uint32_t)entry.uop.result,
            (long long)entry.uop.cplt_time, (long long)sim_time);
      }
      if (!entry.killed) {
        if (is_amo_lr_uop(entry.uop)) {
          nxt.reserve_valid = true;
          nxt.reserve_addr = entry.uop.diag_val;
        }
        fifo_push_or_assert(nxt.finished_loads, entry.uop, "finished_loads");
      }
      free_ldq_entry(i);
    }
  }
}

void RealLsu::update_load_ready_state(LdqEntry &entry, uint32_t p_addr,
                                      int64_t ready_cycle) {
  entry.uop.diag_val = p_addr;
  entry.is_mmio_wait = is_mmio_addr(p_addr);
  const bool debug_target =
      is_debug_load_uop(entry.uop) || p_addr == kDebugLoadPaddr;

  const StoreForwardResult fwd_res =
      entry.is_mmio_wait ? StoreForwardResult{}
                         : check_store_forward(p_addr, entry.uop);
  if (debug_target) {
    TEMP_BUG_TRACE_PRINTF(
        "[LSU LOAD DBG][ready] pc=0x%08x inst=0x%08x ldq=%u paddr=0x%08x mmio=%d fwd=%d data=0x%08x cyc=%lld\n",
        (uint32_t)entry.uop.dbg.pc, (uint32_t)entry.uop.dbg.instruction,
        (unsigned)entry.uop.ldq_idx, (uint32_t)p_addr, (int)entry.is_mmio_wait,
        (int)fwd_res.state, (uint32_t)fwd_res.data, (long long)sim_time);
  }
  if (fwd_res.state == StoreForwardState::Hit) {
    entry.uop.result = fwd_res.data;
    entry.uop.cplt_time = ready_cycle;
  } else if (fwd_res.state == StoreForwardState::NoHit) {
    entry.uop.cplt_time = REQ_WAIT_SEND;
  } else {
    entry.uop.cplt_time = REQ_WAIT_RETRY;
  }
}

bool RealLsu::finish_store_addr_once(const MicroOp &inst) {
  Assert(inst.stq_idx >= 0 && inst.stq_idx < STQ_SIZE);
  StqCommitEntry *commit_entry =
      find_commit_stq_entry_by_rob(inst.rob_idx, inst.rob_flag);

  if (commit_entry == nullptr) {
    if (ctx != nullptr) {
      ctx->perf.lsu_store_addr_retry_no_commit_entry_count++;
    }
    if (inst.rob_idx == 133 && inst.rob_flag == 0) {
      TEMP_BUG_TRACE_PRINTF(
          "[LSU TRACE] store addr rob=%u/%u stq=%u/%u commit_entry=null cyc=%lld\n",
          (unsigned)inst.rob_idx, (unsigned)inst.rob_flag,
          (unsigned)inst.stq_idx, (unsigned)inst.stq_flag,
          (long long)sim_time);
    }
    // STA can arrive in comb_recv() before this cycle's dis2lsu allocation is
    // materialized into commit_stq by comb_pipeline(). Retry on the next cycle
    // instead of silently dropping the resolved address.
    return false;
  }


  uint32_t pa = inst.result;
  auto mmu_ret = mmu->translate(pa, inst.result, 2, in.csr_status);
  if (inst.rob_idx == 133 && inst.rob_flag == 0) {
    TEMP_BUG_TRACE_PRINTF(
        "[LSU TRACE] store addr rob=%u/%u stq=%u/%u mmu_ret=%d pa=0x%08x cyc=%lld\n",
        (unsigned)inst.rob_idx, (unsigned)inst.rob_flag,
        (unsigned)inst.stq_idx, (unsigned)inst.stq_flag, (int)mmu_ret,
        (uint32_t)pa, (long long)sim_time);
  }
  if (mmu_ret == AbstractMmu::Result::RETRY) {
    if (ctx != nullptr) {
      ctx->perf.lsu_store_addr_retry_mmu_count++;
    }
    return false;
  }

  if (mmu_ret == AbstractMmu::Result::FAULT) {
    MicroOp fault_op = inst;
    fault_op.page_fault_store = true;
    fault_op.cplt_time = sim_time;
    if (is_amo_sc_uop(inst)) {
      nxt.reserve_valid = false;
      fault_op.op = UOP_LOAD; // Reuse existing LSU load wb/awake path for SC result
      fault_op.dest_en = true;
      fifo_push_or_assert(nxt.finished_loads, fault_op, "finished_loads");
    }else{
      fifo_push_or_assert(nxt.finished_sta_reqs, fault_op, "finished_sta_reqs");
    }
    commit_entry->p_addr = pa;
    commit_entry->addr_valid = false;
    return true;
  }

  MicroOp success_op = inst;
  success_op.cplt_time = sim_time;
  if (is_amo_sc_uop(inst)) {
    bool sc_success = nxt.reserve_valid && (nxt.reserve_addr == pa);
    // SC clears reservation regardless of success/failure.
    nxt.reserve_valid = false;
    success_op.result = sc_success ? 0 : 1;
    success_op.dest_en = true;
    success_op.op =
        UOP_LOAD; // Reuse existing LSU load wb/awake path for SC result
    commit_entry->suppress_write = !sc_success;
    commit_entry->is_mmio = false;
    commit_entry->p_addr = pa;
    commit_entry->addr_valid = true;
    fifo_push_or_assert(nxt.finished_loads, success_op, "finished_loads");
    return true;
  }
  bool is_mmio = is_mmio_addr(pa);
  // MMIO store must not trigger ROB flush at STA writeback. Otherwise ROB may
  // flush globally before LSU consumes rob_commit, dropping the STQ commit.
  success_op.flush_pipe = false;
  fifo_push_or_assert(nxt.finished_sta_reqs, success_op, "finished_sta_reqs");
  
  commit_entry->is_mmio = is_mmio;
  commit_entry->p_addr = pa;
  commit_entry->addr_valid = true;
  if (is_debug_store_addr(pa) || is_debug_store_data(commit_entry->data)) {
    TEMP_BUG_TRACE_PRINTF(
        "[LSU STORE DBG][addr] rob=%u/%u stq=%u/%u paddr=0x%08x data_v=%d data=0x%08x cyc=%lld\n",
        (unsigned)inst.rob_idx, (unsigned)inst.rob_flag,
        (unsigned)inst.stq_idx, (unsigned)inst.stq_flag, (uint32_t)pa,
        (int)commit_entry->data_valid, (uint32_t)commit_entry->data,
        (long long)sim_time);
  }
  if (inst.rob_idx == 133 && inst.rob_flag == 0) {
    TEMP_BUG_TRACE_PRINTF(
        "[LSU TRACE] store addr commit rob=%u/%u stq=%u/%u paddr=0x%08x cyc=%lld\n",
        (unsigned)inst.rob_idx, (unsigned)inst.rob_flag,
        (unsigned)inst.stq_idx, (unsigned)inst.stq_flag, (uint32_t)pa,
        (long long)sim_time);
  }
  return true;
}

StqCommitEntry *RealLsu::find_commit_stq_entry_by_rob(uint32_t rob_idx,
                                                      uint32_t rob_flag) {
  if (ctx != nullptr) {
    ctx->perf.lsu_find_commit_calls++;
  }
  if (lookup_.commit_lookup_valid[rob_flag][rob_idx]) {
    if (ctx != nullptr) {
      ctx->perf.lsu_find_commit_entries_scanned++;
    }
    const std::size_t slot = lookup_.commit_lookup_slot[rob_flag][rob_idx];
    if (nxt.commit_stq.is_valid(slot)) {
      auto &entry = nxt.commit_stq.slot(slot);
      if (entry.rob_idx == rob_idx && entry.rob_flag == rob_flag) {
        return &entry;
      }
    }
    lookup_.commit_lookup_valid[rob_flag][rob_idx] = false;
  }
  for (std::size_t slot = 0; slot < STQ_SIZE; ++slot) {
    if (ctx != nullptr) {
      ctx->perf.lsu_find_commit_entries_scanned++;
    }
    if (!nxt.commit_stq.is_valid(slot)) {
      continue;
    }
    auto &entry = nxt.commit_stq.slot(slot);
    if (entry.rob_idx == rob_idx && entry.rob_flag == rob_flag) {
      lookup_.commit_lookup_valid[rob_flag][rob_idx] = true;
      lookup_.commit_lookup_slot[rob_flag][rob_idx] =
          static_cast<wire<STQ_IDX_WIDTH>>(slot);
      return &entry;
    }
  }
  return nullptr;
}

const StqCommitEntry *RealLsu::find_commit_stq_entry_by_rob(
    uint32_t rob_idx, uint32_t rob_flag) const {
  if (ctx != nullptr) {
    ctx->perf.lsu_find_commit_calls++;
  }
  if (lookup_.commit_lookup_valid[rob_flag][rob_idx]) {
    if (ctx != nullptr) {
      ctx->perf.lsu_find_commit_entries_scanned++;
    }
    const std::size_t slot = lookup_.commit_lookup_slot[rob_flag][rob_idx];
    if (nxt.commit_stq.is_valid(slot)) {
      const auto &entry = nxt.commit_stq.slot(slot);
      if (entry.rob_idx == rob_idx && entry.rob_flag == rob_flag) {
        return &entry;
      }
    }
  }
  for (std::size_t slot = 0; slot < STQ_SIZE; ++slot) {
    if (ctx != nullptr) {
      ctx->perf.lsu_find_commit_entries_scanned++;
    }
    if (!nxt.commit_stq.is_valid(slot)) {
      continue;
    }
    const auto &entry = nxt.commit_stq.slot(slot);
    if (entry.rob_idx == rob_idx && entry.rob_flag == rob_flag) {
      return &entry;
    }
  }
  return nullptr;
}

void RealLsu::progress_pending_sta_addr() {
  if (cur.pending_sta_addr_reqs.empty()) {
    return;
  }
  size_t n = cur.pending_sta_addr_reqs.size();
  for (size_t i = 0; i < n; i++) {
    MicroOp op = nxt.pending_sta_addr_reqs.front();
    nxt.pending_sta_addr_reqs.pop();
    if (!finish_store_addr_once(op)) {
      fifo_push_or_assert(nxt.pending_sta_addr_reqs, op,
                          "pending_sta_addr_reqs");
    }
  }
}

// =========================================================
// 5. Exception: Flush 处理
// =========================================================

void RealLsu::comb_flush() {
  if (in.rob_bcast->flush) {
    // 1. LDQ: 已发请求项标记 killed，未发请求项直接释放
    for (int i = 0; i < LDQ_SIZE; i++) {
      if (!nxt.ldq[i].valid) {
        continue;
      }
      if (nxt.ldq[i].sent) {

        nxt.ldq[i].killed = true;
      } else {

        free_ldq_entry(i);
      }
    }
    nxt.finished_loads.clear();
    nxt.finished_sta_reqs.clear();
    nxt.pending_sta_addr_reqs.clear();
  }
}
void RealLsu::free_ldq_entry(int idx) {
  Assert(idx >= 0 && idx < LDQ_SIZE);
  nxt.ldq[idx] = LdqEntry{};
}
// =========================================================
// 6. Sequential Logic: 状态更新与时序模拟
// =========================================================
void RealLsu::seq() {
  ScopedPerfNs scoped(ctx ? &ctx->perf.lsu_host_time_seq_ns : nullptr);
  cur = nxt;
}

void RealLsu::dump_debug_state() const {
  std::printf(
      "[LSU DEBUG] pending_mmio_valid=%d reserve_valid=%d reserve_addr=0x%08x replay_type=%d pending_sta_addr=%zu finished_loads=%zu finished_sta=%zu\n",
      (int)cur.pending_mmio_valid, (int)cur.reserve_valid,
      (uint32_t)cur.reserve_addr, (int)cur.replay_type,
      cur.pending_sta_addr_reqs.size(), cur.finished_loads.size(),
      cur.finished_sta_reqs.size());
  std::printf(
      "[LSU DEBUG] commit_stq size=%zu head=%zu head_flag=%d tail=%zu tail_flag=%d\n",
      cur.commit_stq.size(), cur.commit_stq.get_head(),
      (int)cur.commit_stq.get_head_flag(), cur.commit_stq.get_tail(),
      (int)cur.commit_stq.get_tail_flag());
  for (std::size_t slot = 0; slot < STQ_SIZE; ++slot) {
    if (!cur.commit_stq.is_valid(slot)) {
      continue;
    }
    const auto &e = cur.commit_stq.slot(slot);
    std::printf(
        "[LSU DEBUG][commit_stq] slot=%zu stq=%u/%u rob=%u/%u addr_v=%d data_v=%d sup=%d mmio=%d paddr=0x%08x data=0x%08x f3=%u\n",
        slot, (unsigned)e.stq_idx, (unsigned)e.stq_flag,
        (unsigned)e.rob_idx, (unsigned)e.rob_flag, (int)e.addr_valid,
        (int)e.data_valid, (int)e.suppress_write, (int)e.is_mmio,
        (uint32_t)e.p_addr, (uint32_t)e.data, (unsigned)e.func3);
  }

  std::printf(
      "[LSU DEBUG] done_stq size=%zu head=%zu head_flag=%d tail=%zu tail_flag=%d\n",
      cur.done_stq.size(), cur.done_stq.get_head(),
      (int)cur.done_stq.get_head_flag(), cur.done_stq.get_tail(),
      (int)cur.done_stq.get_tail_flag());
  for (std::size_t slot = 0; slot < STQ_SIZE; ++slot) {
    if (!cur.done_stq.is_valid(slot)) {
      continue;
    }
    const auto &e = cur.done_stq.slot(slot);
    std::printf(
        "[LSU DEBUG][done_stq] slot=%zu stq=%u/%u rob=%u/%u done=%d inflight=%d replay=%u sup=%d mmio=%d paddr=0x%08x data=0x%08x f3=%u\n",
        slot, (unsigned)e.stq_idx, (unsigned)e.stq_flag,
        (unsigned)e.rob_idx, (unsigned)e.rob_flag, (int)e.done,
        (int)e.inflight, (unsigned)e.replay_priority, (int)e.suppress_write,
        (int)e.is_mmio, (uint32_t)e.p_addr, (uint32_t)e.data,
        (unsigned)e.func3);
  }

  for (int i = 0; i < LDQ_SIZE; ++i) {
    const auto &e = cur.ldq[i];
    if (!e.valid) {
      continue;
    }
    std::printf(
        "[LSU DEBUG][ldq] idx=%d rob=%u/%u stq_stop=%u/%u sent=%d wait=%d tlb_retry=%d mmio=%d killed=%d replay=%u cplt=%lld wait_since=%llu paddr=0x%08x pc=0x%08x inst=0x%08x\n",
        i, (unsigned)e.uop.rob_idx, (unsigned)e.uop.rob_flag,
        (unsigned)e.uop.stq_idx, (unsigned)e.uop.stq_flag, (int)e.sent,
        (int)e.waiting_resp, (int)e.tlb_retry, (int)e.is_mmio_wait,
        (int)e.killed, (unsigned)e.replay_priority,
        (long long)e.uop.cplt_time, (unsigned long long)e.wait_resp_since,
        (uint32_t)e.uop.diag_val, (uint32_t)e.uop.dbg.pc,
        (uint32_t)e.uop.dbg.instruction);
    if (e.waiting_resp && e.wait_resp_since != 0) {
      std::printf("[LSU DEBUG][ldq_wait] idx=%d waited=%llu cycles\n", i,
                  (unsigned long long)(sim_time - e.wait_resp_since));
    }
  }
}

bool RealLsu::has_older_store_pending(const MicroOp &load_uop) const {
  if (!nxt.commit_stq.empty()) {
    for (std::size_t i = 0; i < nxt.commit_stq.size(); ++i) {
      const StqCommitEntry entry = nxt.commit_stq[i];
      if (!commit_store_is_older_than_load(entry, load_uop)) {
        break;
      }
      if (!entry.suppress_write) {
        return true;
      }
    }
  }

  for (std::size_t i = 0; i < nxt.done_stq.size(); ++i) {
    const StqDoneEntry entry = nxt.done_stq[i];
    if (!entry.suppress_write) {
      return true;
    }
  }

  return false;
}

// =========================================================
// 🛡️ [Nanako Implementation] 完整的 STLF 模拟逻辑
// =========================================================

StoreForwardResult RealLsu::check_store_forward(uint32_t p_addr,
                                                const MicroOp &load_uop) {
  if (ctx != nullptr) {
    ctx->perf.lsu_check_store_forward_calls++;
  }
  uint32_t current_word = 0;
  bool hit_any = false;
  const bool debug_target =
      is_debug_load_uop(load_uop) || p_addr == kDebugLoadPaddr;

  // Global age order is: done_stq (oldest committed stores still draining)
  // before commit_stq (younger not-yet-committed stores). Forwarding merges
  // from older to younger so that younger stores overwrite older bytes.
  for (std::size_t i = 0; i < nxt.done_stq.size(); ++i) {
    if (ctx != nullptr) {
      ctx->perf.lsu_check_store_forward_done_entries_scanned++;
    }
    const StqDoneEntry entry = nxt.done_stq[i];
    if (entry.suppress_write) {
      continue;
    }
    if (debug_target) {
      TEMP_BUG_TRACE_PRINTF(
          "[LSU LOAD DBG][done] pc=0x%08x load_addr=0x%08x stop=%u/%u entry=%u/%u done=%d inflight=%d paddr=0x%08x data=0x%08x f3=%u\n",
          (uint32_t)load_uop.dbg.pc, (uint32_t)p_addr,
          (unsigned)load_uop.stq_idx, (unsigned)load_uop.stq_flag,
          (unsigned)entry.stq_idx, (unsigned)entry.stq_flag,
          (int)entry.done, (int)entry.inflight, (uint32_t)entry.p_addr,
          (uint32_t)entry.data, (unsigned)entry.func3);
    }

    int store_width = get_mem_width(entry.func3);
    int load_width = get_mem_width(load_uop.func3);
    uint32_t s_start = entry.p_addr;
    uint32_t s_end = s_start + store_width;
    uint32_t l_start = p_addr;
    uint32_t l_end = l_start + load_width;
    uint32_t overlap_start = std::max(s_start, l_start);
    uint32_t overlap_end = std::min(s_end, l_end);

    if (s_start <= l_start && s_end >= l_end) {
      hit_any = true;
      current_word =
          merge_data_to_word(current_word, entry.data, entry.p_addr, entry.func3);
    } else if (overlap_start < overlap_end) {
      hit_any = true;
      return {StoreForwardState::Retry, 0};
    }
  }

  // Scan the older prefix of commit_stq in ROB age order. This avoids relying
  // on (stq_idx, stq_flag) snapshots after the queue has wrapped many times
  // while a long-lived load is still waiting to execute.
  if (!nxt.commit_stq.empty()) {
    for (std::size_t i = 0; i < nxt.commit_stq.size(); ++i) {
      if (ctx != nullptr) {
        ctx->perf.lsu_check_store_forward_commit_entries_scanned++;
      }
      const StqCommitEntry entry = nxt.commit_stq[i];
      if (!commit_store_is_older_than_load(entry, load_uop)) {
        break;
      }
      if (!entry.suppress_write) {
        if (debug_target) {
          TEMP_BUG_TRACE_PRINTF(
              "[LSU LOAD DBG][commit] pc=0x%08x load_addr=0x%08x stop=%u/%u entry=%u/%u addr_v=%d data_v=%d paddr=0x%08x data=0x%08x f3=%u\n",
              (uint32_t)load_uop.dbg.pc, (uint32_t)p_addr,
              (unsigned)load_uop.stq_idx, (unsigned)load_uop.stq_flag,
              (unsigned)entry.stq_idx, (unsigned)entry.stq_flag,
              (int)entry.addr_valid, (int)entry.data_valid,
              (uint32_t)entry.p_addr, (uint32_t)entry.data,
              (unsigned)entry.func3);
        }
        if (!entry.addr_valid) {
          return {StoreForwardState::Retry, 0};
        }

        int store_width = get_mem_width(entry.func3);
        int load_width = get_mem_width(load_uop.func3);
        uint32_t s_start = entry.p_addr;
        uint32_t s_end = s_start + store_width;
        uint32_t l_start = p_addr;
        uint32_t l_end = l_start + load_width;
        uint32_t overlap_start = std::max(s_start, l_start);
        uint32_t overlap_end = std::min(s_end, l_end);

        if (s_start <= l_start && s_end >= l_end) {
          hit_any = true;
          if (!entry.data_valid) {
            return {StoreForwardState::Retry, 0};
          }
          current_word = merge_data_to_word(current_word, entry.data,
                                            entry.p_addr, entry.func3);
        } else if (overlap_start < overlap_end) {
          hit_any = true;
          return {StoreForwardState::Retry, 0};
        }
      }
    }
  }

  if (!hit_any) {
    if (debug_target) {
      TEMP_BUG_TRACE_PRINTF(
          "[LSU LOAD DBG] pc=0x%08x load_addr=0x%08x result=NoHit\n",
          (uint32_t)load_uop.dbg.pc, (uint32_t)p_addr);
    }
    return {StoreForwardState::NoHit, 0};
  }
  if (debug_target) {
    TEMP_BUG_TRACE_PRINTF(
        "[LSU LOAD DBG] pc=0x%08x load_addr=0x%08x result=Hit data=0x%08x\n",
        (uint32_t)load_uop.dbg.pc, (uint32_t)p_addr,
        (uint32_t)extract_data(current_word, p_addr, load_uop.func3));
  }
  return {StoreForwardState::Hit,
          extract_data(current_word, p_addr, load_uop.func3)};
}
