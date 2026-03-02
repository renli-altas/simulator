#include "RealDcache.h"
#include "PeripheralModel.h"
#include "oracle.h"
#include "util.h"
#include <algorithm>
#include <cstdlib>
#include <cstring>

extern uint32_t *p_memory;

// =============================================================
// 构造 / init
// =============================================================

RealDcache::RealDcache(SimContext *ctx) : ctx(ctx) {
  memset(cache_valid, 0, sizeof(cache_valid));
  memset(cache_tag,   0, sizeof(cache_tag));
  memset(cache_data,  0, sizeof(cache_data));
  memset(plru_tree,   0, sizeof(plru_tree));
  for (int i = 0; i < MSHR_NUM;    i++) mshr[i]       = {};
  for (int i = 0; i < WB_NUM;      i++) wb[i]         = {};
  for (int i = 0; i < TOTAL_PORTS; i++) pipe_s1[i]     = {};
  for (int i = 0; i < TOTAL_PORTS; i++) pipe_s1_next[i] = {};
}

void RealDcache::init() {
  memset(cache_valid, 0, sizeof(cache_valid));
  memset(cache_tag,   0, sizeof(cache_tag));
  memset(cache_data,  0, sizeof(cache_data));
  memset(plru_tree,   0, sizeof(plru_tree));
  for (int i = 0; i < MSHR_NUM;    i++) mshr[i]       = {};
  for (int i = 0; i < WB_NUM;      i++) wb[i]         = {};
  for (int i = 0; i < TOTAL_PORTS; i++) pipe_s1[i]     = {};
  for (int i = 0; i < TOTAL_PORTS; i++) pipe_s1_next[i] = {};
}

// =============================================================
// PLRU 辅助
// =============================================================

bool RealDcache::get_plru_bit(uint8_t tree[], int bit_index) const {
  return (tree[bit_index / 8] >> (bit_index % 8)) & 1;
}

void RealDcache::set_plru_bit(uint8_t tree[], int bit_index, bool value) {
  int byte_idx   = bit_index / 8;
  int bit_offset = bit_index % 8;
  if (value) tree[byte_idx] |=  (1 << bit_offset);
  else       tree[byte_idx] &= ~(1 << bit_offset);
}

int RealDcache::select_evict_way(uint32_t addr) {
  uint32_t index = (uint32_t)get_index(addr);
  int tree_depth = 0, temp = WAY_NUM;
  while (temp > 1) { tree_depth++; temp >>= 1; }

  int current_node = 0, selected_way = 0;
  for (int level = 0; level < tree_depth; level++) {
    bool go_left = !get_plru_bit(plru_tree[index], current_node);
    if (go_left) {
      set_plru_bit(plru_tree[index], current_node, true);
      current_node = 2 * current_node + 1;
    } else {
      set_plru_bit(plru_tree[index], current_node, false);
      current_node = 2 * current_node + 2;
    }
    selected_way = (selected_way << 1) | (go_left ? 0 : 1);
  }
  return std::min(selected_way, WAY_NUM - 1);
}

void RealDcache::update_plru(uint32_t index, int accessed_way) {
  int tree_depth = 0, temp = WAY_NUM;
  while (temp > 1) { tree_depth++; temp >>= 1; }

  int current_node = 0;
  int mask = 1 << (tree_depth - 1);
  for (int level = 0; level < tree_depth; level++) {
    bool went_left = !(accessed_way & mask);
    if (went_left) {
      set_plru_bit(plru_tree[index], current_node, true);
      current_node = 2 * current_node + 1;
    } else {
      set_plru_bit(plru_tree[index], current_node, false);
      current_node = 2 * current_node + 2;
    }
    mask >>= 1;
  }
}

// =============================================================
// Cache 查询 / 填充
// =============================================================

// 仅查标签（当前状态），返回命中 way；-1=miss，不更新 PLRU。
int RealDcache::lookup(uint32_t addr) {
  int idx = get_index(addr);
  int tag = get_tag(addr);
  for (int w = 0; w < WAY_NUM; w++) {
    if (cache_valid[w][idx] && cache_tag[w][idx] == (uint32_t)tag)
      return w;
  }
  return -1;
}

// 选一路替换：更新 tag/valid，并从 p_memory 填充整条 cache_data，
// 返回被替换的 way。
int RealDcache::fill(uint32_t addr) {
  int way = select_evict_way(addr);
  int idx = get_index(addr);
  cache_tag  [way][idx] = (uint32_t)get_tag(addr);
  cache_valid[way][idx] = true;

  // 从 p_memory 填充整条 cache line 的数据
  uint32_t line_base = addr & ~((1u << OFFSET_WIDTH) - 1u);
  for (int w = 0; w < WORDS_PER_LINE; w++)
    cache_data[way][idx][w] = p_memory[(line_base >> 2) + w];

  return way;
}

// =============================================================
// MSHR 辅助
// =============================================================

int RealDcache::find_free_mshr() const {
  for (int i = 0; i < MSHR_NUM; i++)
    if (!mshr[i].valid) return i;
  return -1;
}

int RealDcache::find_mshr_by_line(uint32_t addr) const {
  uint32_t line = addr >> OFFSET_WIDTH;
  for (int i = 0; i < MSHR_NUM; i++) {
    if (mshr[i].valid && !mshr[i].resp_pending &&
        (mshr[i].addr >> OFFSET_WIDTH) == line)
      return i;
  }
  return -1;
}

// =============================================================
// 写缓冲辅助
// =============================================================

int RealDcache::find_free_wb() const {
  for (int i = 0; i < WB_NUM; i++)
    if (!wb[i].valid) return i;
  return -1;
}

// 将写缓冲条目 drain 到 p_memory，并同步更新 cache_data（若该行仍在 cache 中）。
void RealDcache::drain_wb_entry(int idx) {
  if (!wb[idx].valid) return;
  uint32_t paddr   = wb[idx].addr;
  uint32_t old_val = p_memory[paddr >> 2];
  uint32_t wmask   = 0;
  for (int b = 0; b < 4; b++) {
    if (wb[idx].strb & (1 << b))
      wmask |= (0xFFu << (b * 8));
  }
  uint32_t new_val = (old_val & ~wmask) | (wb[idx].data & wmask);
  p_memory[paddr >> 2] = new_val;
  if (peripheral_model)
    peripheral_model->on_mem_store_effective(paddr, new_val);

  // 若该 cache line 仍在 cache 中，同步更新 cache_data
  int way = lookup(paddr);
  if (way >= 0)
    cache_data[way][get_index(paddr)][get_word_off(paddr)] = new_val;

  wb[idx].valid = false;
}

// =============================================================
// 数据读取辅助
// =============================================================

uint32_t RealDcache::read_mem_data(uint32_t p_addr) const {
  if (p_addr == 0x1fd0e000) {
#ifdef CONFIG_BPU
    return (uint32_t)sim_time;
#else
    return get_oracle_timer();
#endif
  }
  if (p_addr == 0x1fd0e004) return 0;
  return p_memory[p_addr >> 2];
}

bool RealDcache::is_difftest_skip_addr(uint32_t p_addr) const {
  return (p_addr == 0x1fd0e000 || p_addr == 0x1fd0e004);
}

// =============================================================
// S2 Load 端口处理
// =============================================================

void RealDcache::process_load_s2(int port, const PipeS1Entry &s1,
                                  DCacheRespPorts &resp) {
  uint32_t p_addr      = s1.p_addr;
  const LoadReq &req   = s1.load_req;

  // MMIO：直接读内存，不经过 cache
  if (is_mmio(p_addr)) {
    LoadResp &lr         = resp.load_resps[port];
    lr.valid             = true;
    lr.replay            = 0;
    lr.req_id            = req.req_id;
    lr.uop               = req.uop;
    lr.data              = read_mem_data(p_addr);
    lr.uop.difftest_skip = is_difftest_skip_addr(p_addr);
    lr.uop.is_cache_miss = false;
    return;
  }

  if (ctx) ctx->perf.dcache_access_num++;

  // ── S2 命中判断：使用 S1 快照（模拟 tag 阵列在 S1 已读出） ─────────
  int      hit_way  = -1;
  uint32_t req_tag  = (uint32_t)get_tag(p_addr);
  for (int w = 0; w < WAY_NUM; w++) {
    if (s1.vld_snap[w] && s1.tag_snap[w] == req_tag) {
      hit_way = w;
      break;
    }
  }

  if (hit_way >= 0) {
    // 命中：返回 cache_data 快照数据
    update_plru((uint32_t)get_index(p_addr), hit_way);
    LoadResp &lr         = resp.load_resps[port];
    lr.valid             = true;
    lr.replay            = 0;
    lr.req_id            = req.req_id;
    lr.uop               = req.uop;
    // 特殊地址（如 timer）通过 read_mem_data 获取实时值；普通地址用快照
    lr.data              = is_difftest_skip_addr(p_addr)
                             ? read_mem_data(p_addr)
                             : s1.data_snap[hit_way];
    lr.uop.difftest_skip = is_difftest_skip_addr(p_addr);
    lr.uop.is_cache_miss = false;
    return;
  }

  // 快照显示 miss；再检查当前状态（处理 MSHR 在两拍间完成填充的边角情况）
  hit_way = lookup(p_addr);
  if (hit_way >= 0) {
    update_plru((uint32_t)get_index(p_addr), hit_way);
    LoadResp &lr         = resp.load_resps[port];
    lr.valid             = true;
    lr.replay            = 0;
    lr.req_id            = req.req_id;
    lr.uop               = req.uop;
    lr.data              = is_difftest_skip_addr(p_addr)
                             ? read_mem_data(p_addr)
                             : cache_data[hit_way][get_index(p_addr)][get_word_off(p_addr)];
    lr.uop.difftest_skip = is_difftest_skip_addr(p_addr);
    lr.uop.is_cache_miss = false;
    return;
  }

  // ── 真正 Miss：分配 MSHR ──────────────────────────────────────────
  if (find_mshr_by_line(p_addr) >= 0) {
    // 同 cache line 已有 in-flight MSHR → conflict
    resp.load_resps[port].valid = false;
    return;
  }

  int free_slot = find_free_mshr();
  if (free_slot < 0) {
    // MSHR 全满 → replay=2
    LoadResp &lr  = resp.load_resps[port];
    lr.valid  = true;
    lr.replay = 2;
    lr.req_id = req.req_id;
    lr.uop    = req.uop;
    return;
  }

  // fill：更新 tag/valid，从 p_memory 填充 cache_data
  fill(p_addr);
  if (ctx) ctx->perf.dcache_miss_num++;
  int latency = MISS_LATENCY + rand() % 10;

  MshrEntry &m        = mshr[free_slot];
  m.valid             = true;
  m.resp_pending      = false;
  m.type              = 0;  // load
  m.addr              = p_addr;
  m.complete_time     = sim_time + latency;
  m.uop               = req.uop;
  m.uop.is_cache_miss = true;
  m.req_id            = req.req_id;

  resp.load_resps[port].valid = false;
}

// =============================================================
// S2 Store 端口处理
// =============================================================

void RealDcache::process_store_s2(int port, const PipeS1Entry &s1,
                                   DCacheRespPorts &resp) {
  uint32_t p_addr      = s1.p_addr;
  const StoreReq &req  = s1.store_req;

  // MMIO：直接写内存，不经过 cache
  if (is_mmio(p_addr)) {
    uint32_t old_val = p_memory[p_addr >> 2];
    uint32_t wmask   = 0;
    for (int b = 0; b < 4; b++) {
      if (req.strb & (1 << b))
        wmask |= (0xFFu << (b * 8));
    }
    uint32_t new_val = (old_val & ~wmask) | (req.data & wmask);
    p_memory[p_addr >> 2] = new_val;
    if (peripheral_model)
      peripheral_model->on_mem_store_effective(p_addr, new_val);
    resp.store_resps[port].valid  = true;
    resp.store_resps[port].replay = 0;
    resp.store_resps[port].req_id = req.req_id;
    return;
  }

  // ── S2 命中判断：使用 S1 快照 ────────────────────────────────────
  int      hit_way = -1;
  uint32_t req_tag = (uint32_t)get_tag(p_addr);
  for (int w = 0; w < WAY_NUM; w++) {
    if (s1.vld_snap[w] && s1.tag_snap[w] == req_tag) {
      hit_way = w;
      break;
    }
  }

  // 回退：检查当前状态（处理 MSHR 在两拍间完成填充的边角情况）
  if (hit_way < 0)
    hit_way = lookup(p_addr);

  if (hit_way >= 0) {
    // 命中：写 cache_data 和 p_memory
    int      idx      = get_index(p_addr);
    int      word_off = get_word_off(p_addr);
    uint32_t old_val  = cache_data[hit_way][idx][word_off];
    uint32_t wmask    = 0;
    for (int b = 0; b < 4; b++) {
      if (req.strb & (1 << b))
        wmask |= (0xFFu << (b * 8));
    }
    uint32_t new_val = (old_val & ~wmask) | (req.data & wmask);
    cache_data[hit_way][idx][word_off] = new_val;
    p_memory[p_addr >> 2] = new_val;
    if (peripheral_model)
      peripheral_model->on_mem_store_effective(p_addr, new_val);
    update_plru((uint32_t)idx, hit_way);

    resp.store_resps[port].valid  = true;
    resp.store_resps[port].replay = 0;
    resp.store_resps[port].req_id = req.req_id;
    return;
  }

  // ── Miss：写缓冲 + MSHR ──────────────────────────────────────────
  int free_wb = find_free_wb();
  if (free_wb < 0) {
    resp.store_resps[port].valid  = true;
    resp.store_resps[port].replay = 2;
    resp.store_resps[port].req_id = req.req_id;
    return;
  }

  int free_mshr = find_free_mshr();
  if (free_mshr < 0) {
    resp.store_resps[port].valid  = true;
    resp.store_resps[port].replay = 2;
    resp.store_resps[port].req_id = req.req_id;
    return;
  }

  // 写入写缓冲（延迟到 MSHR 完成时 drain 到 p_memory 和 cache_data）
  wb[free_wb].valid  = true;
  wb[free_wb].addr   = p_addr;
  wb[free_wb].data   = req.data;
  wb[free_wb].strb   = req.strb;
  wb[free_wb].req_id = req.req_id;

  // fill：将该 cache line 填入 tag/valid/cache_data（write-allocate）
  fill(p_addr);
  int latency = MISS_LATENCY + rand() % 10;

  MshrEntry &m    = mshr[free_mshr];
  m.valid         = true;
  m.resp_pending  = false;
  m.type          = 1;  // store
  m.addr          = p_addr;
  m.complete_time = sim_time + latency;
  m.uop           = {};
  m.uop.rob_idx   = (uint32_t)req.req_id;
  m.req_id        = req.req_id;

  resp.store_resps[port].valid  = true;
  resp.store_resps[port].replay = 1;  // MSHR 已分配
  resp.store_resps[port].req_id = req.req_id;
}

// =============================================================
// MSHR 完成扫描
// =============================================================

void RealDcache::drain_completed_mshr(DCacheRespPorts &resp) {
  bool any_freed = false;

  for (int i = 0; i < MSHR_NUM; i++) {
    MshrEntry &m = mshr[i];
    if (!m.valid) continue;

    if (!m.resp_pending && sim_time >= m.complete_time) {
      m.resp_pending = true;

      if (m.type == 1) {
        // Store：将写缓冲 drain 到 p_memory（并同步 cache_data）
        for (int j = 0; j < WB_NUM; j++) {
          if (wb[j].valid && wb[j].req_id == m.req_id) {
            drain_wb_entry(j);
            break;
          }
        }
      }
    }

    if (m.resp_pending && !resp.mshr_resp.valid) {
      if (m.type == 0) {
        // Load MSHR 完成：返回数据（来自 cache_data 或 read_mem_data）
        resp.mshr_resp.valid  = true;
        resp.mshr_resp.type   = 0;
        resp.mshr_resp.data   = read_mem_data(m.addr);
        resp.mshr_resp.uop    = m.uop;
        resp.mshr_resp.req_id = m.req_id;
        resp.mshr_resp.uop.is_cache_miss = true;
        resp.mshr_resp.uop.difftest_skip = is_difftest_skip_addr(m.addr);
      } else {
        // Store MSHR 完成
        resp.mshr_resp.valid  = true;
        resp.mshr_resp.type   = 1;
        resp.mshr_resp.uop    = m.uop;
        resp.mshr_resp.req_id = m.req_id;
      }
      m.valid        = false;
      m.resp_pending = false;
      any_freed      = true;
    }
  }

  resp.mshr_replay = any_freed && (find_free_mshr() >= 0);
}

// =============================================================
// comb()：主组合逻辑（2 级流水）
// =============================================================

void RealDcache::comb() {
  Assert(lsu2dcache != nullptr && "RealDcache: lsu2dcache not connected");
  Assert(dcache2lsu != nullptr && "RealDcache: dcache2lsu not connected");

  DCacheRespPorts &resp     = dcache2lsu->resp_ports;
  const DCacheReqPorts &req = lsu2dcache->req_ports;
  resp.clear();

  // 清空 S1 下一拍的流水线寄存器
  for (int i = 0; i < TOTAL_PORTS; i++)
    pipe_s1_next[i] = {};

  // ── Stage 2：处理上一周期 S1 快照，判断命中/缺失，驱动响应 ─────────
  for (int i = 0; i < LSU_LDU_COUNT; i++) {
    if (pipe_s1[i].valid)
      process_load_s2(i, pipe_s1[i], resp);
  }
  for (int i = 0; i < LSU_STA_COUNT; i++) {
    if (pipe_s1[LSU_LDU_COUNT + i].valid)
      process_store_s2(i, pipe_s1[LSU_LDU_COUNT + i], resp);
  }

  // ── Stage 1：接收新请求，读取 tag/data 阵列，写入 pipe_s1_next ────
  for (int i = 0; i < LSU_LDU_COUNT; i++) {
    const LoadReq &lr = req.load_ports[i];
    if (!lr.valid) continue;

    PipeS1Entry &next = pipe_s1_next[i];
    next.valid    = true;
    next.is_load  = true;
    next.port     = i;
    next.p_addr   = lr.addr;
    next.load_req = lr;

    int idx      = get_index(lr.addr);
    int word_off = get_word_off(lr.addr);
    for (int w = 0; w < WAY_NUM; w++) {
      next.tag_snap [w] = cache_tag  [w][idx];
      next.vld_snap [w] = cache_valid[w][idx];
      next.data_snap[w] = cache_data [w][idx][word_off];
    }
  }

  for (int i = 0; i < LSU_STA_COUNT; i++) {
    const StoreReq &sr = req.store_ports[i];
    if (!sr.valid) continue;

    PipeS1Entry &next  = pipe_s1_next[LSU_LDU_COUNT + i];
    next.valid     = true;
    next.is_load   = false;
    next.port      = i;
    next.p_addr    = sr.addr;
    next.store_req = sr;

    int idx      = get_index(sr.addr);
    int word_off = get_word_off(sr.addr);
    for (int w = 0; w < WAY_NUM; w++) {
      next.tag_snap [w] = cache_tag  [w][idx];
      next.vld_snap [w] = cache_valid[w][idx];
      next.data_snap[w] = cache_data [w][idx][word_off];
    }
  }

  // ── MSHR 完成扫描：发送 mshr_resp，设置 mshr_replay ───────────────
  drain_completed_mshr(resp);
}

// =============================================================
// seq()：时序逻辑——将 S1 采样结果锁存到 S2 流水线寄存器
// =============================================================

void RealDcache::seq() {
  for (int i = 0; i < TOTAL_PORTS; i++)
    pipe_s1[i] = pipe_s1_next[i];
}
