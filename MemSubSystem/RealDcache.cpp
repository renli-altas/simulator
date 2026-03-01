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
  memset(plru_tree,   0, sizeof(plru_tree));
  for (int i = 0; i < MSHR_NUM; i++) mshr[i] = {};
  for (int i = 0; i < WB_NUM;   i++) wb[i]   = {};
}

void RealDcache::init() {
  memset(cache_valid, 0, sizeof(cache_valid));
  memset(cache_tag,   0, sizeof(cache_tag));
  memset(plru_tree,   0, sizeof(plru_tree));
  for (int i = 0; i < MSHR_NUM; i++) mshr[i] = {};
  for (int i = 0; i < WB_NUM;   i++) wb[i]   = {};
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
// Cache 查询 / 填充 / 访问
// =============================================================

// 仅查标签，不更新状态。返回命中 way，-1 表示 miss。
int RealDcache::lookup(uint32_t addr) {
  int idx = get_index(addr);
  int tag = get_tag(addr);
  for (int w = 0; w < WAY_NUM; w++) {
    if (cache_valid[w][idx] && cache_tag[w][idx] == (uint32_t)tag)
      return w;
  }
  return -1;
}

// 选一路填充（tag/valid），返回被替换的 way。
int RealDcache::fill(uint32_t addr) {
  int way = select_evict_way(addr);
  int idx = get_index(addr);
  cache_tag  [way][idx] = (uint32_t)get_tag(addr);
  cache_valid[way][idx] = true;
  return way;
}

// 完整访问：命中返回 HIT_LATENCY，缺失填充并返回 MISS_LATENCY+jitter。
// MMIO（addr < 0x80000000）强制返回 1（不模拟 miss）。
int RealDcache::access(uint32_t addr) {
  if (ctx) ctx->perf.dcache_access_num++;
  if (is_mmio(addr)) return 1;

  int way = lookup(addr);
  if (way >= 0) {
    update_plru((uint32_t)get_index(addr), way);
    return HIT_LATENCY;
  }

  // Miss：立即填充 tag（模拟器中 tag 阵列总是同步的）
  fill(addr);
  if (ctx) ctx->perf.dcache_miss_num++;
  return MISS_LATENCY + rand() % 10;
}

// =============================================================
// MSHR 辅助
// =============================================================

int RealDcache::find_free_mshr() const {
  for (int i = 0; i < MSHR_NUM; i++)
    if (!mshr[i].valid) return i;
  return -1;
}

// 查找同 cache line（忽略 offset）是否已有 in-flight MSHR。
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
// Load 端口处理
// =============================================================

void RealDcache::handle_load_port(int port, const LoadReq &req,
                                  DCacheRespPorts &resp) {
  uint32_t p_addr = req.addr;
  int latency = access(p_addr);
  bool is_hit = (latency == HIT_LATENCY);

  if (is_hit) {
    // ── 命中：本周期返回数据 ───────────────────────────────
    LoadResp &lr = resp.load_resps[port];
    lr.valid  = true;
    lr.replay = 0;
    lr.req_id = req.req_id;
    lr.uop    = req.uop;
    lr.data   = read_mem_data(p_addr);
    lr.uop.difftest_skip  = is_difftest_skip_addr(p_addr);
    lr.uop.is_cache_miss  = false;
    return;
  }

  // ── Miss：尝试分配 MSHR ────────────────────────────────────
  // 若同 cache line 已有 in-flight MSHR，直接 conflict
  if (find_mshr_by_line(p_addr) >= 0) {
    LoadResp &lr = resp.load_resps[port];
    lr.valid  = true;
    lr.replay = 2;  // conflict：无空闲 MSHR slot，等待后重试
    lr.req_id = req.req_id;
    lr.uop    = req.uop;
    return;
  }

  int free_slot = find_free_mshr();
  if (free_slot < 0) {
    // MSHR 全满：conflict
    LoadResp &lr = resp.load_resps[port];
    lr.valid  = true;
    lr.replay = 2;
    lr.req_id = req.req_id;
    lr.uop    = req.uop;
    return;
  }

  // 分配 MSHR，本周期返回 replay=1
  MshrEntry &m       = mshr[free_slot];
  m.valid            = true;
  m.resp_pending     = false;
  m.type             = 0;  // load
  m.addr             = p_addr;
  m.complete_time    = sim_time + latency;
  m.uop              = req.uop;
  m.uop.is_cache_miss = true;
  m.req_id           = req.req_id;

  LoadResp &lr = resp.load_resps[port];
  lr.valid  = true;
  lr.replay = 1;  // MSHR 已分配，数据将经由 mshr_resp 回传
  lr.req_id = req.req_id;
  lr.uop    = req.uop;
  lr.uop.is_cache_miss = true;
}

// =============================================================
// Store 端口处理
// =============================================================

void RealDcache::handle_store_port(int port, const StoreReq &req,
                                   DCacheRespPorts &resp) {
  uint32_t p_addr = req.addr;
  int latency = access(p_addr);
  bool is_hit = (latency == HIT_LATENCY);

  if (is_hit) {
    // ── 命中：直接写内存，本周期 ack ──────────────────────────
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

  // ── Miss：写缓冲 + MSHR ────────────────────────────────────
  int free_wb = find_free_wb();
  if (free_wb < 0) {
    // 写缓冲满：conflict
    resp.store_resps[port].valid  = true;
    resp.store_resps[port].replay = 2;
    resp.store_resps[port].req_id = req.req_id;
    return;
  }

  int free_mshr = find_free_mshr();
  if (free_mshr < 0) {
    // MSHR 满：conflict
    resp.store_resps[port].valid  = true;
    resp.store_resps[port].replay = 2;
    resp.store_resps[port].req_id = req.req_id;
    return;
  }

  // 写入写缓冲（延迟到 MSHR 完成时 drain 到 p_memory）
  wb[free_wb].valid  = true;
  wb[free_wb].addr   = p_addr;
  wb[free_wb].data   = req.data;
  wb[free_wb].strb   = req.strb;
  wb[free_wb].req_id = req.req_id;

  // 分配 MSHR 跟踪完成时机（store 的 rob_idx 用 req_id 即 STQ index）
  MshrEntry &m     = mshr[free_mshr];
  m.valid          = true;
  m.resp_pending   = false;
  m.type           = 1;  // store
  m.addr           = p_addr;
  m.complete_time  = sim_time + latency;
  m.uop            = {};
  m.uop.rob_idx    = (uint32_t)req.req_id;  // STQ index（供 LSU mshr_resp 路径）
  m.req_id         = req.req_id;

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

    // 等待 fill 完成
    if (!m.resp_pending && sim_time >= m.complete_time) {
      m.resp_pending = true;

      if (m.type == 1) {
        // ── Store：从写缓冲 drain 到 p_memory ─────────────
        for (int j = 0; j < WB_NUM; j++) {
          if (wb[j].valid && wb[j].req_id == m.req_id) {
            drain_wb_entry(j);
            break;
          }
        }
      }
    }

    // 发送 mshr_resp（每周期只发一条，避免 resp 冲突）
    if (m.resp_pending && !resp.mshr_resp.valid) {
      if (m.type == 0) {
        // ── Load MSHR 完成 ───────────────────────────────
        resp.mshr_resp.valid  = true;
        resp.mshr_resp.type   = 0;
        resp.mshr_resp.data   = read_mem_data(m.addr);
        resp.mshr_resp.uop    = m.uop;
        resp.mshr_resp.req_id = m.req_id;
        resp.mshr_resp.uop.is_cache_miss  = true;
        resp.mshr_resp.uop.difftest_skip  = is_difftest_skip_addr(m.addr);
      } else {
        // ── Store MSHR 完成 ──────────────────────────────
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

  // mshr_replay：本周期有 MSHR 槽释放，且仍有空闲槽可接受重试
  resp.mshr_replay = any_freed && (find_free_mshr() >= 0);
}

// =============================================================
// comb()：主组合逻辑
// =============================================================

void RealDcache::comb() {
  Assert(lsu2dcache != nullptr && "RealDcache: lsu2dcache not connected");
  Assert(dcache2lsu != nullptr && "RealDcache: dcache2lsu not connected");

  DCacheRespPorts &resp       = dcache2lsu->resp_ports;
  const DCacheReqPorts &req   = lsu2dcache->req_ports;
  resp.clear();

  // 1. 处理 Load 请求（按端口顺序）
  for (int i = 0; i < LSU_LDU_COUNT; i++) {
    if (req.load_ports[i].valid)
      handle_load_port(i, req.load_ports[i], resp);
  }

  // 2. 处理 Store 请求（按端口顺序）
  for (int i = 0; i < LSU_STA_COUNT; i++) {
    if (req.store_ports[i].valid)
      handle_store_port(i, req.store_ports[i], resp);
  }

  // 3. MSHR 完成扫描 + mshr_resp + mshr_replay
  drain_completed_mshr(resp);
}

// =============================================================
// seq()：时序逻辑（状态已在 comb 中更新，seq 无额外寄存器）
// =============================================================

void RealDcache::seq() {}
