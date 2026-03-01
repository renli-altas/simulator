#pragma once

#include "AbstractDcache.h"
#include "IO.h"
#include "config.h"
#include "types.h"
#include <cstdint>
#include <cstring>

// ==========================================
// RealDcache: 组相联 DCache + MSHR + 写缓冲
// ==========================================
//
// 接口：使用多端口 LSU<->DCache 协议（LsuDcacheIO / DcacheLsuIO）
//
// 流水语义（每 comb() 周期）：
//   1. 处理 load_ports[]  → 命中返回 replay=0，缺失分配 MSHR 返回 replay=1，
//                          MSHR 满返回 replay=2（conflict）
//   2. 处理 store_ports[] → 命中立即写内存 replay=0，缺失进写缓冲+MSHR replay=1，
//                          写缓冲满返回 replay=2
//   3. 检查 MSHR 完成项   → 向 LSU 发送 mshr_resp，并置 mshr_replay
//
// MSHR 条目完成后经由 mshr_resp 直接把数据/完成信号传回 LSU；
// mshr_replay=true 表示本周期有 MSHR 槽释放，LSU 可把等待中的请求重发。

class SimContext;

class RealDcache : public AbstractDcache {
public:
  // Cache geometry（与 SimpleCache 对齐）
  static constexpr int OFFSET_WIDTH = 6;              // 64-byte cache line
  static constexpr int INDEX_WIDTH  = 8;              // 256 sets
  static constexpr int WAY_NUM      = 4;              // 4-way SA
  static constexpr int HIT_LATENCY  = 1;
  static constexpr int MISS_LATENCY = 50;

  // 结构参数
  static constexpr int MSHR_NUM = 16;
  static constexpr int WB_NUM   = 16;

  explicit RealDcache(SimContext *ctx);

  void init() override;
  void comb() override;
  void seq() override;

private:
  SimContext *ctx;

  // ── Cache Tag/Valid/PLRU 阵列 ──────────────────────────────
  uint32_t cache_tag  [WAY_NUM][1 << INDEX_WIDTH];
  bool     cache_valid[WAY_NUM][1 << INDEX_WIDTH];
  uint8_t  plru_tree  [1 << INDEX_WIDTH][(WAY_NUM - 1 + 7) / 8];

  // ── MSHR 表项 ─────────────────────────────────────────────
  struct MshrEntry {
    bool     valid         = false;
    bool     resp_pending  = false;  // fill done, mshr_resp not yet sent
    uint8_t  type          = 0;      // 0=load, 1=store
    uint32_t addr          = 0;      // 物理地址
    int64_t  complete_time = 0;      // 填充完成的仿真时间
    MicroOp  uop           = {};     // load: uop.rob_idx = LDQ index
    size_t   req_id        = 0;      // store: STQ index
  };
  MshrEntry mshr[MSHR_NUM];

  // ── 写缓冲 ────────────────────────────────────────────────
  struct WbEntry {
    bool     valid  = false;
    uint32_t addr   = 0;
    uint32_t data   = 0;
    uint8_t  strb   = 0;
    size_t   req_id = 0;   // STQ index（用于与 MSHR 条目关联）
  };
  WbEntry wb[WB_NUM];

  // ── 地址解析 ──────────────────────────────────────────────
  int get_tag  (uint32_t addr) const {
    return addr >> (OFFSET_WIDTH + INDEX_WIDTH);
  }
  int get_index(uint32_t addr) const {
    return (addr >> OFFSET_WIDTH) & ((1 << INDEX_WIDTH) - 1);
  }
  bool is_mmio (uint32_t addr) const { return addr < 0x80000000; }

  // ── PLRU ──────────────────────────────────────────────────
  bool get_plru_bit(uint8_t tree[], int bit_index) const;
  void set_plru_bit(uint8_t tree[], int bit_index, bool value);
  int  select_evict_way(uint32_t addr);
  void update_plru    (uint32_t index, int accessed_way);

  // ── Cache 查询/填充 ───────────────────────────────────────
  // 返回命中的 way；-1=miss（不修改 PLRU，不填充）
  int  lookup(uint32_t addr);
  // 填充：更新 tag/valid，返回被选中的 way
  int  fill  (uint32_t addr);
  // 完整访问：命中返回 HIT_LATENCY，缺失填充并返回 MISS_LATENCY+jitter
  int  access(uint32_t addr);

  // ── MSHR 辅助 ─────────────────────────────────────────────
  int  find_free_mshr() const;
  // 检查同 cache line 是否已有 in-flight MSHR（用于 merge 判断）
  int  find_mshr_by_line(uint32_t addr) const;

  // ── 写缓冲辅助 ────────────────────────────────────────────
  int  find_free_wb() const;
  // 将写缓冲某条目写入 p_memory（返回是否成功）
  void drain_wb_entry(int idx);

  // ── 特殊地址数据读取（MMIO timer 等）────────────────────────
  uint32_t read_mem_data(uint32_t p_addr) const;
  bool     is_difftest_skip_addr(uint32_t p_addr) const;

  // ── 端口处理 ──────────────────────────────────────────────
  void handle_load_port (int port, const LoadReq  &req, DCacheRespPorts &resp);
  void handle_store_port(int port, const StoreReq &req, DCacheRespPorts &resp);

  // ── MSHR 完成扫描（填充 mshr_resp，设置 mshr_replay）────────
  void drain_completed_mshr(DCacheRespPorts &resp);
};
