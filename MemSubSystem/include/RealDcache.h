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
// 流水线语义（2 级流水）：
//   S1（第 1 周期）: 接收请求，读取 tag/valid/data 阵列，快照到 pipe_s1_next[]
//   S2（第 2 周期）: 从快照比较 tag 判断命中；
//     - 命中 Load  : 返回 cache_data（快照）
//     - 命中 Store : 写 cache_data + p_memory，返回 ack
//     - 缺失       : 调用 fill() 填充 cache_data，分配 MSHR / 写缓冲
//   MSHR 完成扫描 : 向 LSU 发送 mshr_resp，置 mshr_replay
//
// cache_data 数组大小可通过 DATA_WORDS_PER_LINE 在编译期调整（默认与
// cache line 宽度对齐：WORDS_PER_LINE = (1<<OFFSET_WIDTH)/4 = 16）。

class SimContext;

class RealDcache : public AbstractDcache {
public:
  // Cache geometry
  static constexpr int OFFSET_WIDTH    = 6;   // 64-byte cache line
  static constexpr int INDEX_WIDTH     = 8;   // 256 sets
  static constexpr int WAY_NUM         = 4;   // 4-way SA
  static constexpr int HIT_LATENCY     = 1;
  static constexpr int MISS_LATENCY    = 50;

  // cache_data 每路每组存放的 32-bit word 数（可配置，默认与 cache line 对齐）
  static constexpr int WORDS_PER_LINE  = (1 << OFFSET_WIDTH) / 4;  // 16

  // 结构参数
  static constexpr int MSHR_NUM        = 16;
  static constexpr int WB_NUM          = 16;

  // S1->S2 流水线总槽数（每个 load/store 端口各占一个槽）
  static constexpr int TOTAL_PORTS     = LSU_LDU_COUNT + LSU_STA_COUNT;

  explicit RealDcache(SimContext *ctx);

  void init() override;
  void comb() override;
  void seq()  override;

private:
  SimContext *ctx;

  // ── Cache Tag / Valid / Data 阵列 ──────────────────────────────────
  uint32_t cache_tag  [WAY_NUM][1 << INDEX_WIDTH];
  bool     cache_valid[WAY_NUM][1 << INDEX_WIDTH];
  uint32_t cache_data [WAY_NUM][1 << INDEX_WIDTH][WORDS_PER_LINE];
  uint8_t  plru_tree  [1 << INDEX_WIDTH][(WAY_NUM - 1 + 7) / 8];

  // ── S1→S2 流水线寄存器 ─────────────────────────────────────────────
  struct PipeS1Entry {
    bool     valid     = false;
    bool     is_load   = false;
    int      port      = 0;
    uint32_t p_addr    = 0;
    LoadReq  load_req  = {};
    StoreReq store_req = {};
    // S1 阵列快照（所有 way）
    uint32_t tag_snap [WAY_NUM] = {};
    bool     vld_snap [WAY_NUM] = {};
    uint32_t data_snap[WAY_NUM] = {};  // 对应地址偏移处的 32-bit word
  };

  PipeS1Entry pipe_s1     [TOTAL_PORTS];   // 当前 S2 使用
  PipeS1Entry pipe_s1_next[TOTAL_PORTS];   // 由 seq() 锁存到 pipe_s1

  // ── MSHR 表项 ─────────────────────────────────────────────────────
  struct MshrEntry {
    bool     valid         = false;
    bool     resp_pending  = false;
    uint8_t  type          = 0;      // 0=load, 1=store
    uint32_t addr          = 0;
    int64_t  complete_time = 0;
    MicroOp  uop           = {};
    size_t   req_id        = 0;
  };
  MshrEntry mshr[MSHR_NUM];

  // ── 写缓冲 ────────────────────────────────────────────────────────
  struct WbEntry {
    bool     valid  = false;
    uint32_t addr   = 0;
    uint32_t data   = 0;
    uint8_t  strb   = 0;
    size_t   req_id = 0;
  };
  WbEntry wb[WB_NUM];

  // ── 地址解析 ──────────────────────────────────────────────────────
  int get_tag     (uint32_t addr) const {
    return addr >> (OFFSET_WIDTH + INDEX_WIDTH);
  }
  int get_index   (uint32_t addr) const {
    return (addr >> OFFSET_WIDTH) & ((1 << INDEX_WIDTH) - 1);
  }
  int get_word_off(uint32_t addr) const {
    return (addr & ((1 << OFFSET_WIDTH) - 1)) >> 2;
  }
  bool is_mmio(uint32_t addr) const {
    return ((addr & UART_ADDR_MASK) == UART_ADDR_BASE) ||
           ((addr & PLIC_ADDR_MASK) == PLIC_ADDR_BASE);
  }

  // ── PLRU ──────────────────────────────────────────────────────────
  bool get_plru_bit(uint8_t tree[], int bit_index) const;
  void set_plru_bit(uint8_t tree[], int bit_index, bool value);
  int  select_evict_way(uint32_t addr);
  void update_plru    (uint32_t index, int accessed_way);

  // ── Cache 查询 / 填充 ─────────────────────────────────────────────
  // 仅比较当前状态，返回命中 way；-1=miss（不更新 PLRU）
  int  lookup(uint32_t addr);
  // 选一路替换：更新 tag/valid，从 p_memory 填充 cache_data，返回被选 way
  int  fill  (uint32_t addr);

  // ── MSHR 辅助 ─────────────────────────────────────────────────────
  int  find_free_mshr   () const;
  int  find_mshr_by_line(uint32_t addr) const;

  // ── 写缓冲辅助 ────────────────────────────────────────────────────
  int  find_free_wb   () const;
  // 将 WB 条目写入 p_memory 和 cache_data（如果该行仍在 cache 中）
  void drain_wb_entry(int idx);

  // ── 特殊地址数据读取 ──────────────────────────────────────────────
  uint32_t read_mem_data       (uint32_t p_addr) const;
  bool     is_difftest_skip_addr(uint32_t p_addr) const;

  // ── S2 端口处理（由 comb() 调用，处理上一周期 S1 快照）────────────
  void process_load_s2 (int port, const PipeS1Entry &s1, DCacheRespPorts &resp);
  void process_store_s2(int port, const PipeS1Entry &s1, DCacheRespPorts &resp);

  // ── MSHR 完成扫描 ─────────────────────────────────────────────────
  void drain_completed_mshr(DCacheRespPorts &resp);
};
