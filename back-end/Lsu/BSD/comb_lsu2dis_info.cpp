#include "comb_lsu2dis_info.h"

namespace RealLsuBsd {

void comb_lsu2dis_info(CombLsu2DisInfoInterface ifc) {
  const bool stq_tail_flag =
      (ifc.stq_count == STQ_SIZE ||
       (ifc.stq_count > 0 && ifc.stq_tail < ifc.stq_head))
          ? !ifc.stq_head_flag
          : ifc.stq_head_flag;
  ifc.out.lsu2dis->stq_tail = ifc.stq_tail;
  ifc.out.lsu2dis->stq_tail_flag = stq_tail_flag;

  const int visible_stq_free_raw = STQ_SIZE - ifc.stq_count - COMMIT_WIDTH;
  ifc.out.lsu2dis->stq_free =
      visible_stq_free_raw > 0 ? visible_stq_free_raw : 0;
  ifc.out.lsu2dis->ldq_free = LDQ_SIZE - ifc.ldq_count;

  for (auto &v : ifc.out.lsu2dis->ldq_alloc_idx) {
    v = -1;
  }
  for (auto &v : ifc.out.lsu2dis->ldq_alloc_valid) {
    v = false;
  }

  int scan_pos = ifc.ldq_alloc_tail;
  int produced = 0;
  for (int n = 0; n < LDQ_SIZE && produced < MAX_LDQ_DISPATCH_WIDTH; n++) {
    if (!ifc.ldq[scan_pos].valid) {
      ifc.out.lsu2dis->ldq_alloc_idx[produced] = scan_pos;
      ifc.out.lsu2dis->ldq_alloc_valid[produced] = true;
      produced++;
    }
    scan_pos = (scan_pos + 1) % LDQ_SIZE;
  }

  uint64_t mask = 0;
  for (int i = 0; i < LDQ_SIZE; i++) {
    const auto &entry = ifc.ldq[i];
    if (entry.valid && !entry.killed && entry.uop.tma.is_cache_miss) {
      mask |= (1ULL << entry.uop.rob_idx);
    }
  }
  ifc.out.lsu2rob->tma.miss_mask = mask;
  ifc.out.lsu2rob->committed_store_pending = ifc.committed_store_pending;
}

} // namespace RealLsuBsd
