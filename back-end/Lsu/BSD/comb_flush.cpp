#include "comb_flush.h"

namespace RealLsuBsd {

void comb_flush(CombFlushInterface ifc) {
  if (!ifc.in.rob_bcast->flush) {
    return;
  }

  for (int i = 0; i < LDQ_SIZE; i++) {
    if (!ifc.ldq[i].valid) {
      continue;
    }
    if (ifc.ldq[i].sent) {
      ifc.ldq[i].killed = true;
    } else {
      ifc.free_ldq_entry(i);
    }
  }
  ifc.finished_loads.clear();
  ifc.finished_sta_reqs.clear();
  ifc.pending_sta_addr_reqs.clear();
}

} // namespace RealLsuBsd
