#include "WriteBuffer.h"

#include <cassert>
#include <cstddef>
#include <cstring>

namespace {

static constexpr uint8_t kCacheLineReqTotalSize =
    static_cast<uint8_t>(DCACHE_LINE_BYTES - 1u);

static constexpr uint64_t full_line_wstrb_mask() {
    uint64_t mask = 0;
    for (uint32_t i = 0; i < DCACHE_LINE_BYTES && i < 64; i++) {
        mask |= (1ull << i);
    }
    return mask;
}

static int find_wb_entry_in_view(const WriteBufferFifo &entries,
                                 uint32_t addr) {
    int best_match = -1;
    const uint32_t line_addr = (addr & ~(DCACHE_LINE_BYTES - 1));
    for (std::size_t i = 0; i < entries.size(); i++) {
        const WriteBufferEntry *entry = entries.at(i);
        if (entry != nullptr && entry->valid && entry->addr == line_addr) {
            best_match = static_cast<int>(i);
        }
    }
    return best_match;
}

static void clear_axi_req(WBOut &out) {
    out.axi_out.req_valid = false;
    out.axi_out.req_addr = 0;
    out.axi_out.req_total_size = 0;
    out.axi_out.req_id = 0;
    out.axi_out.req_wstrb = 0;
    for (int w = 0; w < DCACHE_LINE_WORDS; w++) {
        out.axi_out.req_wdata[w] = 0;
    }
}

static void clear_fifo_in_find_slot(FIFOIN &fifo_in, int port) {
    assert(port >= 0 && port < WB_FIND_PORTS && "find port out of range");
    fifo_in.writebuffer_find_valid[port] = false;
    fifo_in.writebuffer_find_index[port] = -1;
    fifo_in.writebuffer_find_entry[port] = {};
}

static void clear_all_fifo_in_find(FIFOIN &fifo_in) {
    for (int p = 0; p < WB_FIND_PORTS; p++) {
        clear_fifo_in_find_slot(fifo_in, p);
    }
}

static void clear_fifo_out_update_slot(FIFOOUT &fifo_out, int port) {
    assert(port >= 0 && port < WB_FIND_PORTS && "update port out of range");
    fifo_out.writebuffer_update_valid[port] = false;
    fifo_out.writebuffer_update_index[port] = -1;
    fifo_out.writebuffer_update_entry[port] = {};
}

static void reset_fifo_out_cmd(FIFOOUT &fifo_out) {
    fifo_out.pop = false;
    fifo_out.push_valid = false;
    fifo_out.push_entry = {};
    fifo_out.writebuffer_update_head = false;
    fifo_out.writebuffer_update_head_entry = {};
    for (int p = 0; p < WB_FIND_PORTS; p++) {
        fifo_out.writebuffer_find_req[p] = false;
        fifo_out.writebuffer_find_addr[p] = 0;
        clear_fifo_out_update_slot(fifo_out, p);
    }
}

static void refresh_fifo_in_basic(WBIn &in) {
    in.fifo_in.writebuffer_full = write_buffer.full();

    const WriteBufferEntry *head = write_buffer.front_ptr();
    if (head != nullptr) {
        in.fifo_in.writebuffer_head_valid = true;
        in.fifo_in.writebuffer_head = *head;
    } else {
        in.fifo_in.writebuffer_head_valid = false;
        in.fifo_in.writebuffer_head = {};
    }
}

static bool refresh_fifo_in_find_slot(WBIn &in, int port, uint32_t addr) {
    assert(port >= 0 && port < WB_FIND_PORTS && "find port out of range");

    const int idx = find_wb_entry_in_view(write_buffer, addr);
    if (idx < 0) {
        clear_fifo_in_find_slot(in.fifo_in, port);
        return false;
    }

    const WriteBufferEntry *entry =
        write_buffer.at(static_cast<std::size_t>(idx));
    if (entry == nullptr) {
        clear_fifo_in_find_slot(in.fifo_in, port);
        return false;
    }

    in.fifo_in.writebuffer_find_valid[port] = true;
    in.fifo_in.writebuffer_find_index[port] = idx;
    in.fifo_in.writebuffer_find_entry[port] = *entry;
    return true;
}

static void apply_fifo_update_port_cmd(WBOut &out, int update_port) {
    assert(update_port >= 0 && update_port < WB_FIND_PORTS &&
           "update port out of range");

    if (out.fifo_out.writebuffer_update_valid[update_port]) {
        assert(out.fifo_out.writebuffer_update_index[update_port] >= 0 &&
               "WriteBuffer update index is invalid");
        WriteBufferEntry *dst = write_buffer.at(
            static_cast<std::size_t>(out.fifo_out.writebuffer_update_index[update_port]));
        assert(dst != nullptr && "WriteBuffer update index out of range");
        *dst = out.fifo_out.writebuffer_update_entry[update_port];
    }
}

static void apply_fifo_update_head_cmd(WBOut &out) {
    if (out.fifo_out.writebuffer_update_head) {
        WriteBufferEntry *head = write_buffer.at(0);
        assert(head != nullptr && "WriteBuffer head update on empty FIFO");
        *head = out.fifo_out.writebuffer_update_head_entry;
    }
}

static void apply_fifo_push_cmd(WBOut &out) {
    if (out.fifo_out.push_valid) {
        assert(!write_buffer.full() && "WriteBuffer overflow");
        const bool pushed = write_buffer.push(out.fifo_out.push_entry);
        assert(pushed && "WriteBuffer push failed unexpectedly");
    }
}

static void apply_fifo_pop_cmd(WBOut &out) {
    if (out.fifo_out.pop) {
        WriteBufferEntry popped{};
        const bool popped_ok = write_buffer.pop(popped);
        assert(popped_ok && "WriteBuffer pop failed unexpectedly");
    }
}

static bool request_fifo_find(WBIn &in, WBOut &out, int port, uint32_t addr) {
    assert(port >= 0 && port < WB_FIND_PORTS && "find port out of range");
    out.fifo_out.writebuffer_find_req[port] = true;
    out.fifo_out.writebuffer_find_addr[port] = addr;
#if !CONFIG_BSD
    return refresh_fifo_in_find_slot(in, port, addr);
#else
    // On BSD, we cannot guarantee combinational read response from the FIFO,
    // so we just issue the find request and return false. The caller should
    // check the response in the next cycle.
    return in.fifo_in.writebuffer_find_valid[port];
#endif
}

static bool pending_push_matches(const WBState &state, uint32_t addr) {
    return state.pending_push_valid &&
           cache_line_match(state.pending_push_entry.addr, addr);
}

static void drive_axi_req_from_head(WBOut &out, uint32_t send,
                                    const FIFOIN &fifo_in) {
    clear_axi_req(out);
    out.axi_out.resp_ready = true;
    if (send != 0 || !fifo_in.writebuffer_head_valid) {
        return;
    }

    const WriteBufferEntry &head_e = fifo_in.writebuffer_head;
    if (!head_e.valid || head_e.send) {
        return;
    }

    out.axi_out.req_valid = true;
    out.axi_out.req_addr = head_e.addr;
    out.axi_out.req_total_size = kCacheLineReqTotalSize;
    out.axi_out.req_id = 0;
    out.axi_out.req_wstrb = full_line_wstrb_mask();
    for (int w = 0; w < DCACHE_LINE_WORDS; w++) {
        out.axi_out.req_wdata[w] = head_e.data[w];
    }
}

static void drive_axi_req_from_hold(WBOut &out, const WBState &cur) {
    clear_axi_req(out);
    out.axi_out.resp_ready = true;
    if (!cur.issue_pending) {
        return;
    }
    out.axi_out.req_valid = true;
    out.axi_out.req_addr = cur.issue_addr;
    out.axi_out.req_total_size = kCacheLineReqTotalSize;
    out.axi_out.req_id = 0;
    out.axi_out.req_wstrb = full_line_wstrb_mask();
    for (int w = 0; w < DCACHE_LINE_WORDS; w++) {
        out.axi_out.req_wdata[w] = cur.issue_data[w];
    }
}

} // namespace

int WriteBuffer::find_wb_entry(uint32_t addr) {
    return find_wb_entry_in_view(write_buffer, addr);
}

void WriteBuffer::init() {
    cur = {};
    nxt = {};
    write_buffer.clear();
    in.clear();
    out.clear();
    reset_fifo_out_cmd(out.fifo_out);
    refresh_fifo_in_basic(in);
    clear_all_fifo_in_find(in.fifo_in);
}

void WriteBuffer::comb_mshr_outputs() {
    // MSHR consumes this ready in mshr_.comb_inputs() after wb_.comb_inputs().
    // Use the post-comb WB view (`nxt`) so we don't advertise stale capacity
    // from the beginning of cycle.
    out.wbmshr.ready = !nxt.pending_push_valid;
}
void WriteBuffer::comb_outputs() {


    for (int i = 0; i < LSU_LDU_COUNT; i++) {
        out.wbdcache.bypass_resp[i].valid = nxt.bypassvalid[i];
        out.wbdcache.bypass_resp[i].data = nxt.bypassdata[i];
    }

    for (int i = 0; i < LSU_STA_COUNT; i++) {
        out.wbdcache.merge_resp[i].valid = nxt.mergevalid[i];
        out.wbdcache.merge_resp[i].busy = nxt.mergebusy[i];
    }
}

void WriteBuffer::comb_inputs() {
    reset_fifo_out_cmd(out.fifo_out);
    // Start from current sequential state, then override per-cycle outputs.
    // nxt.send = cur.send;
    // nxt.issue_pending = cur.issue_pending;
    // nxt.issue_addr = cur.issue_addr;
    // std::memcpy(nxt.issue_data, cur.issue_data, sizeof(nxt.issue_data));
    // nxt.pending_push_valid = cur.pending_push_valid;
    // nxt.pending_push_entry = cur.pending_push_entry;

    // Current-cycle B-consume eligibility for the head entry.
    const bool can_pop_head_now =
        cur.send && in.axi_in.resp_valid && in.fifo_in.writebuffer_head_valid &&
        in.fifo_in.writebuffer_head.valid && in.fifo_in.writebuffer_head.send;

    // Allow same-cycle pop+push so full-at-cycle-start does not cause false
    // overflow when the B channel is also retiring one entry this cycle.
    const bool can_push_now =
        (!in.fifo_in.writebuffer_full) || can_pop_head_now;

    // Local issue view of head payload. Merge updates to head are reflected
    // here so a newly latched issue-hold captures up-to-date bytes.
    FIFOIN fifo_issue_view = in.fifo_in;

    for (int i = 0; i < LSU_LDU_COUNT; i++) {
        nxt.bypassvalid[i] = false;
        nxt.bypassdata[i] = 0;
    }

    for (int i = 0; i < LSU_STA_COUNT; i++) {
        nxt.mergevalid[i] = false;
        nxt.mergebusy[i] = false;

        if (!in.dcachewb.merge_req[i].valid) {
            continue;
        }

        const auto &req = in.dcachewb.merge_req[i];
        const int find_port = i;
        if (request_fifo_find(in, out, find_port, req.addr)) {
            WriteBufferEntry entry = in.fifo_in.writebuffer_find_entry[find_port];
            const int wb_idx = in.fifo_in.writebuffer_find_index[find_port];
            const bool head_issue_frozen =
                (wb_idx == 0) &&
                (cur.issue_pending || cur.send);
            if (entry.send || head_issue_frozen) {
                nxt.mergebusy[i] = true;
            } else {
                nxt.mergevalid[i] = true;
                const uint32_t word_off = decode(req.addr).word_off;
                apply_strobe(entry.data[word_off], req.data, req.strb);
                out.fifo_out.writebuffer_update_valid[find_port] = true;
                out.fifo_out.writebuffer_update_index[find_port] = wb_idx;
                out.fifo_out.writebuffer_update_entry[find_port] = entry;
            #if !CONFIG_BSD
                if (wb_idx == 0) {
                    fifo_issue_view.writebuffer_head_valid = true;
                    fifo_issue_view.writebuffer_head = entry;
                }
                apply_fifo_update_port_cmd(out, find_port);
            #else
                // On BSD, find/update returns one cycle later. Keep the local
                // issue view conservative and do not speculate head bytes.
            #endif
            }
        } else if (pending_push_matches(cur, req.addr)) {
            nxt.mergevalid[i] = true;
            const uint32_t word_off = decode(req.addr).word_off;
            apply_strobe(nxt.pending_push_entry.data[word_off], req.data,
                         req.strb);
        } else if (in.mshrwb.valid && cache_line_match(req.addr, in.mshrwb.addr)) {
            nxt.mergevalid[i] = true;
            const uint32_t word_off = decode(req.addr).word_off;
            apply_strobe(in.mshrwb.data[word_off], req.data, req.strb);
        }
    }

    WriteBufferEntry incoming_push{};
    const bool has_incoming_push = in.mshrwb.valid;
    if (has_incoming_push) {
        incoming_push.valid = true;
        incoming_push.send = false;
        incoming_push.addr = in.mshrwb.addr;
        std::memcpy(incoming_push.data, in.mshrwb.data,
                    DCACHE_LINE_WORDS * sizeof(uint32_t));
    }

    // Priority: drain pending skid entry first.
    if (cur.pending_push_valid) {
        if (can_push_now) {
            out.fifo_out.push_valid = true;
            out.fifo_out.push_entry = cur.pending_push_entry;
            nxt.pending_push_valid = false;
            nxt.pending_push_entry = {};
        }
        if (has_incoming_push) {
            // Refill skid buffer with the new incoming eviction.
            if (nxt.pending_push_valid) {
                // Two unresolved pushes in a 1-entry skid means backpressure
                // protocol is violated by upstream scheduling.
                assert(false && "WriteBuffer pending-push overflow");
            }
            nxt.pending_push_valid = true;
            nxt.pending_push_entry = incoming_push;
        }
    } else if (has_incoming_push) {
        if (can_push_now) {
            out.fifo_out.push_valid = true;
            out.fifo_out.push_entry = incoming_push;
        } else {
            nxt.pending_push_valid = true;
            nxt.pending_push_entry = incoming_push;
        }
    }

    for (int i = 0; i < LSU_LDU_COUNT; i++) {
        if (!in.dcachewb.bypass_req[i].valid) {
            continue;
        }

        const uint32_t addr = in.dcachewb.bypass_req[i].addr;
        const uint32_t word_off = decode(addr).word_off;
        const int find_port = LSU_STA_COUNT + i;
        if (request_fifo_find(in, out, find_port, addr)) {
            nxt.bypassvalid[i] = true;
            nxt.bypassdata[i] =
                in.fifo_in.writebuffer_find_entry[find_port].data[word_off];
        } else if (pending_push_matches(cur, addr)) {
            nxt.bypassvalid[i] = true;
            nxt.bypassdata[i] = cur.pending_push_entry.data[word_off];
        } else if (in.mshrwb.valid && cache_line_match(addr, in.mshrwb.addr)) {
            nxt.bypassvalid[i] = true;
            nxt.bypassdata[i] = in.mshrwb.data[word_off];
        }
    }

    // Drive request either from held payload (stable until accepted) or from
    // current head if no hold exists yet.
    if (!cur.send && cur.issue_pending) {
        drive_axi_req_from_hold(out, cur);
    } else {
        drive_axi_req_from_head(out, cur.send, fifo_issue_view);
    }

    if (!cur.send) {
        const bool req_payload_matches_hold =
            (!cur.issue_pending) ||
            (out.axi_out.req_addr == cur.issue_addr);
        const bool req_handshake =
            in.axi_in.req_accepted && out.axi_out.req_valid &&
            req_payload_matches_hold;

        // When not in hold yet, latch the payload we are trying to issue.
        if (!cur.issue_pending && out.axi_out.req_valid) {
            nxt.issue_pending = true;
            nxt.issue_addr = out.axi_out.req_addr;
            std::memcpy(nxt.issue_data, out.axi_out.req_wdata,
                        sizeof(nxt.issue_data));
        }

        if (req_handshake) {
            WriteBufferEntry updated = fifo_issue_view.writebuffer_head;
            updated.send = true;
            out.fifo_out.writebuffer_update_head = true;
            out.fifo_out.writebuffer_update_head_entry = updated;
            nxt.send = true;
            nxt.issue_pending = false;
            nxt.issue_addr = 0;
            std::memset(nxt.issue_data, 0, sizeof(nxt.issue_data));
        } else if (!out.axi_out.req_valid) {
            // No request to issue, drop stale hold.
            nxt.issue_pending = false;
            nxt.issue_addr = 0;
            std::memset(nxt.issue_data, 0, sizeof(nxt.issue_data));
        }
    } else {
        // In-flight mode never needs an issue-hold.
        nxt.issue_pending = false;
        nxt.issue_addr = 0;
        std::memset(nxt.issue_data, 0, sizeof(nxt.issue_data));
    }

    if (can_pop_head_now) {
        out.fifo_out.pop = true;
        nxt.send = false;
        nxt.issue_pending = false;
        nxt.issue_addr = 0;
        std::memset(nxt.issue_data, 0, sizeof(nxt.issue_data));
    }

}

void WriteBuffer::seq() {
    cur = nxt;
    nxt = cur;
    apply_fifo_update_head_cmd(out);
    if (out.fifo_out.pop) {
        apply_fifo_pop_cmd(out);
    }
    if (out.fifo_out.push_valid) {
        apply_fifo_push_cmd(out);
    }
    refresh_fifo_in_basic(in);
    clear_all_fifo_in_find(in.fifo_in);
}
