// RealLsu class fragment: finished LSU writeback queues.

class RealLsu;

template <int Capacity> struct MicroOpFifo {
  std::array<MicroOp, Capacity> entries{};
  int head = 0;
  int tail = 0;
  int count = 0;

  void clear() {
    head = 0;
    tail = 0;
    count = 0;
  }

  bool empty() const { return count == 0; }

  bool push_back(const MicroOp &uop) {
    if (count >= Capacity) {
      return false;
    }
    entries[tail] = uop;
    tail = (tail + 1) % Capacity;
    count++;
    return true;
  }

  bool pop_front(MicroOp &uop) {
    if (count == 0) {
      return false;
    }
    uop = entries[head];
    head = (head + 1) % Capacity;
    count--;
    if (count == 0) {
      head = 0;
      tail = 0;
    }
    return true;
  }

  template <typename Fn> void for_each_mut(Fn &&fn) {
    int idx = head;
    for (int i = 0; i < count; i++) {
      fn(entries[idx]);
      idx = (idx + 1) % Capacity;
    }
  }

  template <typename Pred> void remove_if(Pred &&pred) {
    const int old_count = count;
    int read = head;
    int write = head;
    int kept = 0;
    for (int i = 0; i < old_count; i++) {
      MicroOp item = entries[read];
      read = (read + 1) % Capacity;
      if (pred(item)) {
        continue;
      }
      entries[write] = item;
      write = (write + 1) % Capacity;
      kept++;
    }
    count = kept;
    if (kept == 0) {
      head = 0;
      tail = 0;
    } else {
      tail = (head + kept) % Capacity;
    }
  }
};

inline constexpr int kFinishedLoadFifoCapacity = LDQ_SIZE + STQ_SIZE;
inline constexpr int kFinishedStaFifoCapacity = STQ_SIZE;

class FinishedState {

private:
  MicroOpFifo<kFinishedLoadFifoCapacity> finished_loads{};
  MicroOpFifo<kFinishedStaFifoCapacity> finished_sta_reqs{};
};
