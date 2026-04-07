#pragma once

#include <cassert>
#include <cstdint>
#include <cstddef>
#include <new>
#include <type_traits>
#include <utility>

template <typename, typename = void> struct fifo_has_mask : std::false_type {};

template <typename T>
struct fifo_has_mask<T, std::void_t<decltype(std::declval<const T &>().mask)>>
    : std::true_type {};

template <typename, typename = void>
struct fifo_has_br_mask : std::false_type {};

template <typename T>
struct fifo_has_br_mask<T,
                        std::void_t<decltype(std::declval<const T &>().br_mask)>>
    : std::true_type {};

template <typename T, std::size_t Capacity> class FIFO {
  static_assert(Capacity > 0, "FIFO capacity must be greater than zero");

public:
  FIFO() = default;

  FIFO(const FIFO &other) { copy_from(other); }

  FIFO(FIFO &&other) noexcept(std::is_nothrow_move_constructible_v<T>) {
    move_from(std::move(other));
  }

  FIFO &operator=(const FIFO &other) {
    if (this != &other) {
      clear();
      copy_from(other);
    }
    return *this;
  }

  FIFO &operator=(FIFO &&other) noexcept(
      std::is_nothrow_move_constructible_v<T> &&
      std::is_nothrow_move_assignable_v<T>) {
    if (this != &other) {
      clear();
      move_from(std::move(other));
    }
    return *this;
  }

  ~FIFO() { clear(); }

  [[nodiscard]] static constexpr std::size_t capacity() noexcept {
    return Capacity;
  }

  [[nodiscard]] constexpr std::size_t size() const noexcept { return count; }

  [[nodiscard]] constexpr bool empty() const noexcept { return count == 0; }

  [[nodiscard]] constexpr bool full() const noexcept { return count == Capacity; }

  [[nodiscard]] constexpr std::size_t get_head() const noexcept {
    return head;
  }

  [[nodiscard]] constexpr std::size_t get_tail() const noexcept {
    return tail_index();
  }

  [[nodiscard]] constexpr bool is_valid(std::size_t idx) const noexcept {
    if (idx >= Capacity || empty()) {
      return false;
    }
    if (full()) {
      return true;
    }
    const std::size_t tail = tail_index();
    if (head < tail) {
      return idx >= head && idx < tail;
    }
    return idx >= head || idx < tail;
  }

  void fill_default() {
    clear();
    while (!full()) {
      const bool ok = emplace();
      assert(ok);
    }
  }

  void clear() noexcept {
    while (!empty()) {
      pop();
    }
  }

  template <typename... Args> bool emplace(Args &&...args) {
    if (full()) {
      return false;
    }

    new (&storage[tail_index()]) T(std::forward<Args>(args)...);
    ++count;
    return true;
  }

  bool push(const T &value) { return emplace(value); }

  bool push(T &&value) { return emplace(std::move(value)); }

  void pop() noexcept {
    assert(!empty());
    ptr(head)->~T();
    head = next_index(head);
    --count;
  }

  bool try_pop(T &value) {
    if (empty()) {
      return false;
    }

    value = std::move(front());
    pop();
    return true;
  }

  [[nodiscard]] T &front() noexcept {
    assert(!empty());
    return *ptr(head);
  }

  [[nodiscard]] const T &front() const noexcept {
    assert(!empty());
    return *ptr(head);
  }

  [[nodiscard]] T &back() noexcept {
    assert(!empty());
    return *ptr(prev_index(tail_index()));
  }

  [[nodiscard]] const T &back() const noexcept {
    assert(!empty());
    return *ptr(prev_index(tail_index()));
  }

  [[nodiscard]] T &operator[](std::size_t idx) noexcept {
    assert(idx < count);
    return *ptr(offset_index(idx));
  }

  [[nodiscard]] const T &operator[](std::size_t idx) const noexcept {
    assert(idx < count);
    return *ptr(offset_index(idx));
  }

  [[nodiscard]] T &slot(std::size_t idx) noexcept {
    assert(idx < Capacity);
    return *ptr(idx);
  }

  [[nodiscard]] const T &slot(std::size_t idx) const noexcept {
    assert(idx < Capacity);
    return *ptr(idx);
  }

protected:
  using Storage = std::aligned_storage_t<sizeof(T), alignof(T)>;

  Storage storage[Capacity];
  std::size_t head = 0;
  std::size_t count = 0;

  [[nodiscard]] static constexpr std::size_t next_index(std::size_t idx) noexcept {
    return (idx + 1 == Capacity) ? 0 : (idx + 1);
  }

  [[nodiscard]] static constexpr std::size_t prev_index(std::size_t idx) noexcept {
    return (idx == 0) ? (Capacity - 1) : (idx - 1);
  }

  [[nodiscard]] constexpr std::size_t tail_index() const noexcept {
    std::size_t tail = head + count;
    return (tail >= Capacity) ? (tail - Capacity) : tail;
  }

  [[nodiscard]] constexpr std::size_t offset_index(std::size_t idx) const noexcept {
    std::size_t physical = head + idx;
    return (physical >= Capacity) ? (physical - Capacity) : physical;
  }

  [[nodiscard]] T *ptr(std::size_t idx) noexcept {
    return std::launder(reinterpret_cast<T *>(&storage[idx]));
  }

  [[nodiscard]] const T *ptr(std::size_t idx) const noexcept {
    return std::launder(reinterpret_cast<const T *>(&storage[idx]));
  }

  void copy_from(const FIFO &other) {
    head = other.head;
    count = other.count;
    for (std::size_t i = 0; i < count; ++i) {
      const std::size_t idx = other.offset_index(i);
      new (&storage[idx]) T(*other.ptr(idx));
    }
  }

  void move_from(FIFO &&other) {
    head = other.head;
    count = other.count;
    for (std::size_t i = 0; i < count; ++i) {
      const std::size_t idx = other.offset_index(i);
      new (&storage[idx]) T(std::move(*other.ptr(idx)));
    }
    other.clear();
  }
};

template <typename T, std::size_t Capacity> class FIFO_FLAG {
  static_assert(Capacity > 0, "FIFO_FLAG capacity must be greater than zero");

public:
  FIFO_FLAG() = default;

  FIFO_FLAG(const FIFO_FLAG &other) { copy_from(other); }

  FIFO_FLAG(FIFO_FLAG &&other) noexcept(std::is_nothrow_move_constructible_v<T>) {
    move_from(std::move(other));
  }

  FIFO_FLAG &operator=(const FIFO_FLAG &other) {
    if (this != &other) {
      clear();
      copy_from(other);
    }
    return *this;
  }

  FIFO_FLAG &operator=(FIFO_FLAG &&other) noexcept(
      std::is_nothrow_move_constructible_v<T> &&
      std::is_nothrow_move_assignable_v<T>) {
    if (this != &other) {
      clear();
      move_from(std::move(other));
    }
    return *this;
  }

  ~FIFO_FLAG() { clear(); }

  [[nodiscard]] static constexpr std::size_t capacity() noexcept {
    return Capacity;
  }

  [[nodiscard]] constexpr std::size_t size() const noexcept { return count; }

  [[nodiscard]] constexpr bool empty() const noexcept { return count == 0; }

  [[nodiscard]] constexpr bool full() const noexcept { return count == Capacity; }

  [[nodiscard]] constexpr std::size_t get_head() const noexcept {
    return head;
  }

  [[nodiscard]] constexpr std::size_t get_tail() const noexcept {
    return tail_index();
  }

  [[nodiscard]] constexpr bool get_head_flag() const noexcept {
    return head_flag;
  }

  [[nodiscard]] constexpr bool get_tail_flag() const noexcept {
    return tail_wraps() ? !head_flag : head_flag;
  }

  [[nodiscard]] constexpr bool is_valid(std::size_t idx) const noexcept {
    if (idx >= Capacity || empty()) {
      return false;
    }
    if (full()) {
      return true;
    }
    const std::size_t tail = tail_index();
    if (head < tail) {
      return idx >= head && idx < tail;
    }
    return idx >= head || idx < tail;
  }

  void fill_default() {
    clear();
    while (!full()) {
      const bool ok = emplace();
      assert(ok);
    }
  }

  void clear() noexcept {
    while (!empty()) {
      pop();
    }
  }

  template <typename... Args> bool emplace(Args &&...args) {
    if (full()) {
      return false;
    }

    new (&storage[tail_index()]) T(std::forward<Args>(args)...);
    ++count;
    return true;
  }

  bool push(const T &value) { return emplace(value); }

  bool push(T &&value) { return emplace(std::move(value)); }

  void pop() noexcept {
    assert(!empty());
    ptr(head)->~T();
    head = next_index(head);
    if (head == 0) {
      head_flag = !head_flag;
    }
    --count;
  }

  bool try_pop(T &value) {
    if (empty()) {
      return false;
    }

    value = std::move(front());
    pop();
    return true;
  }

  [[nodiscard]] T &front() noexcept {
    assert(!empty());
    return *ptr(head);
  }

  [[nodiscard]] const T &front() const noexcept {
    assert(!empty());
    return *ptr(head);
  }

  [[nodiscard]] T &back() noexcept {
    assert(!empty());
    return *ptr(prev_index(tail_index()));
  }

  [[nodiscard]] const T &back() const noexcept {
    assert(!empty());
    return *ptr(prev_index(tail_index()));
  }

  [[nodiscard]] T &operator[](std::size_t idx) noexcept {
    assert(idx < count);
    return *ptr(offset_index(idx));
  }

  [[nodiscard]] const T &operator[](std::size_t idx) const noexcept {
    assert(idx < count);
    return *ptr(offset_index(idx));
  }

  [[nodiscard]] T &slot(std::size_t idx) noexcept {
    assert(idx < Capacity);
    return *ptr(idx);
  }

  [[nodiscard]] const T &slot(std::size_t idx) const noexcept {
    assert(idx < Capacity);
    return *ptr(idx);
  }

private:
  using Storage = std::aligned_storage_t<sizeof(T), alignof(T)>;

  Storage storage[Capacity];
  std::size_t head = 0;
  std::size_t count = 0;
  bool head_flag = false;

  [[nodiscard]] static constexpr std::size_t next_index(std::size_t idx) noexcept {
    return (idx + 1 == Capacity) ? 0 : (idx + 1);
  }

  [[nodiscard]] static constexpr std::size_t prev_index(std::size_t idx) noexcept {
    return (idx == 0) ? (Capacity - 1) : (idx - 1);
  }

  [[nodiscard]] constexpr bool tail_wraps() const noexcept {
    return head + count >= Capacity;
  }

  [[nodiscard]] constexpr std::size_t tail_index() const noexcept {
    std::size_t tail = head + count;
    return (tail >= Capacity) ? (tail - Capacity) : tail;
  }

  [[nodiscard]] constexpr std::size_t offset_index(std::size_t idx) const noexcept {
    std::size_t physical = head + idx;
    return (physical >= Capacity) ? (physical - Capacity) : physical;
  }

  [[nodiscard]] T *ptr(std::size_t idx) noexcept {
    return std::launder(reinterpret_cast<T *>(&storage[idx]));
  }

  [[nodiscard]] const T *ptr(std::size_t idx) const noexcept {
    return std::launder(reinterpret_cast<const T *>(&storage[idx]));
  }

  void copy_from(const FIFO_FLAG &other) {
    head = other.head;
    count = other.count;
    head_flag = other.head_flag;
    for (std::size_t i = 0; i < count; ++i) {
      const std::size_t idx = other.offset_index(i);
      new (&storage[idx]) T(*other.ptr(idx));
    }
  }

  void move_from(FIFO_FLAG &&other) {
    head = other.head;
    count = other.count;
    head_flag = other.head_flag;
    for (std::size_t i = 0; i < count; ++i) {
      const std::size_t idx = other.offset_index(i);
      new (&storage[idx]) T(std::move(*other.ptr(idx)));
    }
    other.clear();
  }
};

template <typename T, std::size_t Capacity>
void erase_after_mask(FIFO<T, Capacity> &queue, uint64_t input_mask) {
  FIFO<T, Capacity> kept;
  bool matched = false;

  const std::size_t n = queue.size();
  for (std::size_t i = 0; i < n; ++i) {
    T item = std::move(queue.front());
    queue.pop();

    if (!matched) {
      const bool ok = kept.push(std::move(item));
      assert(ok && "erase_after_mask overflow on FIFO");
      if constexpr (fifo_has_mask<T>::value) {
        if (kept.back().mask == input_mask) {
          matched = true;
        }
      } else if constexpr (fifo_has_br_mask<T>::value) {
        if ((kept.back().br_mask & input_mask) != 0) {
          matched = true;
        }
      } else {
        static_assert(fifo_has_mask<T>::value || fifo_has_br_mask<T>::value,
                      "erase_after_mask requires T to have mask or br_mask");
      }
    }
  }

  queue = std::move(kept);
}

template <typename T, std::size_t Capacity>
void erase_after_mask(FIFO_FLAG<T, Capacity> &queue, uint64_t input_mask) {
  FIFO_FLAG<T, Capacity> kept;
  bool matched = false;

  const std::size_t n = queue.size();
  for (std::size_t i = 0; i < n; ++i) {
    T item = std::move(queue.front());
    queue.pop();

    if (!matched) {
      const bool ok = kept.push(std::move(item));
      assert(ok && "erase_after_mask overflow on FIFO_FLAG");
      if constexpr (fifo_has_mask<T>::value) {
        if (kept.back().mask == input_mask) {
          matched = true;
        }
      } else if constexpr (fifo_has_br_mask<T>::value) {
        if ((kept.back().br_mask & input_mask) != 0) {
          matched = true;
        }
      } else {
        static_assert(fifo_has_mask<T>::value || fifo_has_br_mask<T>::value,
                      "erase_after_mask requires T to have mask or br_mask");
      }
    }
  }

  queue = std::move(kept);
}
