#pragma once

#include <cstddef>
#include <utility>
#include <vector>

template <typename T, typename Ignored = void>
class FIFO {
public:
    explicit FIFO(std::size_t capacity = 0)
        : capacity_(capacity), data_(capacity) {}

    bool push(const T &value) {
        if (full()) {
            return false;
        }
        data_[physical_index(size_)] = value;
        size_++;
        return true;
    }

    bool push(T &&value) {
        if (full()) {
            return false;
        }
        data_[physical_index(size_)] = std::move(value);
        size_++;
        return true;
    }

    template <typename... Args>
    bool emplace(Args &&...args) {
        if (full()) {
            return false;
        }
        data_[physical_index(size_)] = T(std::forward<Args>(args)...);
        size_++;
        return true;
    }

    bool pop() {
        if (empty()) {
            return false;
        }
        advance_head();
        return true;
    }

    bool pop(T &out) {
        if (empty()) {
            return false;
        }
        out = std::move(data_[head_]);
        advance_head();
        return true;
    }

    void clear() {
        head_ = 0;
        size_ = 0;
    }

    bool empty() const {
        return size_ == 0;
    }

    bool full() const {
        return size_ >= capacity_;
    }

    std::size_t size() const {
        return size_;
    }

    std::size_t capacity() const {
        return capacity_;
    }

    std::size_t free_slots() const {
        return (capacity_ > size_) ? (capacity_ - size_) : 0;
    }

    T *front_ptr() {
        if (empty()) {
            return nullptr;
        }
        return &data_[head_];
    }

    const T *front_ptr() const {
        if (empty()) {
            return nullptr;
        }
        return &data_[head_];
    }

    T *back_ptr() {
        if (empty()) {
            return nullptr;
        }
        return &data_[physical_index(size_ - 1)];
    }

    const T *back_ptr() const {
        if (empty()) {
            return nullptr;
        }
        return &data_[physical_index(size_ - 1)];
    }

    T *at(std::size_t index) {
        if (index >= size_) {
            return nullptr;
        }
        return &data_[physical_index(index)];
    }

    const T *at(std::size_t index) const {
        if (index >= size_) {
            return nullptr;
        }
        return &data_[physical_index(index)];
    }

    bool update_at(std::size_t index, const T &value) {
        T *slot = at(index);
        if (slot == nullptr) {
            return false;
        }
        *slot = value;
        return true;
    }

    bool update_at(std::size_t index, T &&value) {
        T *slot = at(index);
        if (slot == nullptr) {
            return false;
        }
        *slot = std::move(value);
        return true;
    }

    template <typename Pred>
    T *find_if(Pred pred) {
        for (std::size_t i = 0; i < size_; i++) {
            T *item = at(i);
            if (item != nullptr && pred(*item)) {
                return item;
            }
        }
        return nullptr;
    }

    template <typename Pred>
    const T *find_if(Pred pred) const {
        for (std::size_t i = 0; i < size_; i++) {
            const T *item = at(i);
            if (item != nullptr && pred(*item)) {
                return item;
            }
        }
        return nullptr;
    }

    template <typename Pred, typename Mutator>
    bool modify_if(Pred pred, Mutator mutator) {
        for (std::size_t i = 0; i < size_; i++) {
            T *item = at(i);
            if (item != nullptr && pred(*item)) {
                mutator(*item);
                return true;
            }
        }
        return false;
    }

    template <typename Func>
    void for_each(Func func) {
        for (std::size_t i = 0; i < size_; i++) {
            T *item = at(i);
            if (item != nullptr) {
                func(*item);
            }
        }
    }

    template <typename Func>
    void for_each(Func func) const {
        for (std::size_t i = 0; i < size_; i++) {
            const T *item = at(i);
            if (item != nullptr) {
                func(*item);
            }
        }
    }

private:
    std::size_t physical_index(std::size_t logical_index) const {
        if (capacity_ == 0) {
            return 0;
        }
        return (head_ + logical_index) % capacity_;
    }

    void advance_head() {
        size_--;
        if (size_ == 0 || capacity_ == 0) {
            head_ = 0;
            return;
        }
        head_ = (head_ + 1) % capacity_;
    }

    std::size_t capacity_ = 0;
    std::vector<T> data_;
    std::size_t head_ = 0;
    std::size_t size_ = 0;
};

