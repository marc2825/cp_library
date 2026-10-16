#pragma once
#include <cassert>
#include <deque>
#include <functional>
#include <limits>
#include <utility>

// FIFO min/max queue; less<T> means minimum. Equal best values keep the oldest index.
// pop() removes the oldest LOGICAL element, not necessarily a retained candidate.
// Docs: docs/static_query/monotonic_queue.md
namespace cp {

template <class T, class Compare = std::less<T>> class MonotonicQueue {
  public:
    struct Entry { int index; T value; };
    explicit MonotonicQueue(Compare compare = Compare{}) : compare_(std::move(compare)) {}
    void push(T value) {
        assert(right_ < std::numeric_limits<int>::max());
        while (!data_.empty() && compare_(value, data_.back().value)) data_.pop_back();
        data_.push_back({right_++, std::move(value)});
    }
    void pop() {
        assert(!empty());
        if (data_.front().index == left_) data_.pop_front();
        ++left_;
    }
    int size() const { return right_ - left_; } // Logical FIFO length.
    bool empty() const { return left_ == right_; }
    const T& best() const { assert(!empty()); return data_.front().value; }
    int best_index() const { assert(!empty()); return data_.front().index; }
    const std::deque<Entry>& candidates() const { return data_; }
    void clear() { data_.clear(); left_ = right_ = 0; }

  private:
    Compare compare_;
    int left_ = 0, right_ = 0;
    std::deque<Entry> data_;
};

}  // namespace cp
