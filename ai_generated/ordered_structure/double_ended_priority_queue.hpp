#pragma once
#include <cassert>
#include <cstddef>
#include <functional>
#include <iterator>
#include <set>
#include <utility>

namespace cp {
// 両端を O(1) で参照する多重集合。順序は Compare に従う。
template<class T, class Compare = std::less<T>> class DoubleEndedPriorityQueue {
    std::multiset<T, Compare> values_;
public:
    explicit DoubleEndedPriorityQueue(Compare compare = Compare{}) : values_(compare) {}
    template<class It> DoubleEndedPriorityQueue(It first, It last, Compare compare = Compare{})
        : values_(first, last, compare) {}
    bool empty() const { return values_.empty(); }
    std::size_t size() const { return values_.size(); }
    const T& min() const { assert(!empty()); return *values_.begin(); }
    const T& max() const { assert(!empty()); return *values_.rbegin(); }
    void push(T value) { values_.insert(std::move(value)); }
    void pop_min() { assert(!empty()); values_.erase(values_.begin()); }
    void pop_max() { assert(!empty()); values_.erase(std::prev(values_.end())); }
    void clear() { values_.clear(); }
};
} // namespace cp
