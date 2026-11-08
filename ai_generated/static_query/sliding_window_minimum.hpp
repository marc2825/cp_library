#pragma once
#include <cassert>
#include <functional>
#include <limits>
#include <vector>
#include "monotonic_queue.hpp"

// Minimum / earliest argmin of every length-k window. k>0; k>n returns empty.
// Pass greater<T> for maxima. Linear time and O(k) auxiliary memory.
// Docs: docs/static_query/sliding_window_minimum.md
namespace cp {

template <class T, class Compare = std::less<T>>
std::vector<T> sliding_window_minimum(const std::vector<T>& a, int k,
                                     Compare compare = Compare{}) {
    assert(k > 0);
    assert(a.size() <= static_cast<std::size_t>(std::numeric_limits<int>::max()));
    std::vector<T> result;
    if (a.size() < static_cast<std::size_t>(k)) return result;
    result.reserve(a.size() - k + 1);
    MonotonicQueue<T, Compare> queue(compare);
    for (const T& x : a) {
        if (queue.size() == k) queue.pop();
        queue.push(x);
        if (queue.size() == k) result.push_back(queue.best());
    }
    return result;
}

template <class T, class Compare = std::less<T>>
std::vector<int> sliding_window_argmin(const std::vector<T>& a, int k,
                                      Compare compare = Compare{}) {
    assert(k > 0);
    assert(a.size() <= static_cast<std::size_t>(std::numeric_limits<int>::max()));
    std::vector<int> result;
    if (a.size() < static_cast<std::size_t>(k)) return result;
    result.reserve(a.size() - k + 1);
    MonotonicQueue<T, Compare> queue(compare);
    for (const T& x : a) {
        if (queue.size() == k) queue.pop();
        queue.push(x);
        if (queue.size() == k) result.push_back(queue.best_index());
    }
    return result;
}

}  // namespace cp
