#pragma once
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>
#include "../union_find/partially_persistent_disjoint_set_union.hpp"

namespace cp {
// Search nonnegative version IDs in [first,last). Predicate must be false then true.
// O(log(last-first+1)) observations, no mutation of the underlying history.
template<class Predicate>
std::optional<std::int64_t> first_true_version(std::int64_t first, std::int64_t last, Predicate predicate) {
    if (first < 0 || last < first) throw std::invalid_argument("invalid version range");
    const auto end = last;
    while (first < last) {
        const auto middle = first + (last - first) / 2;
        if (predicate(middle)) last = middle;
        else first = middle + 1;
    }
    return first == end ? std::nullopt : std::optional<std::int64_t>(first);
}
// Fixed-size, latest-only point assignments. Every successful set creates a version.
// get: O(log(updates at index+1)); set: amortized O(1); storage: O(n+updates).
template<class T> class PartiallyPersistentArray {
    std::vector<std::vector<std::pair<std::int64_t, T>>> history_;
    std::int64_t time_ = 0;
public:
    explicit PartiallyPersistentArray(const std::vector<T>& initial) {
        history_.reserve(initial.size());
        for (const auto& value : initial) history_.push_back({{0, value}});
    }
    PartiallyPersistentArray(std::size_t n, const T& value = T()) : PartiallyPersistentArray(std::vector<T>(n, value)) {}
    std::size_t size() const { return history_.size(); }
    std::int64_t time() const { return time_; }
    T get(std::int64_t time, std::size_t index) const {
        if (time < 0 || time > time_) throw std::out_of_range("unknown array version");
        const auto& history = history_.at(index);
        auto it = std::upper_bound(history.begin(), history.end(), time,
            [](std::int64_t t, const auto& entry) { return t < entry.first; });
        return std::prev(it)->second;
    }
    std::int64_t set(std::size_t index, const T& value) {
        auto& history = history_.at(index);
        if (time_ == std::numeric_limits<std::int64_t>::max() - 1) throw std::overflow_error("array version overflow");
        history.emplace_back(time_ + 1, value); return ++time_;
    }
};
inline std::optional<int> first_connected_version(const PartiallyPersistentDisjointSetUnion& dsu, int u, int v) {
    if (u < 0 || v < 0 || u >= dsu.vertex_count() || v >= dsu.vertex_count()) throw std::out_of_range("DSU vertex");
    auto answer = first_true_version(0, static_cast<std::int64_t>(dsu.time()) + 1,
        [&](std::int64_t time) { return dsu.same(static_cast<int>(time), u, v); });
    return answer ? std::optional<int>(static_cast<int>(*answer)) : std::nullopt;
}
} // namespace cp
