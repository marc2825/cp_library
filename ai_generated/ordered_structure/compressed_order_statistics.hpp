#pragma once
#include <algorithm>
#include <functional>
#include <limits>
#include <optional>
#include <utility>
#include <vector>
#include "coordinate_compression.hpp"
#include "../range_query/fenwick_tree.hpp"

// Multiset over a preregistered universe, with nonnegative long long counts.
// kth(k) is 0-based and returns optional<T>; absent/invalid mutations return false.
// Docs: docs/ordered_structure/compressed_order_statistics.md
namespace cp {

template <class T, class Compare = std::less<T>> class CompressedOrderStatistics {
  public:
    explicit CompressedOrderStatistics(std::vector<T> universe = {}, Compare compare = Compare{})
        : compression_(std::move(universe), std::move(compare)),
          counts_(compression_.size()), frequency_(compression_.size(), 0) {}
    long long size() const { return total_; }
    bool empty() const { return total_ == 0; }
    int universe_size() const { return compression_.size(); }
    const CoordinateCompression<T, Compare>& compression() const { return compression_; }
    bool insert(const T& x, long long count = 1) {
        if (count < 0 || count > std::numeric_limits<long long>::max() - total_) return false;
        auto i = compression_.index(x);
        if (!i) return false;
        counts_.add(*i, count);
        frequency_[*i] += count;
        total_ += count;
        return true;
    }
    bool erase(const T& x, long long count = 1) {
        if (count < 0) return false;
        auto i = compression_.index(x);
        if (!i || frequency_[*i] < count) return false;
        counts_.add(*i, -count);
        frequency_[*i] -= count;
        total_ -= count;
        return true;
    }
    long long erase_all(const T& x) {
        auto i = compression_.index(x);
        if (!i) return 0;
        long long count = frequency_[*i];
        counts_.add(*i, -count);
        frequency_[*i] = 0;
        total_ -= count;
        return count;
    }
    long long count(const T& x) const {
        auto i = compression_.index(x);
        return i ? frequency_[*i] : 0;
    }
    long long order_of_key(const T& x) const { return counts_.sum(compression_.lower_bound(x)); }
    long long count_less_equal(const T& x) const { return counts_.sum(compression_.upper_bound(x)); }
    long long count_range(const T& low, const T& high) const {
        int l = compression_.lower_bound(low), r = compression_.lower_bound(high);
        return l < r ? counts_.sum(l, r) : 0;
    }
    std::optional<T> kth(long long k) const {
        if (k < 0 || k >= total_) return std::nullopt;
        return compression_.value(counts_.lower_bound(k + 1));
    }
    std::optional<T> lower_bound(const T& x) const { return kth(order_of_key(x)); }
    std::optional<T> upper_bound(const T& x) const { return kth(count_less_equal(x)); }
    std::optional<T> predecessor(const T& x, bool inclusive = false) const {
        return kth((inclusive ? count_less_equal(x) : order_of_key(x)) - 1);
    }
    std::optional<T> successor(const T& x, bool inclusive = false) const {
        return inclusive ? lower_bound(x) : upper_bound(x);
    }
    void clear() {
        counts_ = FenwickTree<long long>(universe_size());
        std::fill(frequency_.begin(), frequency_.end(), 0);
        total_ = 0;
    }

  private:
    CoordinateCompression<T, Compare> compression_;
    FenwickTree<long long> counts_;
    std::vector<long long> frequency_;
    long long total_ = 0;
};

}  // namespace cp
