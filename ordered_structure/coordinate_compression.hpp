#pragma once
#include <algorithm>
#include <cassert>
#include <functional>
#include <optional>
#include <utility>
#include <vector>

// Immutable sorted unique coordinates. Equality is comparator equivalence, not ==.
// index(x) is optional; lower_bound/upper_bound also accept unregistered thresholds.
// Docs: docs/ordered_structure/coordinate_compression.md
namespace cp {

template <class T, class Compare = std::less<T>> class CoordinateCompression {
  public:
    explicit CoordinateCompression(std::vector<T> values = {}, Compare compare = Compare{})
        : values_(std::move(values)), compare_(std::move(compare)) {
        assert(values_.size() <= 100000000);
        std::sort(values_.begin(), values_.end(), compare_);
        values_.erase(std::unique(values_.begin(), values_.end(), [this](const T& x, const T& y) {
            return !compare_(x, y) && !compare_(y, x);
        }), values_.end());
    }
    int size() const { return int(values_.size()); }
    bool empty() const { return values_.empty(); }
    int lower_bound(const T& x) const {
        return int(std::lower_bound(values_.begin(), values_.end(), x, compare_) - values_.begin());
    }
    int upper_bound(const T& x) const {
        return int(std::upper_bound(values_.begin(), values_.end(), x, compare_) - values_.begin());
    }
    std::optional<int> index(const T& x) const {
        int i = lower_bound(x);
        if (i == size() || compare_(x, values_[i])) return std::nullopt;
        return i;
    }
    bool contains(const T& x) const { return index(x).has_value(); }
    const T& value(int i) const {
        assert(0 <= i && i < size());
        return values_[i];
    }
    const std::vector<T>& values() const { return values_; }

  private:
    std::vector<T> values_;
    Compare compare_;
};

}  // namespace cp
