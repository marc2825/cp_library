#pragma once
#include <cstdint>
#include <limits>
#include <map>
#include <stdexcept>
#include "range_set.hpp"

namespace cp {
// Arbitrary int64 keys, duplicates included. Negative keys do not affect MEX.
class DynamicMex {
    std::map<std::int64_t, std::uint64_t> counts_;
    RangeSet<std::uint64_t> present_;
    std::uint64_t size_ = 0;
public:
    std::uint64_t size() const { return size_; }
    std::uint64_t count(std::int64_t x) const {
        const auto it = counts_.find(x); return it == counts_.end() ? 0 : it->second;
    }
    void insert(std::int64_t x) {
        if (size_ == std::numeric_limits<std::uint64_t>::max()) throw std::overflow_error("MEX size overflow");
        auto& count = counts_[x];
        if (count == 0 && x >= 0) present_.insert(static_cast<std::uint64_t>(x), static_cast<std::uint64_t>(x) + 1);
        ++count; ++size_;
    }
    bool erase(std::int64_t x) {
        auto it = counts_.find(x);
        if (it == counts_.end()) return false;
        if (it->second == 1) {
            if (x >= 0) present_.erase(static_cast<std::uint64_t>(x), static_cast<std::uint64_t>(x) + 1);
            counts_.erase(it);
        } else --it->second;
        --size_; return true;
    }
    std::uint64_t mex() const { auto range = present_.containing(0); return range ? range->second : 0; }
    void clear() { counts_.clear(); present_.clear(); size_ = 0; }
};
} // namespace cp
