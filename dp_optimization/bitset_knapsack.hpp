#pragma once
#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace cp {
// 0-1 部分和の到達可能性。64ビット単位の in-place shift-OR。
// 詳細: docs/dp_optimization/bitset_knapsack.md。
class BitsetKnapsack {
public:
    explicit BitsetKnapsack(int max_sum) : max_sum_(max_sum) {
        assert(max_sum >= 0);
        words_.assign(static_cast<std::size_t>(max_sum) / 64 + 1, 0); words_[0] = 1;
    }
    int max_sum() const { return max_sum_; }
    void add(int weight) {
        assert(weight >= 0);
        if (!weight || weight > max_sum_) return;
        const std::size_t shift = static_cast<std::size_t>(weight) / 64;
        const int bits = weight % 64;
        for (std::size_t i = words_.size(); i-- > shift;) {
            std::uint64_t value = words_[i - shift] << bits;
            if (bits && i > shift) value |= words_[i - shift - 1] >> (64 - bits);
            words_[i] |= value;
        }
        const int used = max_sum_ % 64 + 1;
        if (used < 64) words_.back() &= (std::uint64_t(1) << used) - 1;
    }
    bool contains(int sum) const {
        return 0 <= sum && sum <= max_sum_ && ((words_[sum / 64] >> (sum % 64)) & 1);
    }
    const std::vector<std::uint64_t>& words() const { return words_; }
    std::vector<int> reachable_sums() const {
        std::vector<int> result;
        for (std::size_t s = 0; s <= static_cast<std::size_t>(max_sum_); ++s)
            if (contains(static_cast<int>(s))) result.push_back(static_cast<int>(s));
        return result;
    }
    void reset() { std::fill(words_.begin(), words_.end(), 0); words_[0] = 1; }
private:
    int max_sum_;
    std::vector<std::uint64_t> words_;
};
} // namespace cp
