#pragma once
#include <algorithm>
#include <cstdint>
#include <deque>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>
#include "knapsack_common.hpp"

namespace cp {
struct KnapsackReconstructionResult {
    KnapsackResult values;
    std::vector<int> weights;
    std::vector<std::vector<long long>> take; // 品物段階ごとの個数。後から上書きされない。
    int best_weight() const {
        int best = 0;
        for (std::size_t w = 1; w < values.dp.size(); ++w)
            if (values.dp[w] && *values.dp[w] > *values.dp[best]) best = static_cast<int>(w);
        return best;
    }
    std::optional<std::vector<long long>> restore(int weight) const {
        if (weight < 0 || static_cast<std::size_t>(weight) >= values.dp.size() || !values.dp[weight]) return std::nullopt;
        std::vector<long long> counts(weights.size());
        for (std::size_t i = weights.size(); i-- > 0;) {
            counts[i] = take[i][weight];
            weight -= static_cast<int>(counts[i] * weights[i]);
        }
        return counts;
    }
};
// 剰余類の単調queue。O(nW)時間・復元表O(nW)領域、重さ0・負価値も可。
inline KnapsackReconstructionResult bounded_knapsack_reconstruct(int capacity, const std::vector<BoundedKnapsackItem>& items) {
    if (capacity < 0) throw std::invalid_argument("negative knapsack capacity");
    KnapsackReconstructionResult result{KnapsackResult(capacity), {}, {}};
    for (const auto& item : items) {
        if (item.weight < 0 || item.count < 0) throw std::invalid_argument("invalid knapsack item");
        result.weights.push_back(item.weight);
        result.take.emplace_back(static_cast<std::size_t>(capacity) + 1);
        auto& take = result.take.back();
        if (!item.count || item.weight > capacity) continue;
        if (!item.weight) {
            if (item.value > 0) {
                const auto gain = detail::dp_multiply(item.value, item.count);
                for (int w = 0; w <= capacity; ++w) if (result.values.dp[w]) {
                    *result.values.dp[w] = detail::dp_add(*result.values.dp[w], gain); take[w] = item.count;
                }
            }
            continue;
        }
        const auto previous = result.values.dp;
        for (int residue = 0; residue < item.weight && residue <= capacity; ++residue) {
            std::deque<std::pair<int, long long>> queue;
            auto score = [&](const auto& candidate, int t) {
                return detail::dp_add(candidate.second, detail::dp_multiply(t - candidate.first, item.value));
            };
            for (int t = 0; t <= (capacity - residue) / item.weight; ++t) {
                while (!queue.empty() && static_cast<long long>(t) - queue.front().first > item.count) queue.pop_front();
                const int w = residue + t * item.weight;
                if (previous[w]) {
                    while (!queue.empty() && score(queue.back(), t) <= *previous[w]) queue.pop_back();
                    queue.emplace_back(t, *previous[w]);
                }
                if (queue.empty()) result.values.dp[w] = std::nullopt;
                else { result.values.dp[w] = score(queue.front(), t); take[w] = t - queue.front().first; }
            }
        }
    }
    return result;
}
inline KnapsackReconstructionResult zero_one_knapsack_reconstruct(int capacity, const std::vector<KnapsackItem>& items) {
    std::vector<BoundedKnapsackItem> bounded;
    for (const auto& item : items) bounded.push_back({item.weight, item.value, 1});
    return bounded_knapsack_reconstruct(capacity, bounded);
}
inline KnapsackReconstructionResult unbounded_knapsack_reconstruct(int capacity, const std::vector<KnapsackItem>& items) {
    if (capacity < 0) throw std::invalid_argument("negative knapsack capacity");
    std::vector<BoundedKnapsackItem> bounded;
    for (const auto& item : items) {
        if (item.weight <= 0) throw std::invalid_argument("unbounded weight must be positive");
        bounded.push_back({item.weight, item.value, capacity / item.weight});
    }
    return bounded_knapsack_reconstruct(capacity, bounded);
}

class BitsetSubsetSumReconstruction {
    int maximum_;
    std::vector<std::uint64_t> words_;
    std::vector<int> parent_item_, parent_sum_, weights_;
public:
    explicit BitsetSubsetSumReconstruction(int maximum) : maximum_(maximum) {
        if (maximum < 0) throw std::invalid_argument("negative subset sum bound");
        words_.resize(static_cast<std::size_t>(maximum) / 64 + 1); words_[0] = 1;
        parent_item_.resize(static_cast<std::size_t>(maximum) + 1, -1); parent_sum_ = parent_item_;
    }
    int add(int weight) {
        if (weight < 0) throw std::invalid_argument("negative subset weight");
        if (weights_.size() >= static_cast<std::size_t>(std::numeric_limits<int>::max())) throw std::length_error("too many items");
        const int id = static_cast<int>(weights_.size()); weights_.push_back(weight);
        if (!weight || weight > maximum_) return id;
        const auto offset = static_cast<std::size_t>(weight) / 64; const int bits = weight % 64;
        for (std::size_t i = words_.size(); i-- > offset;) {
            auto shifted = words_[i-offset] << bits;
            if (bits && i > offset) shifted |= words_[i-offset-1] >> (64-bits);
            if (i + 1 == words_.size() && maximum_ % 64 != 63)
                shifted &= (std::uint64_t(1) << (maximum_ % 64 + 1)) - 1;
            auto fresh = shifted & ~words_[i];
            words_[i] |= shifted;
            while (fresh) {
                const int bit = __builtin_ctzll(fresh), sum = static_cast<int>(64*i + bit);
                parent_item_[sum] = id; parent_sum_[sum] = sum - weight;
                fresh &= fresh - 1;
            }
        }
        return id;
    }
    bool contains(int sum) const { return sum >= 0 && sum <= maximum_ && ((words_[sum/64] >> (sum%64)) & 1); }
    std::optional<std::vector<int>> restore(int sum) const {
        if (!contains(sum)) return std::nullopt;
        std::vector<int> result;
        while (sum) { result.push_back(parent_item_[sum]); sum = parent_sum_[sum]; }
        std::reverse(result.begin(), result.end());
        return result;
    }
};
} // namespace cp
