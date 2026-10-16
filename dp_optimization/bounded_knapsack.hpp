#pragma once
#include <deque>
#include <utility>
#include "knapsack_common.hpp"

namespace cp {
// 個数制限。重さ0可、count>=0。剰余類ごとの単調キューで O(NW)。
// 詳細: docs/dp_optimization/bounded_knapsack.md。
inline KnapsackResult bounded_knapsack(int capacity, const std::vector<BoundedKnapsackItem>& items) {
    KnapsackResult result(capacity);
    for (auto item : items) {
        assert(item.weight >= 0 && item.count >= 0);
        if (!item.count || item.weight > capacity) continue;
        if (!item.weight) {
            if (item.value > 0) {
                const auto gain = detail::dp_multiply(item.value, item.count);
                for (auto& value : result.dp) if (value) *value = detail::dp_add(*value, gain);
            }
            continue;
        }
        const auto previous = result.dp;
        for (int r = 0; r < item.weight && r <= capacity; ++r) {
            std::deque<std::pair<int, long long>> candidates;
            auto score = [&](const std::pair<int, long long>& candidate, int t) {
                return detail::dp_add(candidate.second, detail::dp_multiply(t - candidate.first, item.value));
            };
            for (int t = 0; t <= (capacity - r) / item.weight; ++t) {
                while (!candidates.empty() && static_cast<long long>(t) - candidates.front().first > item.count) candidates.pop_front();
                const int w = r + t * item.weight;
                if (previous[w]) {
                    while (!candidates.empty() && score(candidates.back(), t) <= *previous[w]) candidates.pop_back();
                    candidates.emplace_back(t, *previous[w]);
                }
                result.dp[w] = candidates.empty() ? std::optional<long long>{} : score(candidates.front(), t);
            }
        }
    }
    return result;
}
} // namespace cp
