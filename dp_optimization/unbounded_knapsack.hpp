#pragma once
#include "knapsack_common.hpp"

namespace cp {
// 各品物は何個でも。重さは正。詳細: docs/dp_optimization/unbounded_knapsack.md。
inline KnapsackResult unbounded_knapsack(int capacity, const std::vector<KnapsackItem>& items) {
    KnapsackResult result(capacity);
    for (auto item : items) {
        assert(item.weight > 0);
        for (int w = item.weight; w <= capacity; ++w) if (result.dp[w - item.weight])
            detail::knapsack_relax(result.dp[w], detail::dp_add(*result.dp[w - item.weight], item.value));
    }
    return result;
}
} // namespace cp
