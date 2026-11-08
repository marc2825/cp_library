#pragma once
#include "knapsack_common.hpp"

namespace cp {
// 各品物は高々1個。重さ0も可。詳細: docs/dp_optimization/zero_one_knapsack.md。
inline KnapsackResult zero_one_knapsack(int capacity, const std::vector<KnapsackItem>& items) {
    KnapsackResult result(capacity);
    for (auto item : items) {
        assert(item.weight >= 0);
        for (int w = capacity; w >= item.weight; --w) if (result.dp[w - item.weight])
            detail::knapsack_relax(result.dp[w], detail::dp_add(*result.dp[w - item.weight], item.value));
    }
    return result;
}
} // namespace cp
