#pragma once
#include <algorithm>
#include <cassert>
#include <cstddef>
#include <optional>
#include <vector>
#include "detail/arithmetic.hpp"

namespace cp {
struct KnapsackItem { int weight; long long value; };
struct BoundedKnapsackItem { int weight; long long value, count; };
struct KnapsackResult {
    std::vector<std::optional<long long>> dp; // 重さちょうどの最大価値。未到達は nullopt。
    explicit KnapsackResult(int capacity) {
        assert(capacity >= 0);
        dp.resize(static_cast<std::size_t>(capacity) + 1); dp[0] = 0;
    }
    long long best_value() const {
        long long answer = 0;
        for (const auto& value : dp) if (value) answer = std::max(answer, *value);
        return answer;
    }
};
namespace detail {
inline void knapsack_relax(std::optional<long long>& target, long long candidate) {
    if (!target || *target < candidate) target = candidate;
}
}
} // namespace cp
