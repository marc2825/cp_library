#pragma once
#include <algorithm>
#include <functional>
#include <limits>
#include <optional>
#include <stdexcept>
#include <vector>
#include "../basic_utility/big_integer.hpp"
#include "knapsack_common.hpp"

namespace cp {
template<class Count = BigInt> struct OptimalLengthCount { int length = 0; Count count = Count(0); };

// 添字列の異なるLISを区別する。空列の解は1。strict=falseは非減少。
template<class Count = BigInt, class T, class Compare = std::less<T>>
OptimalLengthCount<Count> lis_count(const std::vector<T>& values, bool strict = true, Compare compare = {}) {
    if (values.size() > static_cast<std::size_t>(std::numeric_limits<int>::max())) throw std::length_error("LIS too long");
    auto sorted = values; std::sort(sorted.begin(), sorted.end(), compare);
    sorted.erase(std::unique(sorted.begin(), sorted.end(), [&](const T& a, const T& b) {
        return !compare(a,b) && !compare(b,a);
    }), sorted.end());
    std::vector<OptimalLengthCount<Count>> tree(sorted.size() + 1);
    auto merge = [](auto& a, const auto& b) {
        if (a.length < b.length) a = b;
        else if (a.length == b.length) detail::dp_count_add(a.count, b.count);
    };
    for (const auto& value : values) {
        const auto rank = static_cast<std::size_t>(std::lower_bound(sorted.begin(), sorted.end(), value, compare) - sorted.begin());
        OptimalLengthCount<Count> best;
        for (auto j = rank + (strict ? 0 : 1); j; j -= j & -j) merge(best, tree[j]);
        if (best.length == 0) best.count = Count(1);
        ++best.length;
        for (auto j = rank + 1; j < tree.size(); j += j & -j) merge(tree[j], best);
    }
    OptimalLengthCount<Count> result;
    for (auto j = sorted.size(); j; j -= j & -j) merge(result, tree[j]);
    if (result.length == 0) result.count = Count(1);
    return result;
}

// 両列の一致添字pair列を区別する（値列のdistinct LCS数ではない）。O(nm)、領域O(m)。
template<class Count = BigInt, class A, class B, class Equal = std::equal_to<>>
OptimalLengthCount<Count> lcs_alignment_count(const A& a, const B& b, Equal equal = {}) {
    if (std::min(a.size(), b.size()) > static_cast<std::size_t>(std::numeric_limits<int>::max())) throw std::length_error("LCS too long");
    std::vector<OptimalLengthCount<Count>> dp(b.size() + 1, {0, Count(1)});
    for (const auto& x : a) {
        OptimalLengthCount<Count> diagonal = dp[0];
        for (std::size_t j = 0; j < b.size(); ++j) {
            const auto top = dp[j+1], left = dp[j];
            const bool match = equal(x, b[j]);
            const int length = std::max({top.length, left.length, match ? diagonal.length + 1 : 0});
            Count ways(0);
            if (top.length == length) ways = top.count;
            if (left.length == length) {
                Count extra = left.count;
                if (diagonal.length == length) extra -= diagonal.count;
                detail::dp_count_add(ways, extra);
            }
            if (match && diagonal.length + 1 == length) detail::dp_count_add(ways, diagonal.count);
            dp[j+1] = {length, ways}; diagonal = top;
        }
    }
    return dp.back();
}

template<class Count = BigInt> struct KnapsackOptimalCount {
    KnapsackResult values;
    std::vector<Count> counts; // 重さちょうどの最適な品物ID部分集合数。未到達0。
    std::pair<long long, Count> best() const {
        const auto value = values.best_value(); Count ways(0);
        for (std::size_t w = 0; w < counts.size(); ++w)
            if (values.dp[w] && *values.dp[w] == value) detail::dp_count_add(ways, counts[w]);
        return {value, ways};
    }
};
template<class Count = BigInt>
KnapsackOptimalCount<Count> zero_one_knapsack_count(int capacity, const std::vector<KnapsackItem>& items) {
    if (capacity < 0) throw std::invalid_argument("negative knapsack capacity");
    KnapsackOptimalCount<Count> result{KnapsackResult(capacity), std::vector<Count>(static_cast<std::size_t>(capacity) + 1, Count(0))};
    result.counts[0] = Count(1);
    for (const auto& item : items) {
        if (item.weight < 0) throw std::invalid_argument("negative knapsack weight");
        for (int w = capacity; w >= item.weight; --w) if (result.values.dp[w-item.weight]) {
            const auto candidate = detail::dp_add(*result.values.dp[w-item.weight], item.value);
            const Count ways = result.counts[w-item.weight];
            if (!result.values.dp[w] || *result.values.dp[w] < candidate) {
                result.values.dp[w] = candidate; result.counts[w] = ways;
            } else if (*result.values.dp[w] == candidate) detail::dp_count_add(result.counts[w], ways);
        }
    }
    return result;
}
} // namespace cp
