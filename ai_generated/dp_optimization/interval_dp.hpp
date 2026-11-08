#pragma once
#include <array>
#include <cassert>
#include <cstddef>
#include <vector>
#include "detail/arithmetic.hpp"

namespace cp {
struct IntervalDPResult {
    std::vector<std::vector<long long>> dp;
    std::vector<std::vector<int>> split;
    long long value() const { return dp[0].back(); }
    // 選んだ二分割 {l,k,r} を子→親の順で復元。
    std::vector<std::array<int, 3>> merge_order() const {
        std::vector<std::array<int, 3>> result, stack{{0, static_cast<int>(dp.size()) - 1, 0}};
        while (!stack.empty()) {
            auto entry = stack.back(); stack.pop_back();
            const int l = entry[0], r = entry[1], k = split[l][r];
            if (k == -1) continue;
            if (entry[2]) result.push_back({l, k, r});
            else { stack.push_back({l, r, 1}); stack.push_back({k, r, 0}); stack.push_back({l, k, 0}); }
        }
        return result;
    }
};
// dp[l][r] = min_k(dp[l][k] + dp[k][r] + cost(l,k,r))。長さ0/1は0。
// cost は long long に変換できる値。詳細: docs/dp_optimization/interval_dp.md。
template <class Cost>
IntervalDPResult interval_dp(int n, Cost cost) {
    assert(n >= 0);
    IntervalDPResult result;
    result.dp.assign(static_cast<std::size_t>(n) + 1, std::vector<long long>(static_cast<std::size_t>(n) + 1));
    result.split.assign(static_cast<std::size_t>(n) + 1, std::vector<int>(static_cast<std::size_t>(n) + 1, -1));
    for (int length = 2; length <= n; ++length) for (int l = 0; l <= n - length; ++l) {
        const int r = l + length;
        for (int k = l + 1; k < r; ++k) {
            const auto candidate = detail::dp_add(detail::dp_add(result.dp[l][k], result.dp[k][r]), cost(l, k, r));
            if (result.split[l][r] == -1 || candidate < result.dp[l][r]) { result.dp[l][r] = candidate; result.split[l][r] = k; }
        }
    }
    return result;
}
} // namespace cp
