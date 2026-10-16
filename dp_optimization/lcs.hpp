#pragma once
#include <algorithm>
#include <cassert>
#include <functional>
#include <limits>
#include <utility>
#include <vector>

namespace cp {
struct LCSResult {
    int length = 0;
    std::vector<std::pair<int, int>> matches; // 両列で一致する添字。元の順序。
};
// ランダムアクセス列用。復元 O(NM) 空間。長さのみは O(M) 空間。
// 詳細: docs/dp_optimization/lcs.md。
template <class A, class B, class Equal = std::equal_to<>>
int lcs_length(const A& a, const B& b, Equal equal = Equal{}) {
    assert(a.size() < static_cast<std::size_t>(std::numeric_limits<int>::max()));
    assert(b.size() < static_cast<std::size_t>(std::numeric_limits<int>::max()));
    std::vector<int> dp(b.size() + 1);
    for (const auto& x : a) {
        int diagonal = 0;
        for (std::size_t j = 0; j < b.size(); ++j) {
            const int old = dp[j + 1];
            dp[j + 1] = std::max(dp[j + 1], dp[j]);
            if (equal(x, b[j])) dp[j + 1] = std::max(dp[j + 1], diagonal + 1);
            diagonal = old;
        }
    }
    return dp.back();
}
template <class A, class B, class Equal = std::equal_to<>>
LCSResult lcs(const A& a, const B& b, Equal equal = Equal{}) {
    assert(a.size() < static_cast<std::size_t>(std::numeric_limits<int>::max()));
    assert(b.size() < static_cast<std::size_t>(std::numeric_limits<int>::max()));
    const int n = static_cast<int>(a.size()), m = static_cast<int>(b.size());
    std::vector<std::vector<int>> dp(n + 1, std::vector<int>(m + 1));
    for (int i = 0; i < n; ++i) for (int j = 0; j < m; ++j) {
        dp[i + 1][j + 1] = std::max(dp[i][j + 1], dp[i + 1][j]);
        if (equal(a[i], b[j])) dp[i + 1][j + 1] = std::max(dp[i + 1][j + 1], dp[i][j] + 1);
    }
    LCSResult result; result.length = dp[n][m];
    for (int i = n, j = m; i && j;) {
        if (equal(a[i - 1], b[j - 1]) && dp[i][j] == dp[i - 1][j - 1] + 1) {
            result.matches.emplace_back(--i, --j);
        } else if (dp[i - 1][j] >= dp[i][j - 1]) --i;
        else --j;
    }
    std::reverse(result.matches.begin(), result.matches.end());
    return result;
}
} // namespace cp
