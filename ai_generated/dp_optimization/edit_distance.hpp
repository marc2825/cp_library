#pragma once
#include <algorithm>
#include <cassert>
#include <functional>
#include <limits>
#include <numeric>
#include <vector>

namespace cp {
enum class EditKind { Keep, Insert, Erase, Replace };
struct EditStep { EditKind kind; int source_index, target_index; };
struct EditDistanceResult { int distance = 0; std::vector<EditStep> steps; };
// 挿入・削除・置換は各1、Keepは0。位置は変形前の両列のカーソル。
// 距離のみ O(M) 空間、復元 O(NM) 空間。詳細: docs/dp_optimization/edit_distance.md。
template <class A, class B, class Equal = std::equal_to<>>
int edit_distance(const A& a, const B& b, Equal equal = Equal{}) {
    assert(a.size() < static_cast<std::size_t>(std::numeric_limits<int>::max()));
    assert(b.size() < static_cast<std::size_t>(std::numeric_limits<int>::max()));
    std::vector<int> dp(b.size() + 1); std::iota(dp.begin(), dp.end(), 0);
    int row = 0;
    for (const auto& x : a) {
        int diagonal = dp[0]; dp[0] = ++row;
        for (std::size_t j = 0; j < b.size(); ++j) {
            const int old = dp[j + 1];
            dp[j + 1] = std::min({diagonal + (equal(x, b[j]) ? 0 : 1), dp[j] + 1, dp[j + 1] + 1});
            diagonal = old;
        }
    }
    return dp.back();
}
template <class A, class B, class Equal = std::equal_to<>>
EditDistanceResult edit_alignment(const A& a, const B& b, Equal equal = Equal{}) {
    assert(a.size() < static_cast<std::size_t>(std::numeric_limits<int>::max()));
    assert(b.size() < static_cast<std::size_t>(std::numeric_limits<int>::max()));
    const int n = static_cast<int>(a.size()), m = static_cast<int>(b.size());
    std::vector<std::vector<int>> dp(n + 1, std::vector<int>(m + 1));
    for (int i = 0; i <= n; ++i) dp[i][0] = i;
    for (int j = 0; j <= m; ++j) dp[0][j] = j;
    for (int i = 0; i < n; ++i) for (int j = 0; j < m; ++j)
        dp[i + 1][j + 1] = std::min({dp[i][j] + (equal(a[i], b[j]) ? 0 : 1), dp[i][j + 1] + 1, dp[i + 1][j] + 1});
    EditDistanceResult result; result.distance = dp[n][m];
    int i = n, j = m;
    while (i || j) {
        if (i && j && dp[i][j] == dp[i - 1][j - 1] + (equal(a[i - 1], b[j - 1]) ? 0 : 1)) {
            result.steps.push_back({equal(a[i - 1], b[j - 1]) ? EditKind::Keep : EditKind::Replace, i - 1, j - 1}); --i; --j;
        } else if (i && dp[i][j] == dp[i - 1][j] + 1) { result.steps.push_back({EditKind::Erase, i - 1, j}); --i; }
        else { result.steps.push_back({EditKind::Insert, i, j - 1}); --j; }
    }
    std::reverse(result.steps.begin(), result.steps.end());
    return result;
}
} // namespace cp
