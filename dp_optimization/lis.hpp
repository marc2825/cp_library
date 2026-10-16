#pragma once
#include <algorithm>
#include <cassert>
#include <functional>
#include <limits>
#include <vector>

namespace cp {
struct LISResult {
    int length = 0;
    std::vector<int> indices, length_ending;
};
// 入力を変更せず、最長部分列の添字と各位置で終わる長さを返す。
// strict=false は同値を許容。降順は greater<T>。詳細: docs/dp_optimization/lis.md。
template <class T, class Compare = std::less<T>>
LISResult lis(const std::vector<T>& a, bool strict = true, Compare compare = Compare{}) {
    assert(a.size() <= static_cast<std::size_t>(std::numeric_limits<int>::max()));
    const int n = static_cast<int>(a.size());
    LISResult result; result.length_ending.resize(n);
    std::vector<int> tails, previous(n, -1);
    for (int i = 0; i < n; ++i) {
        const auto it = strict
            ? std::lower_bound(tails.begin(), tails.end(), a[i], [&](int j, const T& x) { return compare(a[j], x); })
            : std::upper_bound(tails.begin(), tails.end(), a[i], [&](const T& x, int j) { return compare(x, a[j]); });
        const int position = static_cast<int>(it - tails.begin());
        if (position) previous[i] = tails[position - 1];
        result.length_ending[i] = position + 1;
        if (it == tails.end()) tails.push_back(i); else *it = i;
    }
    result.length = static_cast<int>(tails.size());
    if (!tails.empty()) for (int v = tails.back(); v != -1; v = previous[v]) result.indices.push_back(v);
    std::reverse(result.indices.begin(), result.indices.end());
    return result;
}
} // namespace cp
