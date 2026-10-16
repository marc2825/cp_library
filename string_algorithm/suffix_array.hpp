#pragma once
#include <algorithm>
#include <cstddef>
#include <numeric>
#include <string_view>
#include <utility>
#include <vector>
#include "detail/common.hpp"

namespace cp {
namespace string_algorithm_detail {
// rankの組を安定な計数ソートで並べる倍増法。O(n log n)、O(n)領域。
inline std::vector<int> suffix_doubling(std::vector<int> rank, int classes) {
    const int n = checked_size(rank.size()); std::vector<int> sa(n), work(n), next_rank(n); std::iota(sa.begin(), sa.end(), 0);
    if (n < 2) return sa;
    for (int width = 1; width < n; width = width >= n - width ? n : width * 2) {
        const auto second = [&](int pos) { return pos < n - width ? rank[pos + width] + 1 : 0; };
        const auto sort_key = [&](const std::vector<int>& from, std::vector<int>& to, auto key) {
            std::vector<int> count(static_cast<std::size_t>(classes) + 1);
            for (int pos : from) ++count[key(pos)];
            int sum = 0; for (auto& x : count) { const int previous = x; x = sum; sum += previous; }
            for (int pos : from) to[count[key(pos)]++] = pos;
        };
        sort_key(sa, work, second); sort_key(work, sa, [&](int pos) { return rank[pos] + 1; });
        int groups = 1; next_rank[sa[0]] = 0;
        for (int i = 1; i < n; ++i) {
            if (rank[sa[i - 1]] != rank[sa[i]] || second(sa[i - 1]) != second(sa[i])) ++groups;
            next_rank[sa[i]] = groups - 1;
        }
        rank.swap(next_rank); classes = groups; if (classes == n) break;
    }
    return sa;
}
} // namespace string_algorithm_detail
// 空suffixを含めず、sの全suffixの始点を辞書順に返す。stringはunsigned byte順。
inline std::vector<int> suffix_array(std::string_view s) {
    string_algorithm_detail::checked_size(s.size()); std::vector<int> rank; rank.reserve(s.size());
    for (unsigned char byte : s) rank.push_back(byte);
    return string_algorithm_detail::suffix_doubling(std::move(rank), 256);
}
template <class T> std::vector<int> suffix_array(const std::vector<T>& s) {
    const int n = string_algorithm_detail::checked_size(s.size()); std::vector<int> order(n), rank(n); std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](int a, int b) { return s[a] < s[b]; });
    int classes = 0;
    for (int i = 0; i < n; ++i) { if (!i || s[order[i - 1]] < s[order[i]]) ++classes; rank[order[i]] = classes - 1; }
    return string_algorithm_detail::suffix_doubling(std::move(rank), classes);
}
} // namespace cp
