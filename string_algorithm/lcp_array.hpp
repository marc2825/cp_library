#pragma once
#include <stdexcept>
#include <string_view>
#include <vector>
#include "detail/common.hpp"

namespace cp {
namespace string_algorithm_detail {
template <class Sequence> std::vector<int> lcp_impl(const Sequence& s, const std::vector<int>& sa) {
    const int n = checked_size(s.size());
    if (sa.size() != s.size()) throw std::invalid_argument("suffix array length differs from text");
    std::vector<int> rank(n, -1);
    for (int i = 0; i < n; ++i) {
        if (sa[i] < 0 || sa[i] >= n || rank[sa[i]] != -1) throw std::invalid_argument("suffix array must be a permutation");
        rank[sa[i]] = i;
    }
    std::vector<int> lcp(n ? n - 1 : 0);
    for (int i = 0, length = 0; i < n; ++i) {
        if (rank[i] == n - 1) { length = 0; continue; }
        const int j = sa[rank[i] + 1];
        while (length < n - i && length < n - j && s[i + length] == s[j + length]) ++length;
        lcp[rank[i]] = length;
        if (length) --length;
    }
    return lcp;
}
} // namespace string_algorithm_detail
// lcp[i]=隣接suffix sa[i],sa[i+1]のLCP。saは正しい辞書順が前提。O(n)。
inline std::vector<int> lcp_array(std::string_view s, const std::vector<int>& sa) { return string_algorithm_detail::lcp_impl(s, sa); }
template <class T> std::vector<int> lcp_array(const std::vector<T>& s, const std::vector<int>& sa) { return string_algorithm_detail::lcp_impl(s, sa); }
} // namespace cp
