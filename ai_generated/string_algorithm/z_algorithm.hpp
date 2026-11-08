#pragma once
#include <algorithm>
#include <string_view>
#include <vector>
#include "detail/common.hpp"

namespace cp {
namespace string_algorithm_detail {
template <class Sequence> std::vector<int> z_impl(const Sequence& s) {
    const int n = checked_size(s.size()); std::vector<int> z(n);
    if (!n) return z;
    z[0] = n;
    for (int i = 1, left = 0, right = 0; i < n; ++i) {
        if (i < right) z[i] = std::min(right - i, z[i - left]);
        while (z[i] < n - i && s[z[i]] == s[i + z[i]]) ++z[i];
        if (i + z[i] > right) { left = i; right = i + z[i]; }
    }
    return z;
}
} // namespace string_algorithm_detail
// z[i]=先頭とs[i..n)のLCP、z[0]=n。空は空。O(n)。
inline std::vector<int> z_algorithm(std::string_view s) { return string_algorithm_detail::z_impl(s); }
template <class T> std::vector<int> z_algorithm(const std::vector<T>& s) { return string_algorithm_detail::z_impl(s); }
} // namespace cp
