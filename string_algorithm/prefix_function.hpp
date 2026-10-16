#pragma once
#include <string_view>
#include <vector>
#include "detail/common.hpp"

namespace cp {
namespace string_algorithm_detail {
template <class Sequence> std::vector<int> prefix_impl(const Sequence& s) {
    const int n = checked_size(s.size()); std::vector<int> prefix(n);
    for (int i = 1; i < n; ++i) {
        int length = prefix[i - 1];
        while (length && !(s[i] == s[length])) length = prefix[length - 1];
        if (s[i] == s[length]) ++length;
        prefix[i] = length;
    }
    return prefix;
}
} // namespace string_algorithm_detail
// prefix[i]=s[0..i]の最長の真のborder長。prefix[0]=0、空は空。O(n)。
inline std::vector<int> prefix_function(std::string_view s) { return string_algorithm_detail::prefix_impl(s); }
template <class T> std::vector<int> prefix_function(const std::vector<T>& s) { return string_algorithm_detail::prefix_impl(s); }
} // namespace cp
