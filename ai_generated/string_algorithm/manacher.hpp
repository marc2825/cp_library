#pragma once
#include <algorithm>
#include <string_view>
#include <vector>
#include "detail/common.hpp"

namespace cp {
struct PalindromicRadii {
    std::vector<int> odd, even; // odd[i]:中心iを含む半径、even[i]:iの直前を中心とする半径。
    bool is_palindrome(int left, int right) const {
        string_algorithm_detail::range(static_cast<int>(odd.size()), left, right);
        if (left == right) return true;
        const int length = right - left, center = left + length / 2;
        return length & 1 ? odd[center] >= length / 2 + 1 : even[center] >= length / 2;
    }
};
namespace string_algorithm_detail {
template <class Sequence> PalindromicRadii manacher_impl(const Sequence& s) {
    const int n = checked_size(s.size()); PalindromicRadii result{std::vector<int>(n), std::vector<int>(n)};
    for (int i = 0, left = 0, right = -1; i < n; ++i) {
        int radius = i > right ? 1 : std::min(result.odd[left + (right - i)], right - i + 1);
        while (radius <= i && radius < n - i && s[i - radius] == s[i + radius]) ++radius;
        result.odd[i] = radius;
        if (i + radius - 1 > right) { left = i - radius + 1; right = i + radius - 1; }
    }
    for (int i = 0, left = 0, right = -1; i < n; ++i) {
        int radius = i > right ? 0 : std::min(result.even[left + (right - i) + 1], right - i + 1);
        while (radius < i && radius < n - i && s[i - radius - 1] == s[i + radius]) ++radius;
        result.even[i] = radius;
        if (i + radius - 1 > right) { left = i - radius; right = i + radius - 1; }
    }
    return result;
}
} // namespace string_algorithm_detail
// 全ての奇数長／偶数長回文の半径をO(n)で返す。空入力も可、番兵文字は不要。
inline PalindromicRadii manacher(std::string_view s) { return string_algorithm_detail::manacher_impl(s); }
template <class T> PalindromicRadii manacher(const std::vector<T>& s) { return string_algorithm_detail::manacher_impl(s); }
} // namespace cp
