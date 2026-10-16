#pragma once
#include <cstddef>
#include <string_view>
#include <vector>
#include "prefix_function.hpp"

namespace cp {
namespace string_algorithm_detail {
template <class Sequence> std::vector<int> kmp_impl(const Sequence& text, const Sequence& pattern) {
    const int n = checked_size(text.size()), m = checked_size(pattern.size());
    std::vector<int> positions;
    if (!m) {
        positions.resize(static_cast<std::size_t>(n) + 1);
        for (std::size_t i = 0; i < positions.size(); ++i) positions[i] = static_cast<int>(i);
        return positions;
    }
    const auto prefix = prefix_impl(pattern); int length = 0;
    for (int i = 0; i < n; ++i) {
        while (length && !(text[i] == pattern[length])) length = prefix[length - 1];
        if (text[i] == pattern[length]) ++length;
        if (length == m) { positions.push_back(i - m + 1); length = prefix[length - 1]; }
    }
    return positions;
}
} // namespace string_algorithm_detail
// text中のpatternの全出現位置。重複一致を含む昇順。空patternは0..text.size()。
inline std::vector<int> kmp_search(std::string_view text, std::string_view pattern) { return string_algorithm_detail::kmp_impl(text, pattern); }
template <class T> std::vector<int> kmp_search(const std::vector<T>& text, const std::vector<T>& pattern) { return string_algorithm_detail::kmp_impl(text, pattern); }
} // namespace cp
