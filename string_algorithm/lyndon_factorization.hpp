#pragma once
#include <string_view>
#include <utility>
#include <vector>
#include "detail/common.hpp"

namespace cp {
// unsigned byte順の非増加Lyndon分解。半開区間を返す。O(n)、空は空。
// docs/string_algorithm/lyndon_factorization.md
inline std::vector<std::pair<int, int>> lyndon_factorization(std::string_view s) {
    const int n = string_algorithm_detail::checked_size(s.size());
    std::vector<std::pair<int, int>> result;
    for (int i = 0; i < n;) {
        int j = i + 1, k = i;
        while (j < n && static_cast<unsigned char>(s[k]) <= static_cast<unsigned char>(s[j])) {
            if (static_cast<unsigned char>(s[k]) < static_cast<unsigned char>(s[j])) k = i;
            else ++k;
            ++j;
        }
        const int length = j - k;
        while (i <= k) { result.emplace_back(i, i + length); i += length; }
    }
    return result;
}
} // namespace cp
