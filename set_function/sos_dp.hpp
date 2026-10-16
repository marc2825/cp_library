#pragma once
#include <cstddef>
#include <type_traits>
#include <vector>
#include "subset_zeta_transform.hpp"

namespace cp {
// 可換・結合的なmergeで全ての部分集合の値を集約。単位元不要、各S自身も含む。
// min/max/bitwise-or等にも使える。merge内の算術・例外は利用者が管理。
template <class T, class Merge> void sos_dp(std::vector<T>& f, Merge merge) {
    static_assert(!std::is_same_v<T, bool>, "bool coefficients are not supported");
    set_function_detail::validate_size(f.size());
    for (std::size_t bit = 1; bit < f.size(); bit <<= 1)
        for (std::size_t first = 0; first < f.size(); first += 2 * bit)
            for (std::size_t offset = 0; offset < bit; ++offset) {
                const auto low = first + offset, high = low + bit;
                f[high] = merge(f[high], f[low]);
            }
}
// 和のSOS DPは部分集合ゼータと同じ。標準整数は超過を検査する。
template <class T> void sos_dp(std::vector<T>& f) { subset_zeta_transform(f); }
} // namespace cp
