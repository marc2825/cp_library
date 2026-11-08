#pragma once
#include <cstddef>
#include <stdexcept>
#include <type_traits>
#include <vector>
#include "detail/transform.hpp"

namespace cp {
// 自然順のHadamard変換。逆は各段で2で割る。正の2冪長、O(N log N)。
// 整数の逆変換は各除算がexactであること。docs/set_function/fwht.md
template <class T> void fwht(std::vector<T>& f, bool inverse = false) {
    static_assert(!std::is_same_v<T, bool>, "bool coefficients are not supported");
    set_function_detail::validate_size(f.size());
    const auto stage = [&](std::size_t bit) {
        for (std::size_t first = 0; first < f.size(); first += 2 * bit)
            for (std::size_t offset = 0; offset < bit; ++offset) {
                const auto low = first + offset, high = low + bit;
                T x = f[low], y = f[low];
                detail::arithmetic_add_assign(x, f[high]);
                detail::arithmetic_subtract_assign(y, f[high]);
                if (inverse) {
                    if constexpr (std::is_integral_v<T>)
                        if (x % 2 || y % 2) throw std::domain_error("integer inverse FWHT is not exact");
                    x = x / T(2); y = y / T(2);
                }
                f[low] = x; f[high] = y;
            }
    };
    if (inverse) for (auto bit = f.size() / 2; bit; bit >>= 1) stage(bit);
    else for (std::size_t bit = 1; bit < f.size(); bit <<= 1) stage(bit);
}
} // namespace cp
