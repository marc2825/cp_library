#pragma once
#include <cstddef>
#include <stdexcept>
#include <type_traits>
#include <vector>
#include "../../number_theory_convolution/detail/arithmetic.hpp"

namespace cp {
namespace set_function_detail {
inline void validate_size(std::size_t size) {
    if (!size || (size & (size - 1))) throw std::invalid_argument("set-function table size must be a positive power of two");
}
// 更新先と読み取り元を共有。逆変換はビットの段階を逆順にする。
template <bool Subset, bool Inverse, class T> void transform(std::vector<T>& f) {
    static_assert(!std::is_same_v<T, bool>, "bool coefficients are not supported");
    validate_size(f.size());
    const auto stage = [&](std::size_t bit) {
        for (std::size_t first = 0; first < f.size(); first += 2 * bit)
            for (std::size_t offset = 0; offset < bit; ++offset) {
                const auto low = first + offset, high = low + bit;
                const auto target = Subset ? high : low, source = Subset ? low : high;
                if constexpr (Inverse) detail::arithmetic_subtract_assign(f[target], f[source]);
                else detail::arithmetic_add_assign(f[target], f[source]);
            }
    };
    if constexpr (Inverse) for (auto bit = f.size() / 2; bit; bit >>= 1) stage(bit);
    else for (std::size_t bit = 1; bit < f.size(); bit <<= 1) stage(bit);
}
} // namespace set_function_detail
} // namespace cp
