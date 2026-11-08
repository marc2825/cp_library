#pragma once
#include <algorithm>
#include <cstddef>
#include <stdexcept>
#include <type_traits>
#include <vector>
#include "../number_theory_convolution/detail/arithmetic.hpp"

namespace cp {
namespace karatsuba_detail {
template <class T> std::vector<T> product(const T* a, const T* b, std::size_t n) {
    std::vector<T> result(2 * n, T(0));
    if (n <= 32) {
        for (std::size_t i = 0; i < n; ++i) for (std::size_t j = 0; j < n; ++j)
            detail::arithmetic_add_assign(result[i + j], detail::arithmetic_multiply(a[i], b[j]));
        return result;
    }
    const auto half = n / 2;
    std::vector<T> x(half, T(0)), y(half, T(0));
    for (std::size_t i = 0; i < half; ++i) {
        x[i] = a[i]; detail::arithmetic_add_assign(x[i], a[i + half]);
        y[i] = b[i]; detail::arithmetic_add_assign(y[i], b[i + half]);
    }
    const auto low = product(a, b, half), high = product(a + half, b + half, half);
    auto middle = product(x.data(), y.data(), half);
    for (std::size_t i = 0; i < n; ++i) {
        detail::arithmetic_subtract_assign(middle[i], low[i]);
        detail::arithmetic_subtract_assign(middle[i], high[i]);
        detail::arithmetic_add_assign(result[i], low[i]);
        detail::arithmetic_add_assign(result[i + half], middle[i]);
        detail::arithmetic_add_assign(result[i + n], high[i]);
    }
    return result;
}
} // namespace karatsuba_detail
// 環上の通常の畳み込み。空なら空、末尾0も保持。整数の中間超過は例外。
// O(N^log2(3))時間・O(N)追加領域。docs/convolution/karatsuba.md
template <class T> std::vector<T> convolution_karatsuba(const std::vector<T>& a, const std::vector<T>& b) {
    static_assert(!std::is_same_v<T, bool>, "bool coefficients are not supported");
    if (a.empty() || b.empty()) return {};
    const auto capacity = std::vector<T>().max_size();
    if (a.size() > capacity - (b.size() - 1)) throw std::length_error("Karatsuba result too large");
    const auto length = a.size() + b.size() - 1;
    if (std::min(a.size(), b.size()) <= 32) {
        std::vector<T> result(length, T(0));
        for (std::size_t i = 0; i < a.size(); ++i) for (std::size_t j = 0; j < b.size(); ++j)
            detail::arithmetic_add_assign(result[i + j], detail::arithmetic_multiply(a[i], b[j]));
        return result;
    }
    std::size_t n = 1;
    while (n < std::max(a.size(), b.size())) {
        if (n > capacity / 2) throw std::length_error("Karatsuba padding too large");
        n *= 2;
    }
    if (n > capacity / 2) throw std::length_error("Karatsuba workspace too large");
    std::vector<T> x(n, T(0)), y(n, T(0));
    std::copy(a.begin(), a.end(), x.begin()); std::copy(b.begin(), b.end(), y.begin());
    auto result = karatsuba_detail::product(x.data(), y.data(), n);
    result.resize(length); return result;
}
} // namespace cp
