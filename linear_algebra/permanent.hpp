#pragma once
#include "detail/arithmetic.hpp"
#include "matrix.hpp"
#include <cstdint>
#include <limits>

namespace cp {
// Ryser in Gray order, O(n 2^n) ring operations, O(n) storage. Use BigInt for exact integers.
template <class T> T permanent(const Matrix<T> &a, std::size_t max_dimension = 24) {
    const auto n = a.rows();
    if (n != a.cols())
        throw std::invalid_argument("permanent needs a square matrix");
    if (n > max_dimension || n >= std::numeric_limits<std::size_t>::digits)
        throw std::length_error("permanent dimension budget exceeded");
    if (!n)
        return T(1);
    for (const auto &x : a.data())
        linear_algebra_detail::input(x);
    const auto limit = std::size_t(1) << n;
    std::vector<T> sums(n, T(0));
    T result(0);
    std::size_t previous = 0;
    bool odd = false;
    for (std::size_t k = 1; k < limit; ++k) {
        auto gray = k ^ (k >> 1), bit = gray ^ previous;
        std::size_t column = 0;
        while ((std::size_t(1) << column) != bit)
            ++column;
        T product(1);
        for (std::size_t i = 0; i < n; ++i) {
            if (gray & bit)
                linear_algebra_detail::add(sums[i], a(i, column));
            else
                linear_algebra_detail::subtract(sums[i], a(i, column));
            product = linear_algebra_detail::multiply(product, sums[i]);
        }
        odd = !odd;
        if (odd == bool(n & 1))
            linear_algebra_detail::add(result, product);
        else
            linear_algebra_detail::subtract(result, product);
        previous = gray;
    }
    return result;
}
} // namespace cp
