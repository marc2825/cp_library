#pragma once
#include "detail/arithmetic.hpp"
#include "matrix.hpp"
#include <limits>

namespace cp {
// Symmetric matrix; diagonal ignored. Sum over perfect matchings, O(n 2^n) time / O(2^n) storage.
template <class T> T hafnian(const Matrix<T> &a, std::size_t max_dimension = 22) {
    const auto n = a.rows();
    if (n != a.cols())
        throw std::invalid_argument("hafnian needs a square matrix");
    if (n > max_dimension || n >= std::numeric_limits<std::size_t>::digits)
        throw std::length_error("hafnian dimension budget exceeded");
    for (std::size_t i = 0; i < n; ++i)
        for (std::size_t j = i + 1; j < n; ++j) {
            linear_algebra_detail::input(a(i, j));
            linear_algebra_detail::input(a(j, i));
            if (!(a(i, j) == a(j, i)))
                throw std::invalid_argument("hafnian matrix must be symmetric");
        }
    if (n & 1)
        return T(0);
    const auto size = std::size_t(1) << n;
    std::vector<T> dp(size, T(0));
    dp[0] = T(1);
    for (std::size_t mask = 1; mask < size; ++mask) {
        auto rest = mask;
        bool odd = false;
        while (rest) {
            odd = !odd;
            rest &= rest - 1;
        }
        if (odd)
            continue;
        std::size_t i = 0;
        while (!(mask & (std::size_t(1) << i)))
            ++i;
        rest = mask ^ (std::size_t(1) << i);
        for (std::size_t j = i + 1; j < n; ++j)
            if (rest & (std::size_t(1) << j))
                linear_algebra_detail::add(
                    dp[mask], linear_algebra_detail::multiply(a(i, j), dp[rest ^ (std::size_t(1) << j)]));
    }
    return dp.back();
}
} // namespace cp
