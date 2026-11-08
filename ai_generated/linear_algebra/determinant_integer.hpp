#pragma once
#include "detail/integer_matrix.hpp"

namespace cp {
// Fraction-free Bareiss elimination. Exact arbitrary precision; the empty determinant is 1.
template <class T> BigInt determinant_integer(const Matrix<T> &input) {
    if (input.rows() != input.cols())
        throw std::invalid_argument("determinant needs a square matrix");
    auto a = integer_matrix_detail::convert(input);
    const auto n = a.rows();
    if (!n)
        return 1;
    BigInt previous = 1, sign = 1;
    for (std::size_t k = 0; k + 1 < n; ++k) {
        auto pivot = k;
        while (pivot < n && a(pivot, k) == 0)
            ++pivot;
        if (pivot == n)
            return 0;
        if (pivot != k) {
            for (std::size_t j = 0; j < n; ++j)
                std::swap(a(k, j), a(pivot, j));
            sign = -sign;
        }
        for (std::size_t i = k + 1; i < n; ++i) {
            for (std::size_t j = k + 1; j < n; ++j) {
                auto qr = divmod(a(i, j) * a(k, k) - a(i, k) * a(k, j), previous);
                if (qr.second != 0)
                    throw std::logic_error("nonexact Bareiss division");
                a(i, j) = std::move(qr.first);
            }
            a(i, k) = 0;
        }
        previous = a(k, k);
    }
    return sign * a(n - 1, n - 1);
}
} // namespace cp
