#pragma once
#include "detail/arithmetic.hpp"
#include "matrix.hpp"
#include <cstdint>
#include <optional>

namespace cp {
// Policy supplies value_type, zero(), one(), add(a,b), multiply(a,b).
template <class S>
Matrix<typename S::value_type> semiring_matrix_product(const Matrix<typename S::value_type> &a,
                                                       const Matrix<typename S::value_type> &b, const S &s) {
    if (a.cols() != b.rows())
        throw std::invalid_argument("semiring matrix dimensions mismatch");
    Matrix<typename S::value_type> c(a.rows(), b.cols(), s.zero());
    for (std::size_t i = 0; i < a.rows(); ++i)
        for (std::size_t k = 0; k < a.cols(); ++k)
            for (std::size_t j = 0; j < b.cols(); ++j)
                c(i, j) = s.add(c(i, j), s.multiply(a(i, k), b(k, j)));
    return c;
}
template <class S>
Matrix<typename S::value_type> semiring_matrix_power(Matrix<typename S::value_type> a, std::uint64_t exponent,
                                                     const S &s) {
    if (a.rows() != a.cols())
        throw std::invalid_argument("semiring power needs a square matrix");
    Matrix<typename S::value_type> result(a.rows(), a.rows(), s.zero());
    for (std::size_t i = 0; i < a.rows(); ++i)
        result(i, i) = s.one();
    for (; exponent; exponent >>= 1) {
        if (exponent & 1)
            result = semiring_matrix_product(result, a, s);
        if (exponent > 1)
            a = semiring_matrix_product(a, a, s);
    }
    return result;
}
template <class T, bool Maximum = false> struct TropicalSemiring {
    using value_type = std::optional<T>;
    value_type zero() const { return std::nullopt; }
    value_type one() const { return T(0); }
    value_type add(const value_type &a, const value_type &b) const {
        if (!a)
            return b;
        if (!b)
            return a;
        return Maximum ? std::max(*a, *b) : std::min(*a, *b);
    }
    value_type multiply(const value_type &a, const value_type &b) const {
        if (!a || !b)
            return std::nullopt;
        linear_algebra_detail::input(*a);
        linear_algebra_detail::input(*b);
        T sum = *a;
        linear_algebra_detail::add(sum, *b);
        return sum;
    }
};
template <class T> using MinPlusSemiring = TropicalSemiring<T, false>;
template <class T> using MaxPlusSemiring = TropicalSemiring<T, true>;
struct BooleanSemiring {
    using value_type = std::uint8_t;
    value_type zero() const { return 0; }
    value_type one() const { return 1; }
    value_type add(value_type a, value_type b) const { return a || b; }
    value_type multiply(value_type a, value_type b) const { return a && b; }
};
} // namespace cp
