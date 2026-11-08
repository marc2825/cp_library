#pragma once
#include <cmath>
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include "matrix.hpp"
#include "detail/arithmetic.hpp"
#include "detail/prime_field.hpp"

namespace cp {
namespace linear_algebra_detail {
template <class T> std::optional<Matrix<T>> inverse(Matrix<T> a, T epsilon) {
    static_assert(!std::is_integral_v<T>, "integer division is not a field: use matrix_inverse_mod");
    if (a.rows() != a.cols()) throw std::invalid_argument("inverse requires a square matrix");
    if constexpr (std::is_floating_point_v<T>) if (!std::isfinite(epsilon) || epsilon <= 0) throw std::invalid_argument("epsilon must be positive and finite");
    for (const auto& x : a.data()) input(x);
    const auto n = a.rows(); auto result = Matrix<T>::identity(n);
    const auto zero = [&](const T& x) { if constexpr (std::is_floating_point_v<T>) return std::abs(x) <= epsilon; else return x == T(0); };
    for (std::size_t col = 0; col < n; ++col) {
        auto pivot = col;
        if constexpr (std::is_floating_point_v<T>) {
            for (std::size_t row = col + 1; row < n; ++row) if (std::abs(a(row, col)) > std::abs(a(pivot, col))) pivot = row;
        } else while (pivot < n && zero(a(pivot, col))) ++pivot;
        if (pivot == n || zero(a(pivot, col))) return std::nullopt;
        if (pivot != col) for (std::size_t j = 0; j < n; ++j) { std::swap(a(col, j), a(pivot, j)); std::swap(result(col, j), result(pivot, j)); }
        const auto divisor = a(col, col);
        // 実数では逆数を先に作らず、各成分を直接割る。
        if constexpr (std::is_floating_point_v<T>) {
            for (std::size_t j = col + 1; j < n; ++j) a(col, j) = finite<T>(a(col, j) / divisor);
            for (std::size_t j = 0; j < n; ++j) result(col, j) = finite<T>(result(col, j) / divisor);
        } else {
            const T inv = T(1) / divisor;
            for (std::size_t j = col + 1; j < n; ++j) a(col, j) = multiply(a(col, j), inv);
            for (std::size_t j = 0; j < n; ++j) result(col, j) = multiply(result(col, j), inv);
        }
        a(col, col) = T(1);
        for (std::size_t row = 0; row < n; ++row) if (row != col) {
            const auto factor = a(row, col);
            for (std::size_t j = col + 1; j < n; ++j) subtract(a(row, j), multiply(factor, a(col, j)));
            for (std::size_t j = 0; j < n; ++j) subtract(result(row, j), multiply(factor, result(col, j)));
            a(row, col) = T(0);
        }
    }
    return result;
}
} // namespace linear_algebra_detail
// 特異ならnullopt。0×0の逆は0×0。O(n^3)時間・O(n^2)領域。
// docs/linear_algebra/inverse.md
template <class T, std::enable_if_t<!std::is_floating_point_v<T>, int> = 0>
std::optional<Matrix<T>> matrix_inverse(const Matrix<T>& a) { return linear_algebra_detail::inverse(a, T(0)); }
template <class T, std::enable_if_t<std::is_floating_point_v<T>, int> = 0>
std::optional<Matrix<T>> matrix_inverse(const Matrix<T>& a, T epsilon = T(1e-10)) { return linear_algebra_detail::inverse(a, epsilon); }
template <int Mod = 998244353> std::optional<Matrix<int>> matrix_inverse_mod(const Matrix<int>& a) {
    using F = linear_algebra_detail::PrimeField<Mod>;
    Matrix<F> field(a.rows(), a.cols());
    for (std::size_t i = 0; i < a.rows(); ++i) for (std::size_t j = 0; j < a.cols(); ++j) field(i, j) = F(a(i, j));
    const auto inverse = matrix_inverse(field);
    if (!inverse) return std::nullopt;
    Matrix<int> result(a.rows(), a.cols());
    for (std::size_t i = 0; i < a.rows(); ++i) for (std::size_t j = 0; j < a.cols(); ++j) result(i, j) = (*inverse)(i, j).value;
    return result;
}
} // namespace cp
