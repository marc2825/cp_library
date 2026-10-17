#pragma once
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>
#include "matrix.hpp"
#include "detail/arithmetic.hpp"
#include "detail/prime_field.hpp"

namespace cp {
// det(xI-A)を定数項から昇冪順で返す。体上、正方行列。0×0は{1}。
// Hessenberg相似変形と主小行列の漸化式。O(n^3)時間・O(n^2)領域。
// docs/linear_algebra/characteristic_polynomial.md
template <class T> std::vector<T> characteristic_polynomial(Matrix<T> a) {
    static_assert(!std::is_integral_v<T>, "integer division is not a field: use characteristic_polynomial_mod");
    using namespace linear_algebra_detail;
    if (a.rows() != a.cols()) throw std::invalid_argument("characteristic polynomial requires a square matrix");
    for (const auto& x : a.data()) input(x);
    const auto n = a.rows();
    for (std::size_t col = 0; col + 2 < n; ++col) {
        auto pivot = col + 1;
        if constexpr (std::is_floating_point_v<T>) {
            for (std::size_t i = col + 2; i < n; ++i) if (std::abs(a(i, col)) > std::abs(a(pivot, col))) pivot = i;
        } else while (pivot < n && a(pivot, col) == T(0)) ++pivot;
        if (pivot == n || a(pivot, col) == T(0)) continue;
        if (pivot != col + 1) {
            for (std::size_t j = 0; j < n; ++j) std::swap(a(col + 1, j), a(pivot, j));
            for (std::size_t i = 0; i < n; ++i) std::swap(a(i, col + 1), a(i, pivot));
        }
        for (std::size_t i = col + 2; i < n; ++i) {
            const T factor = finite<T>(a(i, col) / a(col + 1, col));
            a(i, col) = T(0);
            for (std::size_t j = col + 1; j < n; ++j) subtract(a(i, j), multiply(factor, a(col + 1, j)));
            for (std::size_t j = 0; j < n; ++j) add(a(j, col + 1), multiply(factor, a(j, i)));
        }
    }
    std::vector<std::vector<T>> p(n + 1); p[0] = {T(1)};
    for (std::size_t size = 1; size <= n; ++size) {
        p[size].assign(size + 1, T(0));
        for (std::size_t j = 0; j < size; ++j) {
            add(p[size][j + 1], p[size - 1][j]);
            subtract(p[size][j], multiply(a(size - 1, size - 1), p[size - 1][j]));
        }
        T product = T(1);
        for (std::size_t i = size - 1; i > 0; --i) {
            product = multiply(product, a(i, i - 1));
            const T factor = multiply(a(i - 1, size - 1), product);
            for (std::size_t j = 0; j < i; ++j) subtract(p[size][j], multiply(factor, p[i - 1][j]));
        }
    }
    return p[n];
}
template <int Mod = 998244353> std::vector<int> characteristic_polynomial_mod(const Matrix<int>& a) {
    using F = linear_algebra_detail::PrimeField<Mod>;
    Matrix<F> field(a.rows(), a.cols());
    for (std::size_t i = 0; i < a.rows(); ++i) for (std::size_t j = 0; j < a.cols(); ++j) field(i, j) = F(a(i, j));
    const auto polynomial = characteristic_polynomial(field); std::vector<int> result;
    for (const auto& x : polynomial) result.push_back(x.value);
    return result;
}
} // namespace cp
