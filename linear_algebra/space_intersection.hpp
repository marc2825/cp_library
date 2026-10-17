#pragma once
#include <array>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <vector>
#include "gauss_jordan.hpp"
#include "xor_basis.hpp"

namespace cp {
namespace linear_algebra_detail {
template <class T> Matrix<T> row_basis(const Matrix<T>& a, T epsilon) {
    auto reduced = solve(a, std::vector<T>(a.rows(), T(0)), epsilon);
    Matrix<T> basis(reduced.rank, a.cols());
    for (std::size_t i = 0; i < basis.rows(); ++i) for (std::size_t j = 0; j < basis.cols(); ++j) basis(i, j) = reduced.reduced(i, j);
    return basis;
}
template <class T> Matrix<T> intersection(const Matrix<T>& a, const Matrix<T>& b, T epsilon) {
    if (a.cols() != b.cols()) throw std::invalid_argument("spaces must have the same ambient dimension");
    const auto u = row_basis(a, epsilon), v = row_basis(b, epsilon);
    if (u.rows() > static_cast<std::size_t>(-1) - v.rows()) throw std::length_error("space dimension overflow");
    Matrix<T> equations(a.cols(), u.rows() + v.rows());
    for (std::size_t j = 0; j < a.cols(); ++j) {
        for (std::size_t i = 0; i < u.rows(); ++i) equations(j, i) = u(i, j);
        for (std::size_t i = 0; i < v.rows(); ++i) subtract(equations(j, u.rows() + i), v(i, j));
    }
    const auto kernel = solve(equations, std::vector<T>(a.cols(), T(0)), epsilon).nullspace_basis;
    Matrix<T> result(kernel.size(), a.cols());
    for (std::size_t k = 0; k < kernel.size(); ++k) for (std::size_t i = 0; i < u.rows(); ++i)
        for (std::size_t j = 0; j < a.cols(); ++j) add(result(k, j), multiply(kernel[k][i], u(i, j)));
    return result;
}
} // namespace linear_algebra_detail
// 行が生成する空間の交差の基底を行として返す。入力に従属行・0行も可。
// O((p+q)d^2+d^3)時間。docs/linear_algebra/space_intersection.md
template <class T, std::enable_if_t<!std::is_floating_point_v<T>, int> = 0>
Matrix<T> linear_space_intersection(const Matrix<T>& a, const Matrix<T>& b) { return linear_algebra_detail::intersection(a, b, T(0)); }
template <class T, std::enable_if_t<std::is_floating_point_v<T>, int> = 0>
Matrix<T> linear_space_intersection(const Matrix<T>& a, const Matrix<T>& b, T epsilon = T(1e-10)) { return linear_algebra_detail::intersection(a, b, epsilon); }
template <int Mod = 998244353> Matrix<int> linear_space_intersection_mod(const Matrix<int>& a, const Matrix<int>& b) {
    using F = linear_algebra_detail::PrimeField<Mod>;
    Matrix<F> u(a.rows(), a.cols()), v(b.rows(), b.cols());
    for (std::size_t i = 0; i < a.rows(); ++i) for (std::size_t j = 0; j < a.cols(); ++j) u(i, j) = F(a(i, j));
    for (std::size_t i = 0; i < b.rows(); ++i) for (std::size_t j = 0; j < b.cols(); ++j) v(i, j) = F(b(i, j));
    const auto basis = linear_space_intersection(u, v); Matrix<int> result(basis.rows(), basis.cols());
    for (std::size_t i = 0; i < basis.rows(); ++i) for (std::size_t j = 0; j < basis.cols(); ++j) result(i, j) = basis(i, j).value;
    return result;
}
// 符号なし整数のGF(2)空間の交差。U側の成分を消去と一緒に追跡する。
template <class UInt> XorBasis<UInt> xor_space_intersection(const XorBasis<UInt>& a, const XorBasis<UInt>& b) {
    constexpr int bits = std::numeric_limits<UInt>::digits;
    std::array<UInt, bits> pivots{}, images{}; XorBasis<UInt> result;
    const auto insert = [&](UInt x, UInt image) {
        for (int bit = bits; bit-- > 0;) if ((x >> bit) & UInt(1)) {
            if (pivots[bit]) { x ^= pivots[bit]; image ^= images[bit]; }
            else { pivots[bit] = x; images[bit] = image; return; }
        }
        result.insert(image);
    };
    for (const auto& x : a.basis()) insert(x, x);
    for (const auto& x : b.basis()) insert(x, UInt(0));
    return result;
}
} // namespace cp
