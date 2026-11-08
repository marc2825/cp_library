#pragma once
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <vector>
#include "matrix.hpp"
#include "inverse.hpp"
#include "../graph_combinatorics/determinant_mod.hpp"

namespace cp {
namespace linear_algebra_detail {
template <int Mod> Matrix<int> mod_product(const Matrix<int>& a, const Matrix<int>& b) {
    using F = linear_recurrence_detail::Arithmetic<Mod>;
    if (a.cols() != b.rows()) throw std::invalid_argument("modular matrix product dimension mismatch");
    Matrix<int> result(a.rows(), b.cols());
    for (std::size_t i = 0; i < a.rows(); ++i) for (std::size_t k = 0; k < a.cols(); ++k) for (std::size_t j = 0; j < b.cols(); ++j)
        result(i, j) = F::add(result(i, j), F::mul(F::norm(a(i, k)), F::norm(b(k, j))));
    return result;
}
template <int Mod> void low_rank_dimensions(const Matrix<int>& inverse, const Matrix<int>& u, const Matrix<int>& v) {
    static_assert(linear_recurrence_detail::prime(Mod), "Woodbury requires a prime modulus");
    if (inverse.rows() != inverse.cols() || u.rows() != inverse.rows() || v.rows() != inverse.rows() || u.cols() != v.cols())
        throw std::invalid_argument("low-rank update dimensions differ");
}
template <int Mod> Matrix<int> low_rank_kernel(const Matrix<int>& inverse_u, const Matrix<int>& v) {
    using F = linear_recurrence_detail::Arithmetic<Mod>;
    auto kernel = mod_product<Mod>(v.transposed(), inverse_u);
    for (std::size_t i = 0; i < kernel.rows(); ++i) kernel(i, i) = F::add(kernel(i, i), 1);
    return kernel;
}
} // namespace linear_algebra_detail
// inverse=A^-1が既知として(A+U V^T)^-1。更新後が特異ならnullopt。
template <int Mod = 998244353> std::optional<Matrix<int>> low_rank_inverse_update_mod(
    const Matrix<int>& inverse, const Matrix<int>& u, const Matrix<int>& v) {
    using F = linear_recurrence_detail::Arithmetic<Mod>;
    linear_algebra_detail::low_rank_dimensions<Mod>(inverse, u, v);
    const auto inverse_u = linear_algebra_detail::mod_product<Mod>(inverse, u);
    const auto kernel_inverse = matrix_inverse_mod<Mod>(linear_algebra_detail::low_rank_kernel<Mod>(inverse_u, v));
    if (!kernel_inverse) return std::nullopt;
    const auto right = linear_algebra_detail::mod_product<Mod>(v.transposed(), inverse);
    const auto correction = linear_algebra_detail::mod_product<Mod>(linear_algebra_detail::mod_product<Mod>(inverse_u, *kernel_inverse), right);
    Matrix<int> result(inverse.rows(), inverse.cols());
    for (std::size_t i = 0; i < result.rows(); ++i) for (std::size_t j = 0; j < result.cols(); ++j) result(i, j) = F::sub(F::norm(inverse(i, j)), correction(i, j));
    return result;
}
// det(A+U V^T)/det(A)=det(I+V^T A^-1 U)。特異更新なら0。
template <int Mod = 998244353> int low_rank_determinant_ratio_mod(
    const Matrix<int>& inverse, const Matrix<int>& u, const Matrix<int>& v) {
    linear_algebra_detail::low_rank_dimensions<Mod>(inverse, u, v);
    const auto kernel = linear_algebra_detail::low_rank_kernel<Mod>(linear_algebra_detail::mod_product<Mod>(inverse, u), v);
    std::vector<std::vector<long long>> dense(kernel.rows(), std::vector<long long>(kernel.cols()));
    for (std::size_t i = 0; i < kernel.rows(); ++i) for (std::size_t j = 0; j < kernel.cols(); ++j) dense[i][j] = kernel(i, j);
    return determinant_mod<Mod>(dense);
}
} // namespace cp
