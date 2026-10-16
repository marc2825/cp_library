#pragma once
#include <cstdint>
#include <stdexcept>
#include "matrix_product.hpp"

namespace cp {
// 正方行列A^exponent。指数0は単位行列、0×0も可。最後の不要な二乗は行わない。
template <class T> Matrix<T> matrix_power(Matrix<T> base, std::uint64_t exponent) {
    if (base.rows() != base.cols()) throw std::invalid_argument("matrix power requires a square matrix");
    for (const auto& x : base.data()) linear_algebra_detail::input(x);
    auto result = Matrix<T>::identity(base.rows());
    for (; exponent; exponent >>= 1) {
        if (exponent & 1) result = matrix_multiply(result, base);
        if (exponent > 1) base = matrix_multiply(base, base);
    }
    return result;
}
} // namespace cp
