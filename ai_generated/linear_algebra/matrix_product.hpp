#pragma once
#include <cstddef>
#include <stdexcept>
#include <vector>
#include "matrix.hpp"
#include "detail/arithmetic.hpp"

namespace cp {
// A(n,k) * B(k,m)。整数は各積・部分和の超過、浮動小数点は非有限値を検査。
template <class T> Matrix<T> matrix_multiply(const Matrix<T>& a, const Matrix<T>& b) {
    if (a.cols() != b.rows()) throw std::invalid_argument("matrix product dimensions differ");
    for (const auto& x : a.data()) linear_algebra_detail::input(x);
    for (const auto& x : b.data()) linear_algebra_detail::input(x);
    Matrix<T> result(a.rows(), b.cols());
    for (std::size_t i = 0; i < a.rows(); ++i) for (std::size_t k = 0; k < a.cols(); ++k)
        for (std::size_t j = 0; j < b.cols(); ++j)
            linear_algebra_detail::add(result(i, j), linear_algebra_detail::multiply(a(i, k), b(k, j)));
    return result;
}
template <class T> Matrix<T> operator*(const Matrix<T>& a, const Matrix<T>& b) { return matrix_multiply(a, b); }
// 列ベクトルA*x。空の内側次元では零ベクトル。
template <class T> std::vector<T> matrix_vector_multiply(const Matrix<T>& a, const std::vector<T>& x) {
    if (a.cols() != x.size()) throw std::invalid_argument("matrix and vector dimensions differ");
    for (const auto& value : a.data()) linear_algebra_detail::input(value);
    for (const auto& value : x) linear_algebra_detail::input(value);
    std::vector<T> result(a.rows(), T(0));
    for (std::size_t i = 0; i < a.rows(); ++i) for (std::size_t j = 0; j < a.cols(); ++j)
        linear_algebra_detail::add(result[i], linear_algebra_detail::multiply(a(i, j), x[j]));
    return result;
}
} // namespace cp
