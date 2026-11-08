#pragma once
#include "../detail/prime_field.hpp"
#include "../matrix.hpp"

namespace cp {
namespace strassen_detail {
template <int Mod> Matrix<int> combine(const Matrix<int> &a, const Matrix<int> &b, bool subtract = false) {
    using F = typename linear_algebra_detail::PrimeField<Mod>::R;
    Matrix<int> c(a.rows(), a.cols());
    for (std::size_t i = 0; i < a.rows(); ++i)
        for (std::size_t j = 0; j < a.cols(); ++j)
            c(i, j) = subtract ? F::sub(a(i, j), b(i, j)) : F::add(a(i, j), b(i, j));
    return c;
}
template <int Mod> Matrix<int> classical(const Matrix<int> &a, const Matrix<int> &b) {
    using F = typename linear_algebra_detail::PrimeField<Mod>::R;
    Matrix<int> c(a.rows(), b.cols());
    for (std::size_t i = 0; i < a.rows(); ++i)
        for (std::size_t k = 0; k < a.cols(); ++k)
            if (a(i, k))
                for (std::size_t j = 0; j < b.cols(); ++j)
                    c(i, j) = F::add(c(i, j), F::mul(a(i, k), b(k, j)));
    return c;
}
template <int Mod>
Matrix<int> square_product(const Matrix<int> &a, const Matrix<int> &b, std::size_t cutoff) {
    const auto n = a.rows();
    if (n <= cutoff)
        return classical<Mod>(a, b);
    const auto h = n / 2;
    auto block = [&](const Matrix<int> &x, std::size_t row, std::size_t col) {
        Matrix<int> y(h, h);
        for (std::size_t i = 0; i < h; ++i)
            for (std::size_t j = 0; j < h; ++j)
                y(i, j) = x(row + i, col + j);
        return y;
    };
    auto a11 = block(a, 0, 0), a12 = block(a, 0, h), a21 = block(a, h, 0), a22 = block(a, h, h);
    auto b11 = block(b, 0, 0), b12 = block(b, 0, h), b21 = block(b, h, 0), b22 = block(b, h, h);
    auto product = [&](const Matrix<int> &x, const Matrix<int> &y) {
        return square_product<Mod>(x, y, cutoff);
    };
    auto m1 = product(combine<Mod>(a11, a22), combine<Mod>(b11, b22));
    auto m2 = product(combine<Mod>(a21, a22), b11);
    auto m3 = product(a11, combine<Mod>(b12, b22, true));
    auto m4 = product(a22, combine<Mod>(b21, b11, true));
    auto m5 = product(combine<Mod>(a11, a12), b22);
    auto m6 = product(combine<Mod>(a21, a11, true), combine<Mod>(b11, b12));
    auto m7 = product(combine<Mod>(a12, a22, true), combine<Mod>(b21, b22));
    using F = typename linear_algebra_detail::PrimeField<Mod>::R;
    Matrix<int> c(n, n);
    for (std::size_t i = 0; i < h; ++i)
        for (std::size_t j = 0; j < h; ++j) {
            c(i, j) = F::add(F::sub(F::add(m1(i, j), m4(i, j)), m5(i, j)), m7(i, j));
            c(i, h + j) = F::add(m3(i, j), m5(i, j));
            c(h + i, j) = F::add(m2(i, j), m4(i, j));
            c(h + i, h + j) = F::add(F::add(F::sub(m1(i, j), m2(i, j)), m3(i, j)), m6(i, j));
        }
    return c;
}
template <int Mod> Matrix<int> product(const Matrix<int> &a, const Matrix<int> &b, std::size_t cutoff) {
    if (a.rows() <= cutoff || b.cols() <= cutoff)
        return classical<Mod>(a, b);
    std::size_t size = 1;
    while (size < a.rows()) {
        if (size > std::vector<int>().max_size() / 2)
            throw std::length_error("Strassen padding overflow");
        size *= 2;
    }
    Matrix<int> x(size, size), y(size, size);
    for (std::size_t i = 0; i < a.rows(); ++i) {
        for (std::size_t j = 0; j < a.cols(); ++j)
            x(i, j) = a(i, j);
        for (std::size_t j = 0; j < b.cols(); ++j)
            y(i, j) = b(i, j);
    }
    auto z = square_product<Mod>(x, y, cutoff);
    Matrix<int> c(a.rows(), b.cols());
    for (std::size_t i = 0; i < c.rows(); ++i)
        for (std::size_t j = 0; j < c.cols(); ++j)
            c(i, j) = z(i, j);
    return c;
}
} // namespace strassen_detail
} // namespace cp
