#pragma once
#include "../../basic_utility/big_integer.hpp"
#include "../matrix.hpp"

namespace cp {
namespace integer_matrix_detail {
template <class T> Matrix<BigInt> convert(const Matrix<T> &a) {
    Matrix<BigInt> b(a.rows(), a.cols());
    for (std::size_t i = 0; i < a.rows(); ++i)
        for (std::size_t j = 0; j < a.cols(); ++j)
            b(i, j) = BigInt(a(i, j));
    return b;
}
inline BigInt residue(BigInt x, const BigInt &m) {
    x %= m;
    return x < 0 ? x + m : x;
}
inline BigInt inverse(BigInt a, const BigInt &m) {
    BigInt b = m, x = 1, y = 0;
    while (b != 0) {
        BigInt q = a / b, r = a - q * b, z = x - q * y;
        a = std::move(b);
        b = std::move(r);
        x = std::move(y);
        y = std::move(z);
    }
    if (a != 1)
        throw std::domain_error("integer modular inverse requires a unit");
    return residue(x, m);
}
} // namespace integer_matrix_detail
} // namespace cp
