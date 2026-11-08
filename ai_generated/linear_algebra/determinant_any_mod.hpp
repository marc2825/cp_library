#pragma once
#include "detail/integer_matrix.hpp"
#include <cstdint>

namespace cp {
// Euclidean elimination without dividing by nonunits. Any positive uint64 modulus, including 1.
template <class T> std::uint64_t determinant_any_mod(const Matrix<T> &input, std::uint64_t modulus) {
    if (!modulus)
        throw std::invalid_argument("determinant modulus must be positive");
    if (input.rows() != input.cols())
        throw std::invalid_argument("determinant needs a square matrix");
    __extension__ using Wide = unsigned __int128;
    const auto n = input.rows();
    Matrix<std::uint64_t> a(n, n);
    for (std::size_t i = 0; i < n; ++i)
        for (std::size_t j = 0; j < n; ++j)
            a(i, j) = integer_matrix_detail::residue(BigInt(input(i, j)), BigInt(modulus))
                          .template to_integral<std::uint64_t>();
    auto subtract = [modulus](std::uint64_t x, std::uint64_t y) {
        return x >= y ? x - y : modulus - (y - x);
    };
    std::uint64_t answer = 1 % modulus;
    for (std::size_t k = 0; k < n; ++k) {
        for (std::size_t i = k + 1; i < n; ++i)
            while (a(i, k)) {
                const auto q = a(k, k) / a(i, k);
                for (std::size_t j = k; j < n; ++j) {
                    a(k, j) = subtract(a(k, j), static_cast<std::uint64_t>(Wide(q) * a(i, j) % modulus));
                    std::swap(a(k, j), a(i, j));
                }
                answer = answer ? modulus - answer : 0;
            }
        answer = static_cast<std::uint64_t>(Wide(answer) * a(k, k) % modulus);
    }
    return answer;
}
} // namespace cp
