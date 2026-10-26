#pragma once
#include <cstdint>
#include <stdexcept>
#include <vector>
#include "floor_sum_polynomial.hpp"

namespace cp {
// Sum i^p*((a*i+b) mod m)^q over [left,right). Exact, signed i,a,b, positive m.
inline BigInt polynomial_mod_sum(const BigInt& left, const BigInt& right, const BigInt& a,
        const BigInt& b, const BigInt& m, unsigned p, unsigned q, unsigned max_degree = 12) {
    if (left > right || m <= 0) throw std::invalid_argument("invalid modular-sum interval or modulus");
    if (p > max_degree || q > max_degree - p || p + q > 64) throw std::length_error("modular-sum degree budget");
    const auto degree = p + q;
    const BigInt c = a * left + b;
    const auto moments = floor_sum_moments(right - left, m, a, c, degree, max_degree);
    std::vector<std::vector<BigInt>> choose(degree + 1, std::vector<BigInt>(degree + 1));
    for (unsigned i = 0; i <= degree; ++i) {
        choose[i][0] = choose[i][i] = 1;
        for (unsigned j = 1; j < i; ++j) choose[i][j] = choose[i - 1][j - 1] + choose[i - 1][j];
    }
    BigInt result = 0;
    for (unsigned r = 0; r <= p; ++r) for (unsigned t = 0; t <= q; ++t) for (unsigned s = 0; s <= q - t; ++s)
        result += choose[p][r] * nt_detail::power(left, p - r) * choose[q][t] * nt_detail::power(-m, t)
                * choose[q - t][s] * nt_detail::power(a, s) * nt_detail::power(c, q - t - s) * moments[r + s][t];
    return result;
}
inline std::uint64_t polynomial_mod_sum_mod(const BigInt& left, const BigInt& right, const BigInt& a,
        const BigInt& b, const BigInt& m, unsigned p, unsigned q, std::uint64_t output_modulus,
        unsigned max_degree = 12) {
    if (!output_modulus) throw std::invalid_argument("zero output modulus");
    auto result = polynomial_mod_sum(left, right, a, b, m, p, q, max_degree) % output_modulus;
    if (result < 0) result += output_modulus;
    return result.to_integral<std::uint64_t>();
}
} // namespace cp
