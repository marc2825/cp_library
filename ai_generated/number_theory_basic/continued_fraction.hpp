#pragma once
#include <stdexcept>
#include <vector>
#include "../basic_utility/big_rational.hpp"
#include "detail/advanced_arithmetic.hpp"

namespace cp {
inline std::vector<BigInt> continued_fraction(const BigRational& value) {
    BigInt a = value.numerator(), b = value.denominator();
    std::vector<BigInt> coefficients;
    while (b != 0) {
        const BigInt q = nt_detail::floor_div(a, b), r = a - q * b;
        coefficients.push_back(q); a = b; b = r;
    }
    return coefficients;
}
inline std::vector<BigRational> continued_fraction_convergents(const std::vector<BigInt>& coefficients) {
    BigInt p0 = 0, p1 = 1, q0 = 1, q1 = 0;
    std::vector<BigRational> result;
    for (std::size_t i = 0; i < coefficients.size(); ++i) {
        if (i && coefficients[i] <= 0) throw std::invalid_argument("continued fraction tail must be positive");
        const BigInt p = coefficients[i] * p1 + p0, q = coefficients[i] * q1 + q0;
        result.emplace_back(p, q); p0 = p1; p1 = p; q0 = q1; q1 = q;
    }
    return result;
}
// Minimizes absolute error with denominator<=bound. Ties: smaller denominator, then value.
inline BigRational best_rational_approximation(const BigRational& value, const BigInt& bound) {
    if (bound < 1) throw std::invalid_argument("denominator bound must be positive");
    if (value.denominator() <= bound) return value;
    BigInt n = value.numerator(), d = value.denominator(), p0 = 0, p1 = 1, q0 = 1, q1 = 0;
    for (;;) {
        const BigInt a = nt_detail::floor_div(n, d), q2 = q0 + a * q1;
        if (q2 > bound) break;
        const BigInt p2 = p0 + a * p1, remainder = n - a * d;
        p0 = p1; p1 = p2; q0 = q1; q1 = q2; n = d; d = remainder;
    }
    const BigInt k = (bound - q0) / q1;
    const BigRational a(p0 + k * p1, q0 + k * q1), b(p1, q1);
    const auto da = abs(a - value), db = abs(b - value);
    if (da != db) return da < db ? a : b;
    if (a.denominator() != b.denominator()) return a.denominator() < b.denominator() ? a : b;
    return a < b ? a : b;
}
} // namespace cp
