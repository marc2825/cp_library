#pragma once
#include <type_traits>
#include <vector>
#include "prime_sum.hpp"
#include "multiplicative_summatory.hpp"

namespace cp { namespace quotient_detail {
template<class T> T from_big_integer(BigInt value) {
    if constexpr (std::is_same_v<T, BigInt>) return value;
    else if constexpr (std::is_integral_v<T>) return value.template to_integral<T>();
    else {
        const bool negative = value < 0;
        if (negative) value = -value;
        T result = T(0), place = T(1);
        const std::uint64_t base = std::uint64_t(1) << 32;
        while (value != 0) {
            result = add(result, mul(place, T((value % base).template to_integral<std::uint64_t>())));
            value /= base;
            if (value != 0) place = mul(place, T(base));
        }
        return negative ? sub(T(0), result) : result;
    }
}
// Integer-valued power sum via Stirling numbers and binomials. No division in T.
inline BigInt power_prefix(std::uint64_t n, const std::vector<BigInt>& stirling_factorial) {
    BigInt choose = 1, result = 0;
    for (std::size_t j = 0; j < stirling_factorial.size(); ++j) {
        choose = choose * (BigInt(n) + 1 - j) / (j + 1);
        result += stirling_factorial[j] * choose;
    }
    if (stirling_factorial.size() == 1) --result;
    return result;
}
} // namespace quotient_detail
// sum_{p<=x} (c[0]+c[1]*p+...). Composite-mod rings are supported without inverses.
template<class T> QuotientPrefix<T> polynomial_prime_sum_table(std::uint64_t n, const std::vector<T>& coefficients,
        std::uint64_t max_root = 5000000, std::size_t max_degree = 12) {
    using namespace quotient_detail;
    if (!coefficients.empty() && coefficients.size() - 1 > max_degree) throw std::length_error("prime polynomial degree budget");
    QuotientPrefix<T> result(n, [](auto) { return T(0); }, max_root);
    std::vector<BigInt> stirling{1};
    for (std::size_t degree = 0; degree < coefficients.size(); ++degree) {
        auto factorial_stirling = stirling;
        BigInt factorial = 1;
        for (std::size_t j = 0; j <= degree; ++j) { if (j) factorial *= j; factorial_stirling[j] *= factorial; }
        const auto table = prime_sum_sieve<T>(n,
            [&](auto x) { return from_big_integer<T>(power_prefix(x, factorial_stirling)); },
            [&](auto p) { T value = T(1); for (std::size_t j = 0; j < degree; ++j) value = mul(value, T(p)); return value; }, max_root);
        for (auto x : result.quotients()) result.set(x, add(result.sum(x), mul(coefficients[degree], table.sum(x))));
        std::vector<BigInt> next(stirling.size() + 1);
        for (std::size_t j = 1; j < next.size(); ++j) next[j] = stirling[j - 1] + (j < stirling.size() ? BigInt(j) * stirling[j] : BigInt(0));
        stirling = std::move(next);
    }
    return result;
}
// Min_25: f(p) polynomial, arbitrary f(p^e) callback. Returns all quotient prefix sums.
template<class T, class PrimePower>
QuotientPrefix<T> min25_sieve(std::uint64_t n, const std::vector<T>& prime_coefficients, PrimePower prime_power,
                            std::uint64_t max_root = 5000000, std::size_t max_degree = 12) {
    return multiplicative_summatory_table(polynomial_prime_sum_table(n, prime_coefficients, max_root, max_degree), prime_power);
}
} // namespace cp
