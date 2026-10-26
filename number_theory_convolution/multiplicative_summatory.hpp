#pragma once
#include "quotient_prefix.hpp"

namespace cp {
// Arbitrary multiplicative f with f(1)=1. Supply prime-prefix sums and f(p^e).
// Reverse sieving computes ALL quotient-prefix sums, using O(sqrt N) ring elements.
template<class T, class PrimePower>
QuotientPrefix<T> multiplicative_summatory_table(const QuotientPrefix<T>& prime_prefix, PrimePower prime_power) {
    using namespace quotient_detail;
    auto result = prime_prefix;
    const auto primes = small_primes(result.root());
    for (auto it = primes.rbegin(); it != primes.rend(); ++it) {
        const auto p = *it;
        const auto before = prime_prefix.sum(p);
        // Descending x keeps result[x/p^e] at the previous stage.
        for (auto x : result.quotients()) {
            if (x < p * p) break;
            T value = result.sum(x);
            std::uint64_t power = p;
            unsigned exponent = 1;
            while (power <= x / p) {
                const T contribution = mul(T(prime_power(p, exponent, power)), sub(result.sum(x / power), before));
                value = add(value, contribution);
                power *= p; ++exponent;
                value = add(value, T(prime_power(p, exponent, power)));
            }
            result.set(x, std::move(value));
        }
    }
    for (auto x : result.quotients()) result.set(x, add(result.sum(x), T(1)));
    return result;
}
template<class T, class PrimePower>
T multiplicative_summatory(const QuotientPrefix<T>& prime_prefix, PrimePower prime_power) {
    return multiplicative_summatory_table(prime_prefix, prime_power).total();
}
} // namespace cp
