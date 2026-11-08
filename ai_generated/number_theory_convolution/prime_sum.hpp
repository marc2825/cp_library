#pragma once
#include "quotient_prefix.hpp"
#include "../basic_utility/big_integer.hpp"

namespace cp {
// h completely multiplicative, prefix(x)=sum_{i=1}^x h(i), h(1)=1.
template<class T, class Prefix, class Weight>
QuotientPrefix<T> prime_sum_sieve(std::uint64_t n, Prefix prefix, Weight weight, std::uint64_t max_root = 5000000) {
    using namespace quotient_detail;
    QuotientPrefix<T> result(n, [&](auto x) { return sub(T(prefix(x)), T(1)); }, max_root);
    for (auto p : small_primes(result.root())) {
        const auto before = result.sum(p - 1);
        const T w = weight(p);
        for (auto x : result.quotients()) {
            if (x < p * p) break;
            result.set(x, sub(result.sum(x), mul(w, sub(result.sum(x / p), before))));
        }
    }
    return result;
}
// Exact prime sum; BigInt avoids overflow even when the root budget is raised.
inline QuotientPrefix<BigInt> prime_sum_table(std::uint64_t n, std::uint64_t max_root = 5000000) {
    return prime_sum_sieve<BigInt>(n, [](auto x) { return BigInt(x) * (BigInt(x) + 1) / 2; },
                                  [](auto p) { return BigInt(p); }, max_root);
}
inline BigInt prime_sum(std::uint64_t n, std::uint64_t max_root = 5000000) { return prime_sum_table(n, max_root).total(); }
} // namespace cp
