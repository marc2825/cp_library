#pragma once
#include "detail/common.hpp"

namespace cp {
// f[n] <- sum_{n|m, m<=N} f[m]。in-place、f[0]は保持。
template <class T> void multiple_zeta_transform(std::vector<T>& f) {
    const PrimeSieve sieve(detail::arithmetic_bound(f.size()));
    detail::arithmetic_transform<false, false>(f, sieve.primes());
}
} // namespace cp
