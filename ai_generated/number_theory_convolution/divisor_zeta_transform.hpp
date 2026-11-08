#pragma once
#include "detail/common.hpp"

namespace cp {
// f[n] <- sum_{d|n} f[d]。in-place、f[0]は保持。O(N log log N)。
template <class T> void divisor_zeta_transform(std::vector<T>& f) {
    const PrimeSieve sieve(detail::arithmetic_bound(f.size()));
    detail::arithmetic_transform<true, false>(f, sieve.primes());
}
} // namespace cp
