#pragma once
#include "detail/common.hpp"

namespace cp {
// 倍数ゼータ変換の逆。GCD畳み込みの復元に使う。
template <class T> void multiple_mobius_transform(std::vector<T>& f) {
    const PrimeSieve sieve(detail::arithmetic_bound(f.size()));
    detail::arithmetic_transform<false, true>(f, sieve.primes());
}
} // namespace cp
