#pragma once
#include "detail/common.hpp"

namespace cp {
// 約数ゼータ変換の逆。f[0]は保持。詳細: 同名のdocs。
template <class T> void divisor_mobius_transform(std::vector<T>& f) {
    const PrimeSieve sieve(detail::arithmetic_bound(f.size()));
    detail::arithmetic_transform<true, true>(f, sieve.primes());
}
} // namespace cp
