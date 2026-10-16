#pragma once
#include "detail/common.hpp"

namespace cp {
// c[k]=sum_{lcm(i,j)=k}a[i]*b[j]。出力上限Nを超える組は除外。
template <class T>
std::vector<T> lcm_convolution(const std::vector<T>& a, const std::vector<T>& b) {
    auto inputs = detail::arithmetic_inputs(a, b);
    auto& f = inputs.first; auto& g = inputs.second;
    const PrimeSieve sieve(detail::arithmetic_bound(f.size()));
    detail::arithmetic_transform<true, false>(f, sieve.primes());
    detail::arithmetic_transform<true, false>(g, sieve.primes());
    for (std::size_t i = 1; i < f.size(); ++i) f[i] = detail::arithmetic_multiply(f[i], g[i]);
    detail::arithmetic_transform<true, true>(f, sieve.primes());
    return std::move(f);
}
} // namespace cp
