#pragma once
#include <cstdint>
#include <stdexcept>
#include <vector>
#include "smallest_prime_factor.hpp"
#include "../number_theory_convolution/pollard_rho.hpp"

namespace cp {
inline int mobius(std::uint64_t n) {
    if (!n) throw std::invalid_argument("mobius requires positive n");
    int result=1;
    for (auto [p,e]:prime_factorization_64(n)) { (void)p; if(e>1) return 0; result=-result; }
    return result;
}
// [0,limit]をO(limit)時間/領域。mu(0)=0、mu(1)=1。
inline std::vector<int> mobius_sieve(int limit) {
    SmallestPrimeFactor spf(limit); std::vector<int> result(static_cast<std::size_t>(limit)+1);
    if (limit>=1) result[1]=1;
    for (long long i=2;i<=limit;++i) {
        const auto p=spf[static_cast<int>(i)]; const auto q=i/p;
        result[i]=q%p==0?0:-result[q];
    }
    return result;
}
} // namespace cp
