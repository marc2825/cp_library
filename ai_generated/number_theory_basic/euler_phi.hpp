#pragma once
#include <cstdint>
#include <stdexcept>
#include <vector>
#include "smallest_prime_factor.hpp"
#include "../number_theory_convolution/pollard_rho.hpp"

namespace cp {
// n>=1。単一値は既存の64bit素因数分解を共有。phi(1)=1。
inline std::uint64_t euler_phi(std::uint64_t n) {
    if (!n) throw std::invalid_argument("phi requires positive n");
    auto result=n;
    for (auto [p,e]:prime_factorization_64(n)) { (void)e; result=result/p*(p-1); }
    return result;
}
// [0,limit]をO(limit)時間/領域。0は便宜上0。
inline std::vector<std::uint64_t> euler_phi_sieve(int limit) {
    SmallestPrimeFactor spf(limit);
    std::vector<std::uint64_t> result(static_cast<std::size_t>(limit)+1);
    if (limit>=1) result[1]=1;
    for (long long i=2;i<=limit;++i) {
        const auto p=spf[static_cast<int>(i)]; const auto q=i/p;
        result[i]=result[q]*(q%p==0?p:p-1);
    }
    return result;
}
} // namespace cp
