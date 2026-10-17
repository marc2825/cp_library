#pragma once
#include <cstdint>
#include <stdexcept>
#include "../number_theory_convolution/pollard_rho.hpp"

namespace cp {
// 素数pの最小の正の原始根。合成数はinvalid_argument。p=2では1。
inline std::uint64_t primitive_root(std::uint64_t p) {
    if (!is_prime_64(p)) throw std::invalid_argument("primitive_root requires a prime");
    if (p==2) return 1;
    const auto factors=prime_factorization_64(p-1);
    for (std::uint64_t g=2;g<p;++g) {
        bool good=true;
        for (auto [q,e]:factors) { (void)e; if(factorization_detail::power(g,(p-1)/q,p)==1) { good=false; break; } }
        if (good) return g;
    }
    throw std::logic_error("prime has no primitive root");
}
} // namespace cp
