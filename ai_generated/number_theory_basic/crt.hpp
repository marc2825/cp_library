#pragma once
#include <limits>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <vector>
#include "mod_inverse.hpp"
#include "mod_pow.hpp"

namespace cp {
struct CRTResult { long long remainder,modulus; };
// 非互いに素な法も可。矛盾はnullopt、lcm超過はoverflow_error。空は{0,1}。
inline std::optional<CRTResult> crt(const std::vector<long long>& residues,const std::vector<long long>& moduli) {
    if (residues.size()!=moduli.size()) throw std::invalid_argument("CRT dimension mismatch");
    for (auto m:moduli) if(m<=0) throw std::invalid_argument("CRT modulus must be positive");
    long long r=0,m=1;
    for (std::size_t i=0;i<residues.size();++i) {
        auto b=detail::normalized_remainder(residues[i],moduli[i]),n=moduli[i],g=std::gcd(m,n);
        const auto delta=b-r;
        if (delta%g) return std::nullopt;
        const auto reduced=n/g;
        if (m>std::numeric_limits<long long>::max()/reduced) throw std::overflow_error("CRT lcm overflow");
        const auto t=mod_multiply(delta/g,*mod_inverse(m/g,reduced),reduced);
        r+=m*t; m*=reduced;
    }
    return CRTResult{r,m};
}
} // namespace cp
