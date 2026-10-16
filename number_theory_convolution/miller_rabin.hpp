#pragma once
#include <cstdint>
#include <initializer_list>
#include "../number_theory_basic/mod_pow.hpp"

namespace cp {
namespace factorization_detail {
using UInt=std::uint64_t;
inline UInt multiply(UInt a,UInt b,UInt m) { return detail::multiply_remainders(a,b,m); }
inline UInt power(UInt a,UInt b,UInt m) { UInt result=1; while(b) { if(b&1) result=multiply(result,a,m); b>>=1; if(b) a=multiply(a,a,m); } return result; }
}
// uint64_t全域の決定的Miller--Rabin。標準C++17の安全なmod乗算を使用。
inline bool is_prime_64(std::uint64_t n) {
    if(n<2) return false;
    for(unsigned p:{2U,3U,5U,7U,11U,13U,17U,19U,23U,29U,31U,37U}) if(n%p==0) return n==p;
    auto d=n-1; int s=0; while(!(d&1)) { d>>=1; ++s; }
    for(std::uint64_t a:{2ULL,325ULL,9375ULL,28178ULL,450775ULL,9780504ULL,1795265022ULL}) {
        a%=n; if(!a) continue; auto x=factorization_detail::power(a,d,n);
        if(x==1 || x==n-1) continue;
        bool pass=false;
        for(int r=1;r<s;++r) { x=factorization_detail::multiply(x,x,n); if(x==n-1) { pass=true; break; } }
        if(!pass) return false;
    }
    return true;
}
} // namespace cp
