#pragma once
// 説明: docs/number_theory_basic/mod_tetration.md
#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include "euler_phi.hpp"
namespace cp {
namespace tetration_detail {
// min(limit, a↑↑height)。巨大な指数は、limitに達する最小指数だけで判断する。
inline std::uint64_t capped(std::uint64_t a,std::uint64_t height,std::uint64_t limit){
    if(!limit)return 0;
    if(!height)return std::min<std::uint64_t>(1,limit);
    if(a==0)return height%2?0:std::min<std::uint64_t>(1,limit);
    if(a==1 || limit==1)return 1;
    std::uint64_t value=1,needed=0;while(value<limit){++needed;if(value>(limit-1)/a){value=limit;break;}value*=a;}
    auto exponent=capped(a,height-1,needed);if(exponent>=needed)return limit;value=1;for(std::uint64_t i=0;i<exponent;++i)value*=a;return value;
}
inline std::uint64_t solve(std::uint64_t a,std::uint64_t height,std::uint64_t modulus){
    if(modulus==1)return 0;
    if(height==0)return 1;
    if(a==0)return height%2?0:1;
    if(a==1)return 1;
    auto phi=euler_phi(modulus),exponent=solve(a,height-1,phi);if(capped(a,height-1,phi)>=phi)exponent+=phi;return factorization_detail::power(a%modulus,exponent,modulus);
}
}
// a>=0, height>=0。高さ0は1、0^0=1。法は正のsigned64。
inline long long mod_tetration(std::uint64_t a,std::uint64_t height,long long modulus){if(modulus<=0)throw std::invalid_argument("tetration modulus must be positive");
    return static_cast<long long>(tetration_detail::solve(a,height,static_cast<std::uint64_t>(modulus)));}
} // namespace cp
