#pragma once
// 説明: docs/number_theory_basic/mod_sqrt.md
#include <algorithm>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include "mod_pow.hpp"
#include "../number_theory_convolution/miller_rabin.hpp"
namespace cp {
// 素数pに対するTonelli-Shanks。2根のうち小さい方。不存在はnullopt。
inline std::optional<long long> mod_sqrt(long long a,long long p){
    if(p<2 || !is_prime_64(static_cast<std::uint64_t>(p)))throw std::invalid_argument("square root modulus must be prime");
    using U=std::uint64_t;U prime=static_cast<U>(p),value=static_cast<U>(detail::normalized_remainder(a,p));if(value==0 || p==2)return static_cast<long long>(value);
    auto power=[&](U x,U e){return factorization_detail::power(x,e,prime);};auto multiply=[&](U x,U y){return factorization_detail::multiply(x,y,prime);};
    if(power(value,(prime-1)/2)!=1)return std::nullopt;
    if(prime%4==3){U x=power(value,(prime+1)/4);return static_cast<long long>(std::min(x,prime-x));}
    U odd=prime-1;unsigned order=0;while(odd%2==0){odd/=2;++order;}U nonresidue=2;while(power(nonresidue,(prime-1)/2)==1)++nonresidue;
    U c=power(nonresidue,odd),x=power(value,(odd+1)/2),t=power(value,odd);
    while(t!=1){unsigned i=0;U square=t;while(square!=1){square=multiply(square,square);++i;}U b=power(c,U(1)<<(order-i-1));x=multiply(x,b);c=multiply(b,b);t=multiply(t,c);order=i;}
    return static_cast<long long>(std::min(x,prime-x));
}
} // namespace cp
