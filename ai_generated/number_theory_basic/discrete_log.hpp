#pragma once
// 説明: docs/number_theory_basic/discrete_log.md
#include <cstdint>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <unordered_map>
#include "mod_pow.hpp"
#include "mod_inverse.hpp"
#include "../basic_utility/integer_kth_root.hpp"
namespace cp {
// 最小x>=0: a^x=b (mod m)。非互いに素もgcdを剥がして処理する。
inline std::optional<std::uint64_t> discrete_log(long long a,long long b,long long modulus,std::uint64_t max_baby_steps=2000000){
    if(modulus<=0)throw std::invalid_argument("discrete log modulus must be positive");
    a=detail::normalized_remainder(a,modulus);b=detail::normalized_remainder(b,modulus);long long factor=1%modulus;std::uint64_t offset=0;
    for(;;){if(b==factor)return offset;auto g=std::gcd(a,modulus);if(g==1)break;if(b%g)return std::nullopt;b/=g;modulus/=g;++offset;factor=mod_multiply(factor,a/g,modulus);}
    if(b==0)return std::nullopt;
    auto inverse=mod_inverse(factor,modulus);long long target=mod_multiply(b,*inverse,modulus);auto width=integer_kth_root(static_cast<std::uint64_t>(modulus),2);if(width*width<static_cast<std::uint64_t>(modulus))++width;
    if(width>max_baby_steps)throw std::length_error("BSGS baby-step budget exceeded");
    std::unordered_map<long long,std::uint64_t> baby;baby.reserve(static_cast<std::size_t>(width));long long power=1%modulus;
    for(std::uint64_t j=0;j<width;++j){baby.emplace(power,j);power=mod_multiply(power,a,modulus);}
    auto step=*mod_inverse(mod_pow(a,static_cast<long long>(width),modulus),modulus);long long current=target;
    for(std::uint64_t i=0;i<=width;++i){auto it=baby.find(current);if(it!=baby.end())return offset+i*width+it->second;current=mod_multiply(current,step,modulus);}return std::nullopt;
}
} // namespace cp
