#pragma once
#include <numeric>
#include <stdexcept>
#include <vector>
#include "mod_inverse.hpp"
#include "mod_pow.hpp"

namespace cp {
// 互いに素な正の法列。最小非負CRT解をtarget_modで返す。積は64bit超過可。
// target_modと入力法は互いに素でなくてもよい。O(k^2)回の安全なmod乗算。
inline long long garner(const std::vector<long long>& residues,const std::vector<long long>& moduli,long long target_mod) {
    if (residues.size()!=moduli.size() || target_mod<=0) throw std::invalid_argument("invalid Garner dimensions or target");
    for (std::size_t i=0;i<moduli.size();++i) {
        if (moduli[i]<=0) throw std::invalid_argument("Garner modulus must be positive");
        for (std::size_t j=0;j<i;++j) if(std::gcd(moduli[i],moduli[j])!=1) throw std::invalid_argument("Garner moduli must be coprime");
    }
    auto mods=moduli; mods.push_back(target_mod);
    std::vector<long long> constant(mods.size()),coefficient(mods.size());
    for (std::size_t i=0;i<mods.size();++i) coefficient[i]=1%mods[i];
    for (std::size_t i=0;i<moduli.size();++i) {
        auto t=mod_multiply(detail::normalized_remainder(residues[i],mods[i])-constant[i],*mod_inverse(coefficient[i],mods[i]),mods[i]);
        for (std::size_t j=i+1;j<mods.size();++j) {
            auto add=mod_multiply(coefficient[j],t,mods[j]);
            constant[j]=constant[j]>=mods[j]-add?constant[j]-(mods[j]-add):constant[j]+add;
            coefficient[j]=mod_multiply(coefficient[j],mods[i],mods[j]);
        }
    }
    return constant.back();
}
} // namespace cp
