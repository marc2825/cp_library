#pragma once
#include <algorithm>
#include <cstdint>
#include <numeric>
#include <stdexcept>
#include <utility>
#include <vector>
#include "miller_rabin.hpp"

namespace cp {
namespace factorization_detail {
inline UInt random(UInt& state) { state+=0x9e3779b97f4a7c15ULL; auto x=state; x=(x^(x>>30))*0xbf58476d1ce4e5b9ULL; x=(x^(x>>27))*0x94d049bb133111ebULL; return x^(x>>31); }
inline UInt divisor(UInt n,UInt& state) {
    for(unsigned p:{2U,3U,5U}) if(n%p==0) return p;
    for(;;) {
        UInt y=random(state)%(n-1)+1,c=random(state)%(n-1)+1,g=1,r=1,x=0,saved=0;
        auto step=[&](UInt v) { auto z=multiply(v,v,n); return z>=n-c?z-(n-c):z+c; };
        while(g==1 && r<=(UInt(1)<<20)) {
            x=y; for(UInt i=0;i<r;++i) y=step(y);
            for(UInt k=0;k<r && g==1;k+=128) {
                saved=y; UInt product=1;
                for(UInt i=0;i<std::min(UInt(128),r-k);++i) { y=step(y); product=multiply(product,x>y?x-y:y-x,n); }
                g=std::gcd(product,n);
            }
            r*=2;
        }
        if(g==n) { do { saved=step(saved); g=std::gcd(x>saved?x-saved:saved-x,n); } while(g==1); }
        if(g>1 && g<n) return g;
    }
}
}
// 正整数を昇順の素因数列（重複あり）へ。1は空、0はinvalid_argument。
// Pollard--Rho/Brent。固定seedは再現用、最悪時間保証はない。
inline std::vector<std::uint64_t> factorize_64(std::uint64_t n,std::uint64_t seed=0x72686fULL) {
    if(!n) throw std::invalid_argument("factorization requires a positive integer");
    std::vector<std::uint64_t> result,stack{n};
    while(!stack.empty()) {
        auto v=stack.back(); stack.pop_back(); if(v==1) continue;
        if(is_prime_64(v)) { result.push_back(v); continue; }
        auto d=factorization_detail::divisor(v,seed); stack.push_back(d); stack.push_back(v/d);
    }
    std::sort(result.begin(),result.end()); return result;
}
inline std::vector<std::pair<std::uint64_t,int>> prime_factorization_64(std::uint64_t n,std::uint64_t seed=0x72686fULL) {
    std::vector<std::pair<std::uint64_t,int>> result;
    for(auto p:factorize_64(n,seed)) { if(result.empty() || result.back().first!=p) result.emplace_back(p,1); else ++result.back().second; }
    return result;
}
} // namespace cp
