#pragma once
// 説明: docs/combinatorics/binomial_transform.md
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <vector>
#include "binomial.hpp"
namespace cp {
// b[n]=sum_k C(n,k)a[k]。逆は(-1)^(n-k)を掛ける。合成数法も可。
inline std::vector<long long> binomial_transform(const std::vector<long long>& a,long long modulus=998244353,bool inverse=false){
    if(modulus<=0 || a.size()>static_cast<std::size_t>(std::numeric_limits<int>::max()))throw std::invalid_argument("binomial transform size or modulus");
    if(a.empty())return {};
    PascalBinomial table(static_cast<int>(a.size())-1,modulus);std::vector<long long> result(a.size());for(int n=0;n<static_cast<int>(a.size());++n)for(int k=0;k<=n;++k){auto term=mod_multiply(table(n,k),a[k],modulus);result[n]=inverse && (n-k)%2?combinatorics_detail::subtract_mod(result[n],term,modulus):combinatorics_detail::add_mod(result[n],term,modulus);}return result;
}
} // namespace cp
