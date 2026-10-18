#pragma once
// 説明: docs/combinatorics/stirling_transform.md
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <vector>
#include "stirling.hpp"
namespace cp {
// b[n]=sum_k S(n,k)a[k]。inverse=trueなら符号付き第1種s(n,k)。
inline std::vector<long long> stirling_transform(const std::vector<long long>& a,long long modulus=998244353,bool inverse=false){
    if(modulus<=0 || a.size()>static_cast<std::size_t>(std::numeric_limits<int>::max()))throw std::invalid_argument("Stirling transform size or modulus");
    if(a.empty())return {};
    int n=static_cast<int>(a.size())-1;auto table=inverse?stirling_first(n,modulus,true):stirling_second(n,modulus);std::vector<long long> result(a.size());for(int i=0;i<=n;++i)for(int k=0;k<=i;++k)result[i]=combinatorics_detail::add_mod(result[i],mod_multiply(table[i][k],a[k],modulus),modulus);return result;
}
} // namespace cp
