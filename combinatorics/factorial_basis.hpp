#pragma once
// 説明: docs/combinatorics/factorial_basis.md
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <vector>
#include "stirling.hpp"
namespace cp {
// 冪基底と下降/上昇階乗基底。入力・出力は昇冪/次数順で同じ長さ。
inline std::vector<long long> power_to_factorial_basis(const std::vector<long long>& a,long long modulus=998244353,bool rising=false){
    if(modulus<=0 || a.size()>static_cast<std::size_t>(std::numeric_limits<int>::max()))throw std::invalid_argument("factorial basis size or modulus");
    if(a.empty())return {};
    auto table=stirling_second(static_cast<int>(a.size())-1,modulus);std::vector<long long> result(a.size());for(std::size_t k=0;k<a.size();++k)for(std::size_t j=0;j<=k;++j){auto term=mod_multiply(a[k],table[k][j],modulus);result[j]=rising && (k-j)%2?combinatorics_detail::subtract_mod(result[j],term,modulus):combinatorics_detail::add_mod(result[j],term,modulus);}return result;
}
inline std::vector<long long> factorial_basis_to_power(const std::vector<long long>& a,long long modulus=998244353,bool rising=false){
    if(modulus<=0 || a.size()>static_cast<std::size_t>(std::numeric_limits<int>::max()))throw std::invalid_argument("factorial basis size or modulus");
    if(a.empty())return {};
    auto table=stirling_first(static_cast<int>(a.size())-1,modulus,!rising);std::vector<long long> result(a.size());for(std::size_t k=0;k<a.size();++k)for(std::size_t j=0;j<=k;++j)result[j]=combinatorics_detail::add_mod(result[j],mod_multiply(a[k],table[k][j],modulus),modulus);return result;
}
} // namespace cp
