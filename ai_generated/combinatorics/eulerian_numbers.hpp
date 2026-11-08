#pragma once
// 説明: docs/combinatorics/eulerian_numbers.md
#include <stdexcept>
#include <vector>
#include "detail/arithmetic.hpp"
#include "../number_theory_basic/mod_pow.hpp"
namespace cp {
// A(n,k): n要素の順列で上昇がk個。行nはk=0..n、末尾はn>0で0。
inline std::vector<std::vector<long long>> eulerian_numbers(int max_n,long long modulus=998244353){
    if(max_n<0 || modulus<=0)throw std::invalid_argument("Eulerian bound or modulus");
    std::vector<std::vector<long long>> table{{1%modulus}};
    for(int n=1;n<=max_n;++n){table.emplace_back(static_cast<std::size_t>(n)+1);for(int k=0;k<n;++k){long long a=k?mod_multiply(n-k,table[n-1][k-1],modulus):0,b=k<n-1 || n==1?mod_multiply(k+1,table[n-1][k],modulus):0;table[n][k]=combinatorics_detail::add_mod(a,b,modulus);}}return table;
}
} // namespace cp
