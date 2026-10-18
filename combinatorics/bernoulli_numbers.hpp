#pragma once
// 説明: docs/combinatorics/bernoulli_numbers.md
#include <stdexcept>
#include <vector>
#include "binomial.hpp"
#include "../number_theory_basic/mod_inverse.hpp"
namespace cp {
// B1=-1/2の規約。1..n+1が可逆な法（特に素数p>n+1）。
inline std::vector<long long> bernoulli_numbers(int n,long long modulus=998244353){
    if(n<0 || n==2147483647)throw std::invalid_argument("Bernoulli bound");
    FactorialTable table(n+1,modulus);std::vector<long long> b(static_cast<std::size_t>(n)+1);b[0]=1%modulus;
    for(int i=1;i<=n;++i){long long sum=0;for(int k=0;k<i;++k)sum=combinatorics_detail::add_mod(sum,mod_multiply(binomial(table,i+1,k),b[k],modulus),modulus);b[i]=mod_multiply(combinatorics_detail::subtract_mod(0,sum,modulus),*mod_inverse(i+1,modulus),modulus);}return b;
}
} // namespace cp
