#pragma once
// 説明: docs/combinatorics/narayana.md
#include "binomial.hpp"
#include "../number_theory_basic/mod_inverse.hpp"
namespace cp {
// Narayana N(n,k)=C(n,k)C(n,k-1)/n、N(0,0)=1。
inline long long narayana(const FactorialTable& table,long long n,long long k){if(n==0 && k==0)return 1%table.modulus();if(n<=0 || k<=0 || k>n)return 0;auto a=binomial(table,n,k),b=binomial(table,n,k-1);return mod_multiply(mod_multiply(a,b,table.modulus()),*mod_inverse(n,table.modulus()),table.modulus());}
} // namespace cp
