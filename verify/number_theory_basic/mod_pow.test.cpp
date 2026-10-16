#define PROBLEM "https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=NTL_1_B"
#include <iostream>
#include "number_theory_basic/mod_pow.hpp"
int main(){long long a,b;std::cin>>a>>b;std::cout<<cp::mod_pow(a,b,1000000007)<<'\n';}
