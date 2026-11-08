#define PROBLEM "https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=NTL_1_D"
#include <iostream>
#include <vector>
#include "number_theory_basic/euler_phi.hpp"
int main(){std::uint64_t n;std::cin>>n;std::cout<<cp::euler_phi(n)<<"\n";}
