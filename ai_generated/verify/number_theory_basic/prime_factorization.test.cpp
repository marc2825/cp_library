#define PROBLEM "https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=NTL_1_A"
#include <iostream>
#include "number_theory_basic/prime_factorization.hpp"
int main(){long long n;std::cin>>n;std::cout<<n<<':';for(auto [p,e]:cp::prime_factorization(n))while(e--)std::cout<<' '<<p;std::cout<<'\n';}
