#define PROBLEM "https://atcoder.jp/contests/abc179/tasks/abc179_c"
#include <iostream>
#include "number_theory_basic/smallest_prime_factor.hpp"
int main(){int n;std::cin>>n;cp::SmallestPrimeFactor spf(n);long long answer=0;for(int x=1;x<n;++x){int count=1;for(auto [p,e]:spf.factorize(x)){(void)p;count*=e+1;}answer+=count;}std::cout<<answer<<'\n';}
