#define PROBLEM "https://judge.yosupo.jp/problem/discrete_logarithm_mod"
#include <iostream>
#include "number_theory_basic/discrete_log.hpp"
int main(){int t;if(!(std::cin>>t))return 0;while(t--){long long a,b,m;std::cin>>a>>b>>m;auto x=cp::discrete_log(a,b,m);if(x)std::cout<<*x<<'\n';else std::cout<<-1<<'\n';}}
