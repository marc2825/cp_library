#define PROBLEM "https://judge.yosupo.jp/problem/sum_of_floor_of_linear"
#include <iostream>
#include <vector>
#include "number_theory_basic/floor_sum.hpp"
int main(){int t;std::cin>>t;while(t--){long long n,m,a,b;std::cin>>n>>m>>a>>b;std::cout<<cp::floor_sum(n,m,a,b)<<"\n";}}
