#define PROBLEM "https://judge.yosupo.jp/problem/min_plus_convolution_convex_arbitrary"
#include <iostream>
#include "dp_optimization/min_plus_convolution.hpp"
int main(){int n,m;std::cin>>n>>m;std::vector<long long>a(n),b(m);for(auto &x:a)std::cin>>x;for(auto &x:b)std::cin>>x;auto c=cp::min_plus_convolution_convex_arbitrary(a,b);for(int i=0;i<int(c.size());++i)std::cout<<(i?" ":"")<<c[i];std::cout<<'\n';}
