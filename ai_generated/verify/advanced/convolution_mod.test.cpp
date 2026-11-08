#define PROBLEM "https://judge.yosupo.jp/problem/convolution_mod_1000000007"
#include <iostream>
#include "convolution/convolution_mod.hpp"
int main(){int n,m;std::cin>>n>>m;std::vector<int>a(n),b(m);for(auto &x:a)std::cin>>x;for(auto &x:b)std::cin>>x;auto c=cp::convolution_mod(a,b,1000000007);for(int i=0;i<int(c.size());++i)std::cout<<(i?" ":"")<<c[i];std::cout<<'\n';}
