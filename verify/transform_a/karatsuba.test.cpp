#define PROBLEM "https://judge.yosupo.jp/problem/convolution_mod"
#include <iostream>
#include <vector>
#include "convolution/karatsuba.hpp"
#include "../linear_algebra/modint.hpp"
int main(){int n,m;std::cin>>n>>m;std::vector<TestModInt<998244353>>a(n),b(m);for(auto&x:a){int v;std::cin>>v;x=v;}for(auto&x:b){int v;std::cin>>v;x=v;}for(auto x:cp::convolution_karatsuba(a,b))std::cout<<x.value<<' ';std::cout<<'\n';}
