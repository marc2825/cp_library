#define PROBLEM "https://judge.yosupo.jp/problem/bitwise_and_convolution"
#include <algorithm>
#include <iostream>
#include <vector>
#include "set_function/bitwise_convolution.hpp"
#include "../linear_algebra/modint.hpp"
int main(){
    std::ios::sync_with_stdio(false);std::cin.tie(nullptr);
    int bits;std::cin>>bits;std::size_t n=std::size_t(1)<<bits;
    std::vector<TestModInt<998244353>> a(n),b(n);for(auto& x:a){int v;std::cin>>v;x=v;}for(auto& x:b){int v;std::cin>>v;x=v;}
    std::reverse(a.begin(),a.end());std::reverse(b.begin(),b.end());
    auto c=cp::bitwise_or_convolution(a,b);
    std::reverse(c.begin(),c.end());
    for(auto x:c)std::cout<<x.value<<' ';
    std::cout<<'\n';
}
