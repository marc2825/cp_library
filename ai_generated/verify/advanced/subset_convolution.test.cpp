#define PROBLEM "https://judge.yosupo.jp/problem/subset_convolution"
#include <iostream>
#include "set_function/subset_convolution.hpp"
#include "verify/number_theory_convolution/modint.hpp"
int main() {
    int n; std::cin>>n; std::vector<ModInt> a(1<<n),b(1<<n);
    for(auto &x:a) { long long v; std::cin>>v; x=ModInt(v); }
    for(auto &x:b) { long long v; std::cin>>v; x=ModInt(v); }
    auto c=cp::subset_convolution(a,b);
    for(int i=0;i<int(c.size());++i) std::cout<<(i?" ":"")<<c[i].value;
    std::cout<<'\n';
}
