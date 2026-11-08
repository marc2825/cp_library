#define PROBLEM "https://judge.yosupo.jp/problem/factorize"
#include <cstdint>
#include <iostream>
#include "number_theory_convolution/pollard_rho.hpp"
int main(){int q;std::cin>>q;while(q--){std::uint64_t n;std::cin>>n;auto f=cp::factorize_64(n);std::cout<<f.size();for(auto p:f)std::cout<<' '<<p;std::cout<<'\n';}}
