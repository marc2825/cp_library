#define PROBLEM "https://atcoder.jp/contests/abc141/tasks/abc141_f"
#include <cstdint>
#include <iostream>
#include <vector>
#include "linear_algebra/xor_basis.hpp"
int main() {
    int n;std::cin>>n;std::vector<std::uint64_t>a(n);std::uint64_t total=0;
    for(auto&x:a){std::cin>>x;total^=x;}
    cp::XorBasis<>basis;for(auto x:a)basis.insert(x&~total);
    std::cout<<total+2*basis.max_xor()<<'\n';
}
