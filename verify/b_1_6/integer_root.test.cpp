#define PROBLEM "https://judge.yosupo.jp/problem/kth_root_integer"
#include <cstdint>
#include <iostream>
#include "basic_utility/integer_kth_root.hpp"
int main() {
    std::ios::sync_with_stdio(false); std::cin.tie(nullptr);
    int t; std::cin>>t;
    while(t--) { std::uint64_t n; unsigned k; std::cin>>n>>k; std::cout<<cp::integer_kth_root(n,k)<<'\n'; }
}
