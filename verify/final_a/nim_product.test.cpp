#define PROBLEM "https://judge.yosupo.jp/problem/nim_product_64"
#include <cstdint>
#include <iostream>
#include "game_theory/nim_product.hpp"
int main(){std::ios::sync_with_stdio(false);std::cin.tie(nullptr);int t;std::cin>>t;while(t--){std::uint64_t a,b;std::cin>>a>>b;std::cout<<cp::nim_product(a,b)<<'\n';}}
