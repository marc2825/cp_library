#define PROBLEM "https://judge.yosupo.jp/problem/tetration_mod"
#include <cstdint>
#include <iostream>
#include "number_theory_basic/mod_tetration.hpp"
int main(){int t;if(!(std::cin>>t))return 0;while(t--){std::uint64_t a,h;long long m;std::cin>>a>>h>>m;std::cout<<cp::mod_tetration(a,h,m)<<'\n';}}
