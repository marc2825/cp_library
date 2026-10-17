#define PROBLEM "https://judge.yosupo.jp/problem/primitive_root"
#include <iostream>
#include <vector>
#include "number_theory_basic/primitive_root.hpp"
int main(){int q;std::cin>>q;while(q--){std::uint64_t p;std::cin>>p;std::cout<<cp::primitive_root(p)<<"\n";}}
