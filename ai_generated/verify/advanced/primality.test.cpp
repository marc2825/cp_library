#define PROBLEM "https://judge.yosupo.jp/problem/primality_test"
#include <cstdint>
#include <iostream>
#include "number_theory_convolution/miller_rabin.hpp"
int main(){int q;std::cin>>q;while(q--){std::uint64_t n;std::cin>>n;std::cout<<(cp::is_prime_64(n)?"Yes":"No")<<'\n';}}
