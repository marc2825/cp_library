#define PROBLEM "https://judge.yosupo.jp/problem/counting_primes"
#include <cstdint>
#include <iostream>
#include "number_theory_basic/prime_count.hpp"
int main(){std::uint64_t n;if(std::cin>>n)std::cout<<cp::prime_count(n)<<'\n';}
