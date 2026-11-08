#define PROBLEM "https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=ALDS1_1_C"
#include <iostream>
#include <algorithm>
#include <vector>
#include "number_theory_basic/prime_sieve.hpp"
int main(){int n;std::cin>>n;std::vector<int>a(n);for(auto& x:a)std::cin>>x;cp::PrimeSieve sieve(*std::max_element(a.begin(),a.end()));int count=0;for(int x:a)count+=sieve.is_prime(x);std::cout<<count<<'\n';}
