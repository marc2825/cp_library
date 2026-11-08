#define PROBLEM "https://atcoder.jp/contests/abc186/tasks/abc186_e"
#include <iostream>
#include "number_theory_basic/gcd_lcm.hpp"
#include "number_theory_basic/mod_inverse.hpp"
#include "number_theory_basic/mod_pow.hpp"
int main(){int t;std::cin>>t;while(t--){long long n,s,k;std::cin>>n>>s>>k;const auto g=static_cast<long long>(cp::gcd(n,k));if(s%g){std::cout<<-1<<'\n';continue;}n/=g;s/=g;k/=g;std::cout<<cp::mod_multiply(-s,*cp::mod_inverse(k,n),n)<<'\n';}}
