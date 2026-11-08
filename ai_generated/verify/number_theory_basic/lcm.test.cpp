#define PROBLEM "https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=NTL_1_C"
#include <iostream>
#include "number_theory_basic/gcd_lcm.hpp"
int main(){int n;std::cin>>n;long long result=1,x;while(n--){std::cin>>x;result=static_cast<long long>(*cp::lcm(result,x));}std::cout<<result<<'\n';}
