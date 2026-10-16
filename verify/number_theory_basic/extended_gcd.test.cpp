#define PROBLEM "https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=NTL_1_E"
#include <iostream>
#include <array>
#include <cstdlib>
#include <utility>
#include "number_theory_basic/extended_gcd.hpp"
int main(){long long a,b;std::cin>>a>>b;const auto e=cp::extended_gcd(a,b);const auto A=a/static_cast<long long>(e.gcd),B=b/static_cast<long long>(e.gcd);std::pair<long long,long long> best{e.x,e.y};
// AOJ は |x|+|y| 最小、同点は x<=y を指定するので、折点付近を調べる。
for(long long k:std::array<long long,6>{-e.x/B-1,-e.x/B,-e.x/B+1,e.y/A-1,e.y/A,e.y/A+1}){std::pair<long long,long long> candidate{e.x+k*B,e.y-k*A};auto norm=[](auto p){return std::abs(p.first)+std::abs(p.second);};if(norm(candidate)<norm(best)||(norm(candidate)==norm(best)&&candidate<best))best=candidate;}std::cout<<best.first<<' '<<best.second<<'\n';}
