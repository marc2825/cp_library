#define PROBLEM "https://atcoder.jp/contests/dp/tasks/dp_z"
#include <iostream>
#include <vector>
#include "data_structure/ConvexHullTrick.hpp"
int main(){int n;long long c;std::cin>>n>>c;std::vector<long long> h(n),dp(n);for(auto& x:h)std::cin>>x;CHT_min<long long> lines;for(int i=0;i<n;++i){if(i)dp[i]=lines.query(h[i])+h[i]*h[i]+c;lines.add(-2*h[i],dp[i]+h[i]*h[i]);}std::cout<<dp.back()<<"\n";}
