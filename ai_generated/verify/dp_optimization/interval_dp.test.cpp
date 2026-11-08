#define PROBLEM "https://atcoder.jp/contests/dp/tasks/dp_n"
#include <iostream>
#include <vector>
#include "dp_optimization/interval_dp.hpp"
int main(){std::ios::sync_with_stdio(false);std::cin.tie(nullptr);int n;std::cin>>n;std::vector<long long>prefix(n+1);for(int i=0;i<n;++i){long long x;std::cin>>x;prefix[i+1]=prefix[i]+x;}std::cout<<cp::interval_dp(n,[&](int l,int,int r){return prefix[r]-prefix[l];}).value()<<'\n';}
