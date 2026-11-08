#define PROBLEM "https://cses.fi/problemset/task/1745"
#include <iostream>
#include <numeric>
#include <vector>
#include "dp_optimization/bitset_knapsack.hpp"
int main(){std::ios::sync_with_stdio(false);std::cin.tie(nullptr);int n;std::cin>>n;std::vector<int>a(n);for(auto&x:a)std::cin>>x;cp::BitsetKnapsack dp(std::accumulate(a.begin(),a.end(),0));for(int x:a)dp.add(x);auto sums=dp.reachable_sums();std::cout<<sums.size()-1<<'\n';for(int x:sums)if(x)std::cout<<x<<' ';std::cout<<'\n';}
