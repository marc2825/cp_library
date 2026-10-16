#define PROBLEM "https://cses.fi/problemset/task/1098/"
#include <iostream>
#include <utility>
#include <vector>
#include "game_theory/grundy.hpp"
int main(){std::ios::sync_with_stdio(false);std::cin.tie(nullptr);// Removing 1..3 stones has period 4: g(x)=x mod 4.
    const auto g=cp::calculate_grundy(std::vector<std::vector<int>>{{},{0},{0,1},{0,1,2}}).values;
    int t;std::cin>>t;while(t--){int n;std::cin>>n;int sum=0;while(n--){long long x;std::cin>>x;sum^=g[x%4];}std::cout<<(sum?"first":"second")<<'\n';}}
