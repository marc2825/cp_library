#define PROBLEM "https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=DPL_2_A"
#include <iostream>
#include <optional>
#include <vector>
#include "dp_optimization/subset_dp.hpp"
int main(){std::ios::sync_with_stdio(false);std::cin.tie(nullptr);int n,m;std::cin>>n>>m;std::vector<std::vector<std::optional<long long>>>cost(n,std::vector<std::optional<long long>>(n));while(m--){int u,v;long long w;std::cin>>u>>v>>w;cost[u][v]=w;}auto result=cp::minimum_hamiltonian_cycle(cost);std::cout<<(result.exists?result.cost:-1)<<'\n';}
