#define PROBLEM "https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=GRL_6_B"
#include <iostream>
#include <vector>
#include "combinatorial_optimization/min_cost_flow.hpp"
int main(){std::ios::sync_with_stdio(false);std::cin.tie(nullptr);int n,m;long long f;std::cin>>n>>m>>f;cp::MinCostFlow<>g(n);while(m--){int u,v;long long cap,cost;std::cin>>u>>v>>cap>>cost;g.add_edge(u,v,cap,cost);}auto r=g.flow(0,n-1,f);std::cout<<(r.flow==f?r.cost:-1)<<'\n';}
