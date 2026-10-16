#define PROBLEM "https://judge.yosupo.jp/problem/general_matching"
#include <iostream>
#include <vector>
#include "combinatorial_optimization/general_matching.hpp"
int main(){std::ios::sync_with_stdio(false);std::cin.tie(nullptr);int n,m;std::cin>>n>>m;cp::GeneralMatching g(n);while(m--){int u,v;std::cin>>u>>v;g.add_edge(u,v);}auto r=g.maximum_matching();std::cout<<r.cardinality<<'\n';for(auto[u,v]:r.pairs)std::cout<<u<<' '<<v<<'\n';}
