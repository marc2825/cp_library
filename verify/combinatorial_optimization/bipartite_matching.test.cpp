#define PROBLEM "https://judge.yosupo.jp/problem/bipartitematching"
#include <iostream>
#include <vector>
#include "combinatorial_optimization/bipartite_matching.hpp"
int main(){std::ios::sync_with_stdio(false);std::cin.tie(nullptr);int l,r,m;std::cin>>l>>r>>m;cp::BipartiteMatching g(l,r);while(m--){int u,v;std::cin>>u>>v;g.add_edge(u,v);}auto result=g.maximum_matching();std::cout<<result.cardinality<<'\n';for(auto[u,v]:result.pairs)std::cout<<u<<' '<<v<<'\n';}
