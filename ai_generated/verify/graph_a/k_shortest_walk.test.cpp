#define PROBLEM "https://judge.yosupo.jp/problem/k_shortest_walk"
#include <iostream>
#include "graph_basic/k_shortest_walks.hpp"
int main(){int n,m,s,t,k;std::cin>>n>>m>>s>>t>>k;cp::Graph<long long>g(n,cp::GraphKind::Directed);for(int i=0;i<m;++i){int u,v;long long w;std::cin>>u>>v>>w;g.add_edge(u,v,w);}auto a=cp::k_shortest_walks(g,s,t,k);for(int i=0;i<k;++i)std::cout<<(i<int(a.costs.size())?a.costs[i]:-1)<<'\n';}
