#define PROBLEM "https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=GRL_1_A"
#include <iostream>
#include "graph_basic/radix_heap_dijkstra.hpp"
int main(){int n,m,s;std::cin>>n>>m>>s;cp::Graph<long long>g(n,cp::GraphKind::Directed);for(int i=0;i<m;++i){int u,v;long long w;std::cin>>u>>v>>w;g.add_edge(u,v,w);}auto p=cp::radix_heap_dijkstra(g,s);for(int v=0;v<n;++v)if(p.reachable(v))std::cout<<p.dist[v]<<'\n';else std::cout<<"INF\n";}
