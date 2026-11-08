#define PROBLEM "https://judge.yosupo.jp/problem/directedmst"
#include <iostream>
#include "graph_basic/directed_mst.hpp"
int main(){int n,m,r;if(!(std::cin>>n>>m>>r))return 0;cp::Graph<long long> g(n,cp::GraphKind::Directed);for(int i=0;i<m;++i){int u,v;long long w;std::cin>>u>>v>>w;g.add_edge(u,v,w);}auto a=cp::directed_mst(g,r);if(!a){std::cout<<-1<<'\n';return 0;}std::cout<<a->cost<<'\n';for(int v=0;v<n;++v)std::cout<<a->parent[v]<<(v+1==n?'\n':' ');}
