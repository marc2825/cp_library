#define PROBLEM "https://judge.yosupo.jp/problem/shortest_path"
#include <iostream>
#include "graph_basic/all.hpp"
int main() {
    std::ios::sync_with_stdio(false); std::cin.tie(nullptr);
    int n,m,s,t;std::cin>>n>>m>>s>>t;cp::Graph<> g(n,cp::GraphKind::Directed);
    for(int i=0,u,v;i<m;++i){long long w;std::cin>>u>>v>>w;g.add_edge(u,v,w);}
    auto r=cp::dijkstra(g,s);if(!r.reachable(t)){std::cout<<-1<<'\n';return 0;}
    auto path=r.vertex_path(t);std::cout<<r.dist[t]<<' '<<path.size()-1<<'\n';
    for(int i=1;i<int(path.size());++i)std::cout<<path[i-1]<<' '<<path[i]<<'\n';
}
