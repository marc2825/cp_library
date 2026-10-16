#define PROBLEM "https://judge.yosupo.jp/problem/minimum_spanning_tree"
#include <iostream>
#include "graph_basic/all.hpp"
int main() {
    std::ios::sync_with_stdio(false); std::cin.tie(nullptr);
    int n,m;std::cin>>n>>m;cp::Graph<> g(n);
    for(int i=0,u,v;i<m;++i){long long w;std::cin>>u>>v>>w;g.add_edge(u,v,w);}
    auto r=cp::kruskal(g);std::cout<<r.cost<<'\n';
    for(auto e:r.edges)std::cout<<e.idx<<' ';
    std::cout<<'\n';
}
