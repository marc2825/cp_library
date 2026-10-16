#define PROBLEM "https://judge.yosupo.jp/problem/cycle_detection"
#include <iostream>
#include "graph_basic/all.hpp"
int main() {
    std::ios::sync_with_stdio(false); std::cin.tie(nullptr);
    int n,m;std::cin>>n>>m;cp::Graph<> g(n,cp::GraphKind::Directed);
    for(int i=0,u,v;i<m;++i){std::cin>>u>>v;g.add_edge(u,v);}
    auto cycle=cp::find_cycle(g);if(!cycle){std::cout<<-1<<'\n';return 0;}
    std::cout<<cycle->size()<<'\n';for(int id:cycle->edges)std::cout<<id<<'\n';
}
