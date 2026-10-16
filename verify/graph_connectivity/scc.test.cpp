#define PROBLEM "https://judge.yosupo.jp/problem/scc"
#include <iostream>
#include "graph_connectivity/scc.hpp"
int main(){
    std::ios::sync_with_stdio(false);std::cin.tie(nullptr);
    int n,m;std::cin>>n>>m;cp::Graph<> g(n,cp::GraphKind::Directed);
    for(int i=0,u,v;i<m;++i){std::cin>>u>>v;g.add_edge(u,v);}
    auto result=cp::scc(g);std::cout<<result.size()<<'\n';
    for(const auto& group:result.groups){std::cout<<group.size();for(int v:group)std::cout<<' '<<v;std::cout<<'\n';}
}
