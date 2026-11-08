#define PROBLEM "https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=GRL_3_C"
#include <iostream>
#include "graph_connectivity/scc.hpp"
int main(){
    std::ios::sync_with_stdio(false);std::cin.tie(nullptr);
    int n,m,q;std::cin>>n>>m;cp::Graph<> g(n,cp::GraphKind::Directed);
    for(int i=0,u,v;i<m;++i){std::cin>>u>>v;g.add_edge(u,v);}
    auto result=cp::scc(g);std::cin>>q;
    for(int i=0,u,v;i<q;++i){std::cin>>u>>v;std::cout<<result.same(u,v)<<'\n';}
}
