#define PROBLEM "https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=GRL_3_A"
#include <iostream>
#include "graph_connectivity/articulation_points.hpp"
int main(){
    std::ios::sync_with_stdio(false);std::cin.tie(nullptr);
    int n,m;std::cin>>n>>m;cp::Graph<> g(n);
    for(int i=0,u,v;i<m;++i){std::cin>>u>>v;g.add_edge(u,v);}
    for(int v:cp::articulation_points(g))std::cout<<v<<'\n';
}
