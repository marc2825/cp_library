#define PROBLEM "https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=GRL_1_B"
#include <iostream>
#include "graph_basic/all.hpp"
int main() {
    std::ios::sync_with_stdio(false); std::cin.tie(nullptr);
    int n,m,s=0;std::cin>>n>>m;std::cin>>s;cp::Graph<> g(n,cp::GraphKind::Directed);
    for(int i=0,u,v;i<m;++i){long long w;std::cin>>u>>v>>w;g.add_edge(u,v,w);}
    auto r=cp::bellman_ford(g,s);for(bool b:r.negative_cycle)if(b){std::cout<<"NEGATIVE CYCLE\n";return 0;}
    for(int v=0;v<n;++v){if(r.reachable(v))std::cout<<r.dist[v];else std::cout<<"INF";std::cout<<'\n';}
}
