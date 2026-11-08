#define PROBLEM "https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=ALDS1_11_C"
#include <iostream>
#include "graph_basic/all.hpp"
int main() {
    std::ios::sync_with_stdio(false); std::cin.tie(nullptr);
    int n; std::cin >> n; cp::Graph<> g(n,cp::GraphKind::Directed);
    for(int i=0,u,k;i<n;++i){std::cin>>u>>k;for(int j=0,v;j<k;++j){std::cin>>v;g.add_edge(u-1,v-1);}}
    auto r=cp::bfs(g);
    for(int v=0;v<n;++v)std::cout<<v+1<<' '<<(r.reachable(v)?r.dist[v]:-1)<<'\n';
}
