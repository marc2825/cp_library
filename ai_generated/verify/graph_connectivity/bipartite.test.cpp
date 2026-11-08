#define PROBLEM "https://atcoder.jp/contests/abc282/tasks/abc282_d"
#include <iostream>
#include "graph_connectivity/bipartite.hpp"
#include "graph_connectivity/connected_components.hpp"
int main(){
    std::ios::sync_with_stdio(false);std::cin.tie(nullptr);
    int n,m;std::cin>>n>>m;cp::Graph<> g(n);
    for(int i=0,u,v;i<m;++i){std::cin>>u>>v;g.add_edge(u-1,v-1);}
    auto color=cp::bipartite_coloring(g);if(!color){std::cout<<0<<'\n';return 0;}
    long long answer=1LL*n*(n-1)/2-m;
    for(const auto& group:cp::connected_components(g).groups){
        long long count[2]={0,0};for(int v:group)++count[(*color)[v]];
        for(auto c:count)answer-=c*(c-1)/2;
    }
    std::cout<<answer<<'\n';
}
