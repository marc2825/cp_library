#define PROBLEM "https://atcoder.jp/contests/abc002/tasks/abc002_4"
#include <iostream>
#include "graph_combinatorics/maximum_clique.hpp"
int main() {
    int n,m; std::cin>>n>>m; cp::Graph<> g(n);
    for(int i=0;i<m;++i) { int u,v; std::cin>>u>>v; g.add_edge(u-1,v-1); }
    std::cout<<cp::maximum_clique(g).size()<<'\n';
}
