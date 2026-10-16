#define PROBLEM "https://judge.yosupo.jp/problem/maximum_independent_set"
#include <iostream>
#include <vector>
#include "graph_combinatorics/maximum_independent_set.hpp"
int main() {
    int n,m; std::cin>>n>>m; cp::Graph<> g(n);
    for(int i=0;i<m;++i) { int u,v; std::cin>>u>>v; g.add_edge(u,v); }
    auto result=cp::maximum_independent_set(g); std::cout<<result.size()<<'\n';
    for(int v:result) std::cout<<v<<' ';
    std::cout<<'\n';
}
