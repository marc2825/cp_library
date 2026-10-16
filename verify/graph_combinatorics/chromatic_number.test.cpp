#define PROBLEM "https://judge.yosupo.jp/problem/chromatic_number"
#include <iostream>
#include <vector>
#include "graph_combinatorics/chromatic_number.hpp"
int main() {
    int n,m; std::cin>>n>>m; cp::Graph<> g(n);
    for(int i=0;i<m;++i) { int u,v; std::cin>>u>>v; g.add_edge(u,v); }
    std::cout<<cp::chromatic_number(g)<<'\n';
}
