#define PROBLEM "https://judge.yosupo.jp/problem/counting_spanning_tree_undirected"
#include <iostream>
#include <vector>
#include "graph_combinatorics/matrix_tree.hpp"
int main() {
    int n,m; std::cin>>n>>m; cp::Graph<> g(n);
    for(int i=0;i<m;++i) { int u,v; std::cin>>u>>v; g.add_edge(u,v); }
    std::cout<<cp::count_spanning_trees(g)<<'\n';
}
