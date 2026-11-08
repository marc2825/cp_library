#include <iostream>
#include "graph_basic/bfs.hpp"
#include "graph_basic/dfs.hpp"
int main() {
    cp::Graph<> g(5, cp::GraphKind::Directed);
    g.add_edge(0, 1); g.add_edge(1, 2); g.add_edge(0, 2); g.add_edge(3, 4);
    auto shortest = cp::bfs(g, 0);
    std::cout << shortest.dist[2] << '\n'; // 1
    auto dfs = cp::dfs_forest(g);
    for (int v : dfs.preorder) std::cout << v << ' ';
    std::cout << '\n'; // 0 1 2 3 4
    for (int v : dfs.postorder) std::cout << v << ' ';
    std::cout << '\n'; // 2 1 0 4 3
}
