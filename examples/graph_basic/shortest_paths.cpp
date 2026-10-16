#include <iostream>
#include "graph_basic/zero_one_bfs.hpp"
#include "graph_basic/dijkstra.hpp"
#include "graph_basic/bellman_ford.hpp"
#include "graph_basic/floyd_warshall.hpp"
int main() {
    cp::Graph<> g(4, cp::GraphKind::Directed);
    g.add_edge(0, 1, 1); g.add_edge(0, 2, 0); g.add_edge(2, 1, 0); g.add_edge(1, 3, 1);
    auto a = cp::zero_one_bfs(g, 0);
    auto b = cp::dijkstra(g, 0);
    auto c = cp::bellman_ford(g, 0);
    auto d = cp::floyd_warshall(g);
    std::cout << a.dist[3] << ' ' << b.dist[3] << ' ' << c.dist[3] << ' ' << d[0].dist[3] << '\n';
    for (int v : b.vertex_path(3)) std::cout << v << ' ';
    std::cout << '\n'; // 0 2 1 3
    for (int id : b.edge_path(3)) std::cout << id << ' ';
    std::cout << '\n'; // 1 2 3
}
