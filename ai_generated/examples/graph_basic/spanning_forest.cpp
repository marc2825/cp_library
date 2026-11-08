#include <iostream>
#include "graph_basic/kruskal.hpp"
#include "graph_basic/prim.hpp"
int main() {
    cp::Graph<> g(5);
    g.add_edge(0, 1, 4); g.add_edge(0, 2, 1); g.add_edge(1, 2, 2); g.add_edge(3, 4, -1);
    auto a = cp::kruskal(g), b = cp::prim(g);
    std::cout << a.cost << ' ' << b.cost << ' ' << a.component_count << '\n'; // 2 2 2
    for (const auto& edge : a.edges) std::cout << edge.idx << ' ';
    std::cout << '\n'; // 3 1 2
}
