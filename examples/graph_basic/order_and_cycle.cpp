#include <iostream>
#include "graph_basic/topological_sort.hpp"
#include "graph_basic/cycle.hpp"
int main() {
    cp::Graph<> g(3, cp::GraphKind::Directed);
    g.add_edge(0, 1); g.add_edge(1, 2);
    auto order = cp::topological_sort(g);
    for (int v : *order) std::cout << v << ' ';
    std::cout << '\n';
    g.add_edge(2, 0);
    std::cout << cp::topological_sort(g).has_value() << '\n'; // 0
    auto cycle = cp::find_cycle(g);
    for (int id : cycle->edges) std::cout << id << ' ';
    std::cout << '\n'; // 0 1 2
}
