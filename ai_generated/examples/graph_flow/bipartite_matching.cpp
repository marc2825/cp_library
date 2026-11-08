#include <iostream>
#include "graph_flow/bipartite_matching.hpp"
int main() {
    cp::BipartiteMatching graph(3, 3);
    graph.add_edge(0, 0);
    graph.add_edge(0, 1);
    graph.add_edge(1, 0);
    graph.add_edge(2, 1);
    graph.add_edge(2, 2);
    const auto result = graph.maximum_matching();
    std::cout << result.cardinality << '\n';
    for (auto [left, right] : result.pairs) std::cout << left << ' ' << right << '\n';
}
