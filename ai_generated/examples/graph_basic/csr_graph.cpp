#include <iostream>
#include "graph/csr_graph.hpp"
int main() {
    cp::CSRGraph<int> graph(3, {{0, 1, 5}, {1, 1, 2}, {0, 1, 7}});
    std::cout << graph.edge_count() << " " << graph.arc_count() << "\n";
    for (const auto& arc : graph[1]) {
        std::cout << arc.idx << " " << arc.rev << "\n";
    }
    std::cout << graph[2].empty() << "\n";
}
