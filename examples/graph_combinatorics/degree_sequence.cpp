#include <iostream>
#include "graph_combinatorics/degree_sequence.hpp"
int main() {
    auto g=cp::realize_degree_sequence({2,2,2,2});
    if (g) std::cout << g->edge_count() << '\n';
}
