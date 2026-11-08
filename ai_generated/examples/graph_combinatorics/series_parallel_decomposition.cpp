#include <iostream>
#include "graph_combinatorics/series_parallel_decomposition.hpp"
int main() {
    cp::Graph<int> g(4,cp::GraphKind::Directed);
    g.add_edge(0,1); g.add_edge(1,3); g.add_edge(0,2); g.add_edge(2,3);
    auto r=cp::series_parallel_decomposition(g,0,3);
    std::cout << r->nodes.size() << '\n';
}
