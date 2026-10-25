#include <iostream>
#include "graph_combinatorics/vizing_edge_coloring.hpp"
int main() {
    cp::Graph<int> g(3);
    g.add_edge(0,1); g.add_edge(1,2); g.add_edge(2,0);
    auto r=cp::vizing_edge_coloring(g);
    std::cout << r.max_degree << ' ' << r.color_count << '\n';
}
