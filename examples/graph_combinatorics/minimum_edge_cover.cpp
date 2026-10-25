#include <iostream>
#include "graph_combinatorics/minimum_edge_cover.hpp"
int main() {
    cp::Graph<int> g(4);
    g.add_edge(0,1); g.add_edge(1,2); g.add_edge(2,0); g.add_edge(2,3);
    auto ids=cp::minimum_edge_cover(g);
    if (ids) std::cout << ids->size() << '\n';
}
