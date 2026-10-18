#include <iostream>
#include "graph_connectivity/cactus_decomposition.hpp"
int main() {
    cp::Graph<int> g(4); g.add_edge(0,1); g.add_edge(1,2); g.add_edge(2,0); g.add_edge(2,3);
    auto c=cp::cactus_decomposition(g);
    std::cout << c.is_cactus << ' ' << c.blocks.size() << '\n';
}
