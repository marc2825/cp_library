#include <iostream>
#include "graph_connectivity/unicyclic_decomposition.hpp"
int main() {
    cp::Graph<int> g(5);
    g.add_edge(0,1); g.add_edge(1,2); g.add_edge(2,0); g.add_edge(1,3); g.add_edge(3,4);
    auto r=cp::unicyclic_decomposition(g);
    if (r) std::cout << r->cycle_edge_ids.size() << ' ' << r->root[4] << ' ' << r->depth[4] << '\n';
}
