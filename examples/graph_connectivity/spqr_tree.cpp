#include <iostream>
#include "graph_connectivity/spqr_tree.hpp"
int main() {
    cp::Graph<int> g(3);
    g.add_edge(0,1); g.add_edge(1,2); g.add_edge(2,0); g.add_edge(0,1);
    auto r=cp::spqr_tree(g);
    std::cout << r->nodes.size() << ' ' << r->tree.edge_count() << '\n';
}
