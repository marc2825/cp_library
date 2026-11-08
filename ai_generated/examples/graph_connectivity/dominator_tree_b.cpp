#include <iostream>
#include "graph_connectivity/dominator_tree.hpp"
int main() {
    cp::Graph<long long> g(3,cp::GraphKind::Directed);
    g.add_edge(0,1); g.add_edge(1,2); g.add_edge(0,2);
    auto d=cp::dominator_tree(g);
    std::cout << d.idom[2] << ' ' << d.dominates(1,2) << '\n';
}
