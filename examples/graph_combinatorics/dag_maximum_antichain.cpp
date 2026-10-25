#include <iostream>
#include "graph_combinatorics/dag_maximum_antichain.hpp"
int main() {
    cp::Graph<int> g(5,cp::GraphKind::Directed);
    g.add_edge(0,2); g.add_edge(1,2); g.add_edge(2,3); g.add_edge(2,4);
    auto r=cp::dag_maximum_antichain(g);
    std::cout << r.antichain.size() << ' ' << r.chains.size() << '\n';
}
