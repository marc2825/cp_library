#include <iostream>
#include "tree/doubling_lca.hpp"
int main() {
    cp::Graph<> g(4);
    g.add_edge(0, 1, 2); g.add_edge(1, 2, 3); g.add_edge(1, 3, 5);
    cp::DoublingLCA<> lca(g);
    std::cout << lca.lca(2, 3) << ' ' << lca.distance(2, 3) << ' ' << lca.jump(2, 3, 1) << '\n';
}
