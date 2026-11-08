#include <iostream>
#include "graph_connectivity/tree_decomposition.hpp"
int main() {
    cp::Graph<int> g(4);
    for(int v=0;v<4;++v) g.add_edge(v,(v+1)%4);
    auto r=cp::minimum_tree_decomposition(g);
    std::cout << r.width << ' ' << r.bags.size() << '\n';
}
