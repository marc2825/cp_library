#include <iostream>
#include "graph_connectivity/tree_decomposition_width_2.hpp"
int main() {
    cp::Graph<int> g(4); for(int i=0;i<4;++i) g.add_edge(i,(i+1)%4);
    auto d=cp::tree_decomposition_width_2(g);
    std::cout << bool(d) << ' ' << d->bags.size() << '\n';
}
