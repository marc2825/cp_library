#include <iostream>
#include "graph_connectivity/three_edge_connected_components.hpp"
int main() {
    cp::Graph<int> g(2); for(int i=0;i<3;++i) g.add_edge(0,1);
    auto c=cp::three_edge_connected_components(g);
    std::cout << c.groups.size() << '\n';
}
