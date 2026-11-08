#include <iostream>
#include "graph_basic/minimum_diameter_spanning_tree.hpp"
int main() {
    cp::Graph<int> g(3);
    g.add_edge(0,1,1); g.add_edge(1,2,1); g.add_edge(0,2,1);
    auto r=cp::minimum_diameter_spanning_tree(g);
    std::cout << cp::optimization_to_string(r->diameter) << ' ' << r->edge_ids.size() << '\n';
}
