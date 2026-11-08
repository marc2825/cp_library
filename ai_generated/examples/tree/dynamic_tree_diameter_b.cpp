#include <iostream>
#include "tree/dynamic_tree_diameter.hpp"
int main() {
    cp::Graph<long long> g(3); g.add_edge(0,1,2); g.add_edge(1,2,3);
    cp::DynamicTreeDiameter<> d(g); d.set_edge(0,7);
    std::cout << d.diameter()->length << '\n';
}
