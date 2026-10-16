#include <iostream>
#include "tree/diameter.hpp"
int main() {
    cp::Graph<> g(4);
    g.add_edge(0, 1, 2); g.add_edge(1, 2, 3); g.add_edge(1, 3, 5);
    auto diameter = cp::tree_diameter(g);
    std::cout << diameter.length << ' ' << diameter.vertices.size() << '\n';
}
