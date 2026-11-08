#include <iostream>
#include "tree/virtual_tree.hpp"
int main() {
    cp::Graph<int> g(5); g.add_edge(0, 1); g.add_edge(1, 2); g.add_edge(1, 3); g.add_edge(3, 4);
    cp::VirtualTree<int> tree(g); auto v = tree.build({2, 4, 4});
    for (int i = 0; i < int(v.vertices.size()); ++i) std::cout << v.vertices[i] << ':' << v.parent[i] << ' ';
    std::cout << '\n';
}
