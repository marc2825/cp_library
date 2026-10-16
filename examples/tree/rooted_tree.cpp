#include <iostream>
#include "tree/rooted_tree.hpp"
int main() {
    cp::Graph<> g(4);
    g.add_edge(0, 1); g.add_edge(1, 2); g.add_edge(1, 3);
    cp::RootedTree<> tree(g);
    auto [l, r] = tree.subtree_interval(1);
    std::cout << tree.depth[2] << ' ' << tree.subtree_size[1] << ' ' << l << ' ' << r << '\n';
    for (int v : tree.postorder) std::cout << v << ' ';
    std::cout << '\n';
}
