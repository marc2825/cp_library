#include <iostream>
#include "../../tree/dynamic_top_tree.hpp"
#include "../../tree/fully_dynamic_tree_dp.hpp"
int main() {
    cp::DynamicTopTree<cp::DistanceSumTreeDP<long long>> tree({1, 2, 3, 4});
    tree.link(0, 1); tree.link(1, 2); tree.link(1, 3);
    std::cout << tree.component_cluster(0).top << '\n';
    tree.cut(1, 3); tree.link(2, 3);
    std::cout << tree.component_cluster(0).top << '\n';
    std::cout << tree.subtree_cluster(0, 2)->mass << '\n';
}
