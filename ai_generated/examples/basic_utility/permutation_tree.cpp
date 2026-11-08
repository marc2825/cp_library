#include <iostream>
#include "basic_utility/permutation_tree.hpp"
int main() {
    cp::PermutationTree tree({1, 0, 3, 5, 2, 4});
    const auto& root = tree.nodes()[tree.root()];
    std::cout << root.left << ' ' << root.right << ' ' << root.children.size() << '\n';
    std::cout << tree.nodes().size() << ' ' << tree.common_interval_count() << '\n';
}
