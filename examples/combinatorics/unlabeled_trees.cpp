#include <iostream>
#include "combinatorics/unlabeled_trees.hpp"
int main() {
    const auto counts = cp::unlabeled_tree_counts(8);
    for (int i = 1; i <= 8; ++i) std::cout << counts.rooted[i] << (i == 8 ? '\n' : ' ');
    for (int i = 1; i <= 8; ++i) std::cout << counts.unrooted[i] << (i == 8 ? '\n' : ' ');
}
