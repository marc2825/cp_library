#include <iostream>
#include "ordered_structure/weight_balanced_tree.hpp"
int main() {
    cp::WeightBalancedTree<int> s; s.insert(4); s.insert(1); s.insert(4); s.erase(4);
    std::cout << *s.kth(1) << "\n";
}
