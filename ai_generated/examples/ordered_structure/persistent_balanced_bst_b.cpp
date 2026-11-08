#include <iostream>
#include "ordered_structure/persistent_balanced_bst.hpp"
int main() {
    cp::PersistentBalancedBST<int> a; auto b=a.insert(3); auto c=b.insert(1);
    std::cout << b.size() << " " << *c.kth(0) << "\n";
}
