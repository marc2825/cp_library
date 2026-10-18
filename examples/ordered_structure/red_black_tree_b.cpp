#include <iostream>
#include "ordered_structure/red_black_tree.hpp"
int main() {
    cp::RedBlackTree<int> s; s.insert(4); s.insert(1); s.insert(4); s.erase(4);
    std::cout << *s.kth(1) << "\n";
}
