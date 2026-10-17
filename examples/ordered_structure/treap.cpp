#include <iostream>
#include "ordered_structure/treap.hpp"
int main() {
    cp::Treap<int> s; s.insert(5); s.insert(1); s.insert(3);
    std::cout<<*s.kth(1)<<" "<<s.order_of_key(5)<<"\n";
}
