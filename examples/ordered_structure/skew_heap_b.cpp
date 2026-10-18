#include <iostream>
#include "ordered_structure/skew_heap.hpp"
int main() {
    cp::SkewHeap<int> a,b; a.push(2); b.push(7); a.meld(b);
    std::cout << a.top() << " " << b.empty() << "\n";
}
