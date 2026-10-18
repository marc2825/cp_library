#include <iostream>
#include "ordered_structure/radix_heap.hpp"
int main() {
    cp::RadixHeap<unsigned long long,int> s; s.push(3,10); s.push(8,20); s.pop();
    std::cout << s.top_key() << "\n";
}
