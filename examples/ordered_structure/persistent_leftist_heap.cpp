#include <functional>
#include <iostream>
#include "ordered_structure/persistent_leftist_heap.hpp"
int main() {
    cp::PersistentLeftistHeap<int,std::greater<int>> h;
    int a=h.push(0,4), b=h.push(a,1), c=h.push(a,7);
    int d=h.meld(b,c), next=h.pop(d);
    std::cout << h.top(a) << " " << h.top(b) << " " << h.top(c) << "\n";
    std::cout << h.size(d) << " " << h.top(next) << "\n";
}
