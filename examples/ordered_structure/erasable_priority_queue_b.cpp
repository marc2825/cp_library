#include <iostream>
#include "ordered_structure/erasable_priority_queue.hpp"
int main() {
    cp::ErasablePriorityQueue<int> s; s.push(4); s.push(9); s.erase(9);
    std::cout << s.top() << "\n";
}
