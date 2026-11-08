#include <iostream>
#include "ordered_structure/double_ended_priority_queue.hpp"
int main() {
    cp::DoubleEndedPriorityQueue<int> q;
    for (int x : {4,1,4,9}) q.push(x);
    std::cout << q.min() << " " << q.max() << "\n";
    q.pop_max(); q.pop_min();
    std::cout << q.min() << " " << q.max() << " " << q.size() << "\n";
}
