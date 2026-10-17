#include <iostream>
#include "ordered_structure/persistent_queue.hpp"
int main() {
    cp::PersistentQueue<int> q; int a=q.push(0,4),b=q.push(a,9),c=q.pop(b);
    std::cout<<q.front(a)<<" "<<q.front(c)<<"\n";
}
