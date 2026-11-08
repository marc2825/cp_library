#include <iostream>
#include "../../ordered_structure/partially_retroactive_priority_queue.hpp"
int main(){
    cp::PartiallyRetroactivePriorityQueue<int> queue({1,3,4,8});
    queue.insert(1,5); queue.insert(4,2); queue.insert_delete_min(8);
    std::cout<<*queue.min()<<' '<<queue.size()<<'\n';
    queue.insert(3,1);
    std::cout<<*queue.min()<<' '<<queue.size()<<'\n';
    queue.erase(8);
    std::cout<<*queue.min()<<' '<<queue.size()<<'\n';
}
