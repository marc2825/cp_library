#include <iostream>
#include "static_query/ordered_set.hpp"
int main() {
    cp::OrderedSet<int> s; s.insert(9); s.insert(2);
    std::cout<<*s.kth(0)<<"\n";
}
