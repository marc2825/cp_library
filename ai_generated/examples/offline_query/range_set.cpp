#include <iostream>
#include "offline_query/range_set.hpp"
int main() {
    cp::RangeSet<> s; s.insert(1,8); s.erase(3,5);
    for(auto [l,r]:s.intervals()) std::cout<<l<<":"<<r<<" ";
    std::cout<<"\n";
}
