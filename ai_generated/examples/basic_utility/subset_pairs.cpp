#include <iostream>
#include "basic_utility/subset_pairs.hpp"
int main() {
    int count=0; cp::for_each_subset_pair(3,[&](auto s,auto t){if((s&t)==t) ++count;});
    std::cout<<count<<"\n";
}
