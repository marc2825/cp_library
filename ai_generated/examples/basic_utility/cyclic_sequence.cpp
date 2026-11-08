#include <iostream>
#include "basic_utility/cyclic_sequence.hpp"
int main() {
    std::vector<int> a{1,2,3}; cp::rotate_left(a,-1);
    for(int x:cp::cyclic_slice(a,2,4)) std::cout<<x;
    std::cout<<"\n";
}
