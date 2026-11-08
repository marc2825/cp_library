#include <iostream>
#include "basic_utility/multiple_enumeration.hpp"
int main() {
    cp::for_each_multiple(0,14,5,[](auto x){std::cout<<x<<" ";});
    std::cout<<"\n";
}
