#include <iostream>
#include "basic_utility/quotient_enumeration.hpp"
int main() {
    cp::for_each_quotient(5,[](auto l,auto r,auto q){std::cout<<l<<":"<<r<<"="<<q<<" ";});
    std::cout<<"\n";
}
