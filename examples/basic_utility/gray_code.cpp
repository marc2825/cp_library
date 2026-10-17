#include <iostream>
#include "basic_utility/gray_code.hpp"
int main() {
    cp::for_each_gray_code(2,[](auto x){std::cout<<x<<" ";});
    std::cout<<cp::gray_decode(7)<<"\n";
}
