#include <iostream>
#include "basic_utility/fraction_binary_search.hpp"
int main() {
    auto result=cp::fraction_binary_search(10,[](auto p,auto q){ return p*p<=2*q*q; });
    std::cout << result.first.numerator << "/" << result.first.denominator << "\n";
}
