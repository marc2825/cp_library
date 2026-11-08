#include <iostream>
#include "combinatorics/binomial.hpp"
int main() {
    cp::FactorialTable fast(10);
    std::cout << cp::binomial(fast, 5, 2) << '\n';
    cp::PascalBinomial any_mod(10, 12);
    std::cout << cp::binomial(any_mod, 6, 3) << '\n';
}
