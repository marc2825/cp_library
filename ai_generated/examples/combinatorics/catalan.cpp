#include <iostream>
#include "combinatorics/catalan.hpp"
int main() {
    cp::FactorialTable table(10);
    for (int n = 0; n <= 5; ++n) std::cout << cp::catalan(table, n) << ' ';
    std::cout << '\n';
    cp::PascalBinomial composite(10, 12);
    std::cout << cp::catalan(composite, 4) << '\n';
}
