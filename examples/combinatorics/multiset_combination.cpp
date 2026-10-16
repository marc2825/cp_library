#include <iostream>
#include "combinatorics/multiset_combination.hpp"
int main() {
    cp::FactorialTable table(10);
    std::cout << cp::multiset_combination(table, 3, 4) << '\n';
    std::cout << cp::multiset_combination(table, 0, 0) << ' '
              << cp::multiset_combination(table, 0, 4) << '\n';
}
