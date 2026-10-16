#include <iostream>
#include "combinatorics/permutation_combination.hpp"
int main() {
    cp::FactorialTable table(10);
    std::cout << cp::permutations(table, 5, 3) << '\n';
    std::cout << cp::combinations(table, 5, 3) << '\n';
}
