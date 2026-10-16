#include <iostream>
#include "combinatorics/factorials.hpp"
int main() {
    cp::FactorialTable table(6, 1000000007);
    std::cout << table.factorial(5) << '\n';
    std::cout << table.inverse_factorial(5) << '\n';
    std::cout << cp::factorials(6, 12)[6] << '\n';
}
