#include <iostream>
#include "combinatorics/narayana.hpp"
int main() {
    cp::FactorialTable table(5); std::cout << cp::narayana(table,5,3) << '\n';
}
