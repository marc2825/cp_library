#include <iostream>
#include "number_theory_basic/mod_sum_polynomial.hpp"

int main() {
    std::cout << cp::polynomial_mod_sum(0, 10, 7, 3, 11, 1, 2) << '\n';
}
