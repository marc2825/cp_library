#include <iostream>
#include "number_theory_basic/continued_fraction.hpp"

int main() {
    const cp::BigRational value(355, 113);
    for (const auto& a : cp::continued_fraction(value)) std::cout << a << ' ';
    std::cout << '\n' << cp::best_rational_approximation(value, 100) << '\n';
}
