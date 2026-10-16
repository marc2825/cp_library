#include <iostream>
#include "number_theory_basic/gcd_lcm.hpp"
int main() {
    std::cout << cp::gcd(-18, 24) << '\n';
    if (auto value = cp::lcm(-18, 24)) std::cout << *value << '\n';
}
