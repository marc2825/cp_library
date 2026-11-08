#include <iostream>
#include "number_theory_basic/mod_inverse.hpp"
int main() {
    if (auto inverse = cp::mod_inverse(-3, 10)) std::cout << *inverse << '\n';
    std::cout << (cp::mod_inverse(6, 10).has_value() ? "exists" : "none") << '\n';
}
