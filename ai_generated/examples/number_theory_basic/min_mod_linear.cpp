#include <iostream>
#include "number_theory_basic/min_mod_linear.hpp"

int main() {
    const auto result = cp::min_mod_linear(0, 10, 7, 3, 11);
    std::cout << result->value << ' ' << result->index << '\n';
}
