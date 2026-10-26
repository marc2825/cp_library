#include <iostream>
#include "number_theory_basic/batch_inversion.hpp"

int main() {
    const auto inverses = cp::batch_inverse_mod({3, 5, 7}, 16);
    for (auto x : *inverses) std::cout << x << ' ';
    std::cout << '\n';
}
