#include <iostream>
#include "number_theory_convolution/pollard_rho.hpp"
int main() {
    for (auto p : cp::factorize_64(1000000016000000063ULL)) std::cout << p << ' ';
    std::cout << '\n';
}
