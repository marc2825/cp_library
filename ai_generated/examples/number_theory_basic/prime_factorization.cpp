#include <iostream>
#include "number_theory_basic/prime_factorization.hpp"
int main() {
    for (auto [prime, exponent] : cp::prime_factorization(360))
        std::cout << prime << '^' << exponent << ' ';
    std::cout << '\n';
}
