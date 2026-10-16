#include <iostream>
#include "number_theory_basic/smallest_prime_factor.hpp"
int main() {
    cp::SmallestPrimeFactor spf(100);
    std::cout << spf[84] << '\n';
    for (auto [prime, exponent] : spf.factorize(84))
        std::cout << prime << '^' << exponent << ' ';
    std::cout << '\n';
}
