#include <iostream>
#include "number_theory_convolution/min25_sieve.hpp"

int main() {
    const auto totient = cp::min25_sieve<cp::BigInt>(100, {-1, 1},
        [](auto p, unsigned, auto power) { return cp::BigInt(power / p * (p - 1)); });
    std::cout << totient.total() << '\n';
}
