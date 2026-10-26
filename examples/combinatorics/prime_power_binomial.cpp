#include <iostream>
#include "combinatorics/prime_power_binomial.hpp"

int main() {
    cp::PrimePowerBinomial binomial(72);
    std::cout << binomial.choose(100, 30) << '\n';
}
