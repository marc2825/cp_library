#include <iostream>
#include <vector>
#include "number_theory_basic/mobius.hpp"

int main() {
    auto values=cp::mobius_sieve(12);
    std::cout << cp::mobius(30) << " " << values[12] << "\n";
}
