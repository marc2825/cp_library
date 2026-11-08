#include <iostream>
#include <vector>
#include "number_theory_basic/euler_phi.hpp"

int main() {
    auto values=cp::euler_phi_sieve(10);
    std::cout << cp::euler_phi(36) << " " << values[10] << "\n";
}
