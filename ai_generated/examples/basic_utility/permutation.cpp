#include <iostream>
#include "basic_utility/permutation.hpp"
int main() {
    std::vector<int> p{2, 0, 1, 4, 3};
    auto power = cp::permutation_power(p, -2);
    for (std::size_t i = 0; i < power.size(); ++i) std::cout << (i ? " " : "") << power[i];
    std::cout << '\n' << cp::permutation_sign(p) << '\n';
}
