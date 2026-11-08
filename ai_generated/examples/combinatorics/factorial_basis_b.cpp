#include <iostream>
#include "combinatorics/factorial_basis.hpp"
int main() {
    auto b=cp::power_to_factorial_basis(std::vector<long long>{0,0,1});
    std::cout << b[1] << ' ' << b[2] << '\n';
}
