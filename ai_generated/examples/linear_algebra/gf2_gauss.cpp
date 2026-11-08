#include <iostream>
#include "linear_algebra/gf2_gauss.hpp"
int main() {
    const std::vector<std::bitset<8>> a{std::bitset<8>(3), std::bitset<8>(6)};
    const auto r = cp::gf2_gauss_jordan(a, std::vector<int>{1, 0}, 3);
    std::cout << r.rank << ' ' << r.nullspace_basis.size() << '\n';
    for (std::size_t j = 0; j < 3; ++j) std::cout << r.particular[j];
    std::cout << '\n';
}
