#include <iostream>
#include "linear_algebra/xor_basis.hpp"
int main() {
    cp::XorBasis<> basis;
    basis.insert(3); basis.insert(5);
    std::cout << basis.insert(6) << ' ' << basis.rank() << '\n';
    std::cout << basis.contains(6) << ' ' << basis.contains(1) << '\n';
    std::cout << basis.min_xor(1) << ' ' << basis.max_xor() << '\n';
}
