#include "linear_algebra/determinant_any_mod.hpp"
#include "linear_algebra/determinant_integer.hpp"
#include <iostream>
int main() {
    std::size_t cases;
    std::cin >> cases;
    while (cases--) {
        std::size_t n;
        std::uint64_t modulus;
        std::cin >> n >> modulus;
        cp::Matrix<cp::BigInt> a(n, n);
        for (std::size_t i = 0; i < n; ++i)
            for (std::size_t j = 0; j < n; ++j)
                std::cin >> a(i, j);
        std::cout << cp::determinant_integer(a) << ' ' << cp::determinant_any_mod(a, modulus) << '\n';
    }
}
