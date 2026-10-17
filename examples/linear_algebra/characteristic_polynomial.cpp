#include <iostream>
#include "linear_algebra/characteristic_polynomial.hpp"
int main() {
    for (auto x : cp::characteristic_polynomial_mod(cp::Matrix<int>{{1, 2}, {3, 4}})) std::cout << x << ' ';
    std::cout << '\n';
}
