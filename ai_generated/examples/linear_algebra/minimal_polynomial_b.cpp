#include <iostream>
#include <vector>
#include "../../linear_algebra/minimal_polynomial.hpp"

int main() {
    auto p = cp::matrix_minimal_polynomial_mod(cp::Matrix<int>{{0, 1}, {0, 0}});
    std::cout << p[0] << " " << p[1] << " " << p[2] << "\n";
}
