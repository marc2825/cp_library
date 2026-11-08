#include <iostream>
#include "linear_algebra/matrix_power.hpp"
int main() {
    cp::Matrix<long long> a{{1, 1}, {1, 0}};
    auto power = cp::matrix_power(a, 10);
    std::cout << power(0, 0) << ' ' << power(0, 1) << '\n';
    std::cout << cp::matrix_power(a, 0)(1, 1) << '\n';
}
