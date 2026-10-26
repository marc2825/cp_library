#include <iostream>
#include "number_theory_basic/floor_sum_polynomial.hpp"

int main() {
    const auto moments = cp::floor_sum_moments(6, 5, 3, 1, 2);
    std::cout << moments[0][1] << ' ' << moments[1][1] << ' ' << moments[0][2] << '\n';
}
