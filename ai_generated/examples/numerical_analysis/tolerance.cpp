#include "numerical_analysis/common.hpp"
#include <iomanip>
#include <iostream>
using namespace cp::numerical;
int main() {
    std::cout << std::setprecision(6);
    std::cout << almost_equal(1, 1.0000001L, {1e-8L, 1e-6L}) << "\n";
}
