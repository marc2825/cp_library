#include "numerical_analysis/integration.hpp"
#include <iomanip>
#include <iostream>
using namespace cp::numerical;
int main() {
    std::cout << std::setprecision(6);
    auto result = adaptive_simpson([](long double x) { return std::exp(x); }, 0, 1);
    std::cout << result.value << "\n";
}
