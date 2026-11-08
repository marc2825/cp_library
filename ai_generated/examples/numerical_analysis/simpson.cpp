#include "numerical_analysis/integration.hpp"
#include <iomanip>
#include <iostream>
using namespace cp::numerical;
int main() {
    std::cout << std::setprecision(6);
    auto result = simpson([](long double x) { return x * x * x; }, 0, 1, 8);
    std::cout << result.value << "\n";
}
