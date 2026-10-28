#include "numerical_analysis/root_finding.hpp"
#include <iomanip>
#include <iostream>
using namespace cp::numerical;
int main() {
    std::cout << std::setprecision(6);
    RootOptions options;
    options.value_tolerance = 1e-12L;
    auto f = [](long double x) { return x * x - 2; };
    const auto result = newton(f, [](long double x) { return 2 * x; }, 1, options);
    std::cout << result.root << "\n";
}
