#include "numerical_analysis/polynomial_roots.hpp"
#include <iomanip>
#include <iostream>
using namespace cp::numerical;
int main() {
    std::cout << std::setprecision(6);
    const auto result = real_polynomial_roots({0, -2, 1, 1});
    for (std::size_t i = 0; i < result.intervals.size(); ++i)
        std::cout << (i ? " " : "") << result.intervals[i].value;
    std::cout << "\n";
}
