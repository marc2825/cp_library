#include "numerical_analysis/quadratic.hpp"
#include <iomanip>
#include <iostream>
using namespace cp::numerical;
int main() {
    std::cout << std::setprecision(6);
    const auto result = quadratic_roots(1, -5, 6);
    for (std::size_t i = 0; i < result.roots.size(); ++i)
        std::cout << (i ? " " : "") << result.roots[i];
    std::cout << "\n";
}
