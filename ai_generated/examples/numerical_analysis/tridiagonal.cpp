#include "numerical_analysis/tridiagonal.hpp"
#include <iomanip>
#include <iostream>
using namespace cp::numerical;
int main() {
    std::cout << std::setprecision(6);
    TridiagonalFactorization factor({1, 1}, {0, 2, 2}, {1, 1});
    const auto result = factor.solve({2, 8, 8});
    for (std::size_t i = 0; i < result.solution.size(); ++i)
        std::cout << (i ? " " : "") << result.solution[i];
    std::cout << "\n";
}
