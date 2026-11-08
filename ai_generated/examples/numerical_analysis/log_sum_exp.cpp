#include "numerical_analysis/stable_arithmetic.hpp"
#include <iomanip>
#include <iostream>
using namespace cp::numerical;
int main() {
    std::cout << std::setprecision(6);
    std::cout << log_sum_exp({1000, 1000}) << "\n";
}
