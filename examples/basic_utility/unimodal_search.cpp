#include <iostream>
#include "basic_utility/unimodal_search.hpp"
#include <cmath>
#include <iomanip>
int main() {
    auto integer = cp::fibonacci_search(-10, 11, [](std::int64_t x) { return std::abs(x - 3); });
    auto real = cp::golden_section_search(-10, 10, [](long double x) { return std::abs(x - 3); });
    std::cout << integer.position << ' ' << integer.value << '\n';
    std::cout << std::fixed << std::setprecision(6) << real.position << '\n';
}
