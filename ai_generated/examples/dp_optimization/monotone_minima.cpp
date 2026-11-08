#include <iostream>
#include "dp_optimization/monotone_minima.hpp"
int main() {
    auto a = cp::monotone_minima(4, 6, [](int r, int c) { return (r - c) * (r - c); });
    for (int c : a) std::cout << c << ' ';
    std::cout << '\n';
}
