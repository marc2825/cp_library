#include <iostream>
#include "dp_optimization/smawk.hpp"
int main() {
    auto a = cp::smawk(4, 6, [](int r, int c) { return (r - c) * (r - c); });
    for (int c : a) std::cout << c << ' ';
    std::cout << '\n';
}
