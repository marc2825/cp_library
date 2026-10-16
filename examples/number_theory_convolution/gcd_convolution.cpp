#include <iostream>
#include <vector>
#include "number_theory_convolution/gcd_convolution.hpp"
int main() {
    std::vector<long long> a{0, 1, 2, 3, 4, 5, 6}, b{0, 6, 5, 4, 3, 2, 1};
    const auto c = cp::gcd_convolution(a, b);
    for (std::size_t i = 1; i < c.size(); ++i) std::cout << c[i] << ' ';
    std::cout << '\n';
}
