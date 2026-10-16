#include <iostream>
#include "dp_optimization/min_plus_convolution.hpp"
int main() {
    auto c = cp::min_plus_convolution_arbitrary_convex(std::vector<long long>{3, 0, 4}, std::vector<long long>{0, 1, 4});
    for (auto x : c) std::cout << x << ' ';
    std::cout << '\n';
}
