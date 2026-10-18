#include <iostream>
#include <vector>
#include "../../convolution/min_plus_convolution.hpp"

int main() {
    auto c = cp::min_plus_convolution_convex_convex(std::vector<int>{0, 1, 4}, std::vector<int>{0, 2, 7});
    std::cout << c[2] << "\n";
}
