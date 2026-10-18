#include <iostream>
#include <vector>
#include "../../convolution/gcd_lcm_convolution.hpp"

int main() {
    auto c = cp::gcd_convolution(std::vector<int>{0, 1, 2}, std::vector<int>{0, 3, 4});
    std::cout << c[1] << " " << c[2] << "\n";
}
