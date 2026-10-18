#include <iostream>
#include <vector>
#include "../../convolution/multidimensional_convolution.hpp"

int main() {
    auto c = cp::multidimensional_convolution(std::vector<int>{1, 2, 3, 4}, {2, 2}, std::vector<int>{1, 1}, {1, 2});
    std::cout << c.shape[0] << " " << c.shape[1] << " " << c.coefficients[1] << "\n";
}
