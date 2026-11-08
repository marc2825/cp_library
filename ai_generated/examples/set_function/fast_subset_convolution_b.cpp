#include <iostream>
#include <vector>
#include "../../set_function/fast_subset_convolution.hpp"

int main() {
    auto c = cp::subset_convolution_mod(std::vector<int>{1, 2, 3, 4}, std::vector<int>{1, 1, 1, 1});
    std::cout << c[3] << "\n";
}
