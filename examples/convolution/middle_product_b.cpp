#include <iostream>
#include <vector>
#include "../../convolution/middle_product.hpp"

int main() {
    auto c = cp::middle_product(std::vector<int>{1, 2, 3, 4}, std::vector<int>{2, 1});
    std::cout << c[0] << " " << c[2] << "\n";
}
