#include <iostream>
#include "set_function/subset_convolution.hpp"
int main() {
    auto c = cp::subset_convolution(std::vector<long long>{1, 2, 3, 4}, std::vector<long long>{5, 6, 7, 8});
    for (auto x : c) std::cout << x << ' ';
    std::cout << '\n';
}
