#include <iostream>
#include "convolution/convolution_mod.hpp"
int main() {
    auto c = cp::convolution_mod(std::vector<int>{-1, 2}, std::vector<int>{3, 4}, 1000000007);
    for (int x : c) std::cout << x << ' ';
    std::cout << '\n';
}
