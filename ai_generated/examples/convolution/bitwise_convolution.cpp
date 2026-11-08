#include <iostream>
#include "convolution/bitwise_convolution.hpp"
int main() {
    const std::vector<long long> a{1, 2}, b{3, 4};
    for (auto x : cp::bitwise_and_convolution(a, b)) std::cout << x << ' ';
    std::cout << '\n';
    for (auto x : cp::bitwise_or_convolution(a, b)) std::cout << x << ' ';
    std::cout << '\n';
    for (auto x : cp::bitwise_xor_convolution(a, b)) std::cout << x << ' ';
    std::cout << '\n';
}
