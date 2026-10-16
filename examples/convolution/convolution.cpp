#include <iostream>
#include <vector>
#include "convolution/convolution.hpp"
int main() {
    const std::vector<long long> a{1, -2, 3}, b{4, 5};
    for (int x : cp::convolution(a, b)) std::cout << x << ' ';
    std::cout << '\n';
    for (int x : cp::convolution<17, 3>(a, b)) std::cout << x << ' ';
    std::cout << '\n';
}
