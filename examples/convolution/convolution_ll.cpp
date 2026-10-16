#include <iostream>
#include <vector>
#include "convolution/convolution_ll.hpp"
int main() {
    const std::vector<long long> a{1000000000, -2, 3}, b{4, 5};
    for (auto x : cp::convolution_ll(a, b)) std::cout << x << ' ';
    std::cout << '\n';
}
