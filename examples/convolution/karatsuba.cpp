#include <iostream>
#include "convolution/karatsuba.hpp"
int main() {
    const auto c = cp::convolution_karatsuba(std::vector<long long>{1, 2, 3}, std::vector<long long>{4, 5});
    for (auto x : c) std::cout << x << ' ';
    std::cout << '\n';
}
