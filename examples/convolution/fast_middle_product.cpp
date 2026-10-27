#include "convolution/fast_middle_product.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}
int main() { print(cp::fast_middle_product({1, 2, 3, 4, 5}, {2, 1})); }
