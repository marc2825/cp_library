#include "convolution/multidimensional_cyclic_convolution.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}
int main() { print(cp::multidimensional_cyclic_convolution({1, 2, 3, 4}, {1, 0, 0, 1}, {2, 2})); }
