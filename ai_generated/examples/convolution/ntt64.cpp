#include "convolution/ntt64.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}
int main() { print(cp::convolution_ntt64({cp::ntt64_modulus - 1, 2}, {2, 3})); }
