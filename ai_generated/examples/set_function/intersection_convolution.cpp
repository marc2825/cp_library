#include "set_function/intersection_convolution.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}
int main() {
    auto result = cp::intersection_size_convolution({1, 2, 3, 4}, {1, 1, 1, 1});
    for (const auto &row : result)
        print(row);
}
